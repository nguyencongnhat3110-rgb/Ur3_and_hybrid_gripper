import os
import json
import base64
import cv2
import time
import pandas as pd
from openai import OpenAI

# --- IMPORT MODULE CAMERA 3D ---
try:
    from realsense_brain import RealSenseCamera
    HAS_REALSENSE = True
except ImportError:
    print("⚠️ Không tìm thấy module realsense_brain. Chuyển sang chế độ Webcam thường.")
    HAS_REALSENSE = False

# --- CẤU HÌNH HỆ THỐNG ---
api_key = os.getenv("OPENAI_API_KEY")
client = OpenAI(api_key=OPENAI_API_KEY)
MODEL = "gpt-5.2"

# --- XỬ LÝ ĐƯỜNG DẪN ---
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
KB_FILE_1 = os.path.join(CURRENT_DIR, "physical_knowledge.csv")       
KB_FILE_2 = os.path.join(CURRENT_DIR, "object_folder_knowledge.csv")   
KB_FILE_3 = os.path.join(CURRENT_DIR, "visgel_knowledge.csv")          

print(f"📂 Path A: {KB_FILE_1}")
print(f"📂 Path B: {KB_FILE_2}")
print(f"📂 Path C: {KB_FILE_3}")

def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def load_knowledge_base(file_path, source_name):
    try:
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)
            # Lấy mẫu nhiều hơn để AI có thêm ngữ cảnh
            sample = df.sample(min(len(df), 20)).to_string(index=False)
            return f"--- SOURCE {source_name} (Reference Data) ---\n{sample}\n"
        else:
            return f"--- SOURCE {source_name}: NOT FOUND ---\n"
    except Exception as e:
        return f"--- SOURCE {source_name}: ERROR {e} ---\n"

def get_grasp_parameters():
    global HAS_REALSENSE 
    print(f"\n📥 [{MODEL}] Kích hoạt Vision Brain (5-Level Pressure & Texture Analysis)...")
    
    # 1. Nạp tri thức
    kb1_content = load_knowledge_base(KB_FILE_1, "A (Local Lab Data)")
    kb2_content = load_knowledge_base(KB_FILE_2, "B (ObjectFolder 2.0)")
    kb3_content = load_knowledge_base(KB_FILE_3, "C (VisGel Dataset)")

    # 2. Thu thập dữ liệu từ Camera 3D
    measured_width_mm = 0.0
    frame = None
    
    if HAS_REALSENSE:
        try:
            rs_cam = RealSenseCamera()
            for _ in range(10): rs_cam.get_data() # Warm up
            frame, measured_width_mm, dist_mm = rs_cam.get_data()
            rs_cam.stop()
            if frame is not None:
                print(f"📏 [HARDWARE]: Đo được vật rộng = {measured_width_mm:.1f} mm")
            else:
                raise Exception("Frame is None")
        except Exception as e:
            print(f"❌ Lỗi RealSense: {e}. Fallback Webcam...")
            HAS_REALSENSE = False

    if frame is None:
        cap = cv2.VideoCapture(0)
        for _ in range(5): cap.read()
        ret, frame = cap.read()
        cap.release()
        measured_width_mm = 45.0 # Giá trị giả định an toàn (trung bình)
        print("⚠️ Dùng Webcam thường. Giả định vật rộng 45mm.")

    # 3. Resize ảnh
    scale_percent = 512 / frame.shape[1]
    width = int(frame.shape[1] * scale_percent)
    height = int(frame.shape[0] * scale_percent)
    dim = (width, height)
    resized_frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
    
    image_path = "current_scene.jpg"
    cv2.imwrite(image_path, resized_frame)
    base64_image = encode_image(image_path)

    # 4. SYSTEM PROMPT (CẬP NHẬT: 3 NGUỒN DATA + 5 MỨC ÁP SUẤT)
    system_prompt = f"""
    [SYSTEM ROLE]: Expert in Tribology (Friction) and Robotic Manipulation.
    [OBJECTIVE]: Analyze visual texture and material to determine Gripper Parameters (M, P).

    [CONTEXT - KNOWLEDGE BASES]:
    You MUST search these datasets for similar objects/materials before deciding:
    {kb1_content}
    {kb2_content}
    {kb3_content}

    [STEP 1: VISUAL TEXTURE ANALYSIS]:
    - **Shininess/Specular Highlights:** Look for white reflection spots. 
      -> If YES: Surface is Smooth/Glass/Metal/Plastic -> **LOW Friction** -> Needs **HIGHER P**.
    - **Roughness/Matte:** Look for fuzzy edges, fabric patterns, or porous texture.
      -> If YES: Surface is Rough/Foam/Wood/Fabric -> **HIGH Friction** -> Needs **LOWER P**.
    - **Deformability:** Does it look soft (sponge/plush)? Or rigid?

    [STEP 2: DETERMINE PRESSURE (P) - 5 LEVELS]:
    **Range: 0 - 150 kPa.**
    
    * **LEVEL 1: Very Low (20-40 kPa)**
        - Object: Extremely High Friction (Sandpaper, Rubber) OR Very Light & Fragile (Empty paper cup).
        - Logic: Friction holds it, high pressure isn't needed.

    * **LEVEL 2: Low (50-70 kPa)**
        - Object: Soft/Deformable but needs grip (Sponge, Plush Toy) OR Fragile Food (Bread, Tofu).
        - Logic: Grip firmly but DO NOT crush.

    * **LEVEL 3: Medium (80-100 kPa)**
        - Object: Standard Rigid Objects (Wood block, Cardboard box, Hard Plastic).
        - Logic: Balance between grip and safety.

    * **LEVEL 4: High (110-130 kPa)**
        - Object: Heavy items OR Slightly slippery (Polished Wood, Heavy Book).
        - Logic: Weight requires more force.

    * **LEVEL 5: Very High (140-150 kPa)**
        - Object: **Metal, Glass, Ceramics, Polished Stone.**
        - Visual Cue: Shiny, Reflective surfaces.
        - Logic: Low Friction (coeff < 0.3) requires MAX pressure to prevent slip.

    [STEP 3: DETERMINE MOVEMENT (M) - LIMIT -90 to 0]:
    - Base: M = -(90 - Measured_Width).
    - Compliance Offset:
        - Rigid/Fragile: Add -2mm (Firm touch).
        - Soft/Sponge: Add -15mm to -20mm (Deep squeeze).
    - *Note: AI output can be calculated freely, code will clamp it strictly to -90.*

    [OUTPUT FORMAT]:
    Return ONLY a JSON object:
    {{
        "label": "string (Object Name)",
        "material_analysis": "string (e.g., Shiny/Smooth Metal or Matte/Rough Foam)",
        "friction_estimation": "Very Low/Low/Medium/High/Very High",
        "M": float,
        "P": int,
        "reasoning": "Explain P based on shininess/friction and M based on width."
    }}
    """

    try:
        print(f"⏳ Đang gửi request tới {MODEL}...")
        response = client.chat.completions.create(
            model=MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": f"Object Width: {measured_width_mm}mm. Analyze texture, consult KBs, and determine M, P."},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                    ],
                }
            ],
            response_format={"type": "json_object"},
            max_completion_tokens=400
        )
        
        content = response.choices[0].message.content
        data = json.loads(content)
        
        # --- [SAFETY CLAMPING LOGIC - 90MM] ---
        raw_m = float(data.get('M', -20.0))
        raw_p = int(data.get('P', 50))
        label = data.get('label', 'unknown')
        reason = data.get('reasoning', 'N/A')
        mat_analysis = data.get('material_analysis', 'N/A')
        
        # 2. Kẹp giới hạn M [-90 đến 0]
        safe_m = max(-90.0, min(0.0, raw_m))
        
        # 3. Kẹp giới hạn P [0 đến 150]
        safe_p = max(0, min(150, raw_p))
        
        print(f"\n🧠 [{MODEL} DECISION]: {label.upper()}")
        print(f"👁️ Texture: {mat_analysis}")
        print(f"📉 Raw AI Output: M={raw_m}, P={raw_p}")
        print(f"🛡️ Safety Clamp: M={safe_m:.1f} mm, P={safe_p} kPa")
        print(f"📝 Reasoning: {reason}")
        
        return safe_p, safe_m, label

    except Exception as e:
        print(f"❌ Lỗi xử lý Vision: {e}")
        return 50, -20.0, "error"

if __name__ == "__main__":
    get_grasp_parameters()