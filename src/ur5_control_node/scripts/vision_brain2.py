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
OPENAI_API
client = OpenAI(api_key=OPENAI_API_KEY)
MODEL = "gpt-5.2"

# --- XỬ LÝ ĐƯỜNG DẪN ---
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
KB_FILE_1 = os.path.join(CURRENT_DIR, "physical_knowledge.csv")       
KB_FILE_2 = os.path.join(CURRENT_DIR, "object_folder_knowledge.csv")   
KB_FILE_3 = os.path.join(CURRENT_DIR, "visgel_knowledge.csv")          

def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def load_knowledge_base(file_path, source_name):
    try:
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)
            sample = df.sample(min(len(df), 20)).to_string(index=False)
            return f"--- SOURCE {source_name} (Reference Data) ---\n{sample}\n"
        else:
            return f"--- SOURCE {source_name}: NOT FOUND ---\n"
    except Exception as e:
        return f"--- SOURCE {source_name}: ERROR {e} ---\n"

def get_grasp_parameters():
    global HAS_REALSENSE 
    print(f"\n📥 [{MODEL}] Khởi chạy Vision Brain 2 (Chuyên biệt tính toán góc kẹp M)...")
    
    # 1. Nạp tri thức (Chỉ dùng để tham khảo tính chất vật liệu nhằm tính M)
    kb1_content = load_knowledge_base(KB_FILE_1, "A (Local Lab)")
    kb2_content = load_knowledge_base(KB_FILE_2, "B (ObjectFolder)")
    kb3_content = load_knowledge_base(KB_FILE_3, "C (VisGel)")

    # 2. Thu thập dữ liệu từ Camera 3D
    measured_width_mm = 0.0
    frame = None
    
    if HAS_REALSENSE:
        try:
            rs_cam = RealSenseCamera()
            for _ in range(10): rs_cam.get_data() # Warm up
            frame, measured_width_mm, dist_mm = rs_cam.get_data()
            rs_cam.stop()
            if frame is None: raise Exception("Frame is None")
        except Exception as e:
            print(f"❌ Lỗi RealSense: {e}. Fallback Webcam...")
            HAS_REALSENSE = False

    if frame is None:
        cap = cv2.VideoCapture(0)
        for _ in range(5): cap.read()
        ret, frame = cap.read()
        cap.release()
        measured_width_mm = 45.0 

    # 3. Xử lý ảnh
    scale_percent = 512 / frame.shape[1]
    resized_frame = cv2.resize(frame, (int(frame.shape[1] * scale_percent), int(frame.shape[0] * scale_percent)), interpolation=cv2.INTER_AREA)
    image_path = "experiment_scene_2.jpg"
    cv2.imwrite(image_path, resized_frame)
    base64_image = encode_image(image_path)

    # 4. SYSTEM PROMPT (TỐI GIẢN: CHỈ TẬP TRUNG TÍNH M)
    system_prompt = f"""
    [SYSTEM ROLE]: Robotics Perception Specialist.
    [OBJECTIVE]: Determine the Object Name and the Grasp Movement (M) for a 90mm gripper.
    [INPUT]: Measured Object Width = {measured_width_mm} mm.

    [STEP 1: IDENTIFICATION]:
    - Identify the object in the image. Consult these Knowledge Bases:
    {kb1_content}
    {kb2_content}
    {kb3_content}

    [STEP 2: CALCULATE M (Movement in mm)]:
    - Gripper is fully open at 0 mm (Gap = 90mm).
    - Formula: M = -(90 - Measured_Width + Compliance_Offset).
    - **Compliance_Offset Guidance:**
        * Rigid/Fragile objects (Glass, Metal, Egg, Cup): Add 3mm to 5mm offset to ensure firm contact.
        * Soft/Elastic objects (Sponge, Bread, Plush): Add 15mm to 20mm offset for deep engagement.

    [OUTPUT FORMAT]:
    Return ONLY a JSON object:
    {{
        "label": "string (Object Name)",
        "M": float (The calculated value between -90 and 0),
        "reasoning": "Briefly explain the compliance offset chosen for this material."
    }}
    """

    try:
        response = client.chat.completions.create(
            model=MODEL,
            messages=[
                {"role": "system", "content": system_prompt},
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "Identify object and calculate M based on width."},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                    ],
                }
            ],
            response_format={"type": "json_object"}
        )
        
        data = json.loads(response.choices[0].message.content)
        raw_m = float(data.get('M', -20.0))
        label = data.get('label', 'unknown')
        
        # Safety Clamping [-90, 0]
        safe_m = max(-90.0, min(0.0, raw_m))
        
        print(f"🧠 [AI2]: {label.upper()} | M={safe_m} (Reason: {data.get('reasoning')})")
        return safe_m, label

    except Exception as e:
        print(f"❌ Lỗi Vision2: {e}")
        return -25.0, "error"

if __name__ == "__main__":
    get_grasp_parameters()