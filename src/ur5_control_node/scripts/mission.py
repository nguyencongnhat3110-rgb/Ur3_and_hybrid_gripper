#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import serial
import time
import select
import termios
import tty
import threading
import csv
import os
import datetime
import re
from math import pi
import traceback
from vision_brain import get_grasp_parameters
from std_msgs.msg import Bool

# --- [CẤU HÌNH] ---
# Cổng cho Arduino điều khiển Step/Van
SERIAL_PORT = '/dev/ttyUSB2' 
# Cổng cho Arduino đọc Loadcell (Cân)
LOADCELL_PORT = '/dev/ttyUSB0' 

BAUD_RATE = 115200
DOWN_DISTANCE = 0.069 
MAX_MOVE_TIME = 120.0 

# Biến toàn cục Vision
vision_data = {"category": None, "P": 50, "M": -20.0, "ready": False}

# Biến toàn cục Loadcell
force_data = {
    "current_max_f": 0.0,  # Lưu giá trị F lớn nhất trong quá trình kẹp
    "is_measuring": False, # Cờ bật/tắt chế độ ghi
    "debug_f": 0.0         # Giá trị realtime để in ra màn hình chơi
}

# --- [HÀM ĐỌC PHÍM] ---
def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist: key = sys.stdin.read(1)
        else: key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

# --- [HÀM DI CHUYỂN AN TOÀN] ---
def move_safe(move_group, target, is_plan=False):
    def start_motion():
        if is_plan:
            move_group.execute(target, wait=False)
        else:
            move_group.go(target, wait=False)

    start_motion()
    rospy.sleep(0.2)
    start_time = time.time()

    while not rospy.is_shutdown():
        try:
            if emergency_stop:
                move_group.stop()
                try:
                    move_group.clear_pose_targets()
                except Exception:
                    rospy.logdebug("clear_pose_targets failed: %s", traceback.format_exc())
                rospy.logwarn("[STOP] Emergency stop active.")
                while emergency_stop and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                start_motion()
                rospy.sleep(0.2)
        except NameError:
            pass

        curr_joints = move_group.get_current_joint_values()
        if is_plan:
            target_joints = target.joint_trajectory.points[-1].positions
        else:
            if isinstance(target, list): target_joints = target
            else: 
                cp = move_group.get_current_pose().pose.position
                if ((cp.x-target.position.x)**2 + (cp.y-target.position.y)**2 + (cp.z-target.position.z)**2)**0.5 < 0.01: break
                rospy.sleep(0.05); continue
        
        if all(abs(a - b) < 0.008 for a, b in zip(curr_joints, target_joints)):
            break

        if time.time() - start_time > MAX_MOVE_TIME:
            rospy.logerr("move_safe: timeout (%.1fs)", MAX_MOVE_TIME)
            break

        rospy.sleep(0.02)

    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(0.4)

# --- [LUỒNG 2: VISION REASONING] ---
def vision_worker():
    global vision_data
    rospy.loginfo("--> [Vision] Đang chụp ảnh và phân tích...")
    try:
        p, m, category = get_grasp_parameters()
        vision_data["category"] = category
        vision_data["P"] = p
        vision_data["M"] = m 
        
        print(f"\n✨ [AI RESULT]: {str(category).upper()}")
        print(f"📊 Params: M={m} mm, P={p} kPa\n")
        
    except Exception as e:
        rospy.logerr("Lỗi Vision: %s", str(e))
    
    vision_data["ready"] = True

# --- [LUỒNG 3: FORCE MONITOR (LOADCELL)] ---
def force_worker(ser_loadcell):
    global force_data
    while not rospy.is_shutdown():
        if ser_loadcell and ser_loadcell.in_waiting:
            try:
                # Đọc 1 dòng từ Arduino Uno
                line = ser_loadcell.readline().decode('utf-8', errors='ignore').strip()
                
                # Format mẫu: "Can 1: 10.5 g | Can 2: 12.0 g | Tong F: 3.452 N"
                if "Tong F:" in line:
                    # Tách lấy phần sau chữ "Tong F:"
                    parts = line.split("Tong F:")
                    if len(parts) > 1:
                        # Lấy số, bỏ chữ "N" và khoảng trắng
                        val_str = parts[1].replace("N", "").strip()
                        current_f = float(val_str)
                        
                        force_data["debug_f"] = current_f # Để in ra màn hình nếu thích
                        
                        # Nếu đang trong quá trình kẹp (Flag = True) thì bắt Max
                        if force_data["is_measuring"]:
                            if current_f > force_data["current_max_f"]:
                                force_data["current_max_f"] = current_f
            except Exception as e:
                pass # Lỗi đọc serial lặt vặt thì bỏ qua
        time.sleep(0.01) # Nghỉ tí cho nhẹ CPU

# --- [HÀM LƯU CSV] ---
def save_log_csv(obj_name, max_force, m_val, p_val):
    filename = "log.csv"
    header = ["Timestamp", "Object Detected", "Max Force (N)", "Angle/M (mm)", "Pressure (kPa)"]
    
    # Lấy thời gian hiện tại
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    

    # Dòng dữ liệu mới
    new_row = [now, obj_name, f"{max_force:.3f}", f"{m_val:.2f}", f"{p_val}"]
    
    rows = []
    # 1. Đọc file cũ nếu có
    if os.path.exists(filename):
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                rows = list(reader)
        except: pass

    # 2. Xử lý Header và chèn dòng mới lên đầu (sau header)
    if rows and rows[0] == header:
        data_rows = rows[1:]
    else:
        data_rows = rows # File chưa có hoặc sai header

    # Chèn dòng mới nhất lên đầu danh sách data
    data_rows.insert(0, new_row)

    # 3. Ghi lại tất cả
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header) # Ghi Header
        writer.writerows(data_rows) # Ghi dữ liệu (Mới -> Cũ)
    
    print(f"\n💾 [LOG SAVED] Đã lưu vào {filename}: F_max = {max_force:.3f} N\n")

# --- [ARDUINO INIT] ---
def init_arduino(port, baud, name="Arduino"):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        rospy.loginfo(f"✅ {name} connected on {port}")
        return ser
    except Exception as e:
        rospy.logerr(f"❌ Không kết nối được {name} tại {port}")
        return None

def send_command(ser, mm_move, pressure):
    if not ser: return
    try:
        cmd = f"{mm_move:.2f},{pressure:.2f}\n"
        ser.write(bytes(cmd, 'utf-8'))
        rospy.loginfo(f"📤 Sent Step/Valve: {cmd.strip()}")
    except Exception as e:
        rospy.logerr("Serial error: %s", str(e))
    time.sleep(0.5)

# --- [MAIN MISSION] ---
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_llm_mission', anonymous=True)
    
    # Kết nối 2 Arduino
    arduino_control = init_arduino(SERIAL_PORT, BAUD_RATE, "Step/Valve Controller")
    arduino_loadcell = init_arduino(LOADCELL_PORT, BAUD_RATE, "Loadcell Reader")

    # Khởi chạy luồng đọc Loadcell ngay lập tức
    if arduino_loadcell:
        t_force = threading.Thread(target=force_worker, args=(arduino_loadcell,))
        t_force.daemon = True # Tự tắt khi chương trình chính tắt
        t_force.start()
    else:
        rospy.logwarn("⚠️ Không có Loadcell, tính năng log F sẽ bằng 0.")

    global emergency_stop, vision_data, force_data
    emergency_stop = False

    def _emergency_cb(msg):
        global emergency_stop
        emergency_stop = bool(msg.data)

    rospy.Subscriber('/emergency_stop', Bool, _emergency_cb)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(0.3)
    move_group.set_max_acceleration_scaling_factor(0.3)

    # Hàm đổi Độ sang Radian
    def d2r(deg): return deg * (pi / 180.0)

    # Tọa độ
    joint_home    = [d2r(0), d2r(-90), d2r(0), d2r(-90), d2r(0), d2r(0)]
    joint_A_prime = [d2r(85), d2r(-77), d2r(75), d2r(-87), d2r(-90), d2r(0)]
    joint_B_prime = [d2r(-5), d2r(-77), d2r(75), d2r(-87), d2r(-90), d2r(-90)]

    while not rospy.is_shutdown():
        print("\n" + "="*40)
        u_cmd = input("ENTER: Start Mission | q: Quit: ")
        if u_cmd == 'q': break

        # Reset Vision & Force Data cho vòng lặp mới
        vision_data["ready"] = False
        force_data["current_max_f"] = 0.0 # Reset Max F về 0
        force_data["is_measuring"] = False

        # Chạy Vision
        t_vision = threading.Thread(target=vision_worker)
        t_vision.start()

        # 1. HOME
        move_safe(move_group, joint_home)
        send_command(arduino_control, 0, 0) # Mở Gripper, xả áp

        # 2. ĐẾN A'
        rospy.loginfo("[Flow] Đến A'...")
        move_safe(move_group, joint_A_prime)
        pose_A_ref = move_group.get_current_pose().pose

        # 3. ĐỢI AI
        if not vision_data["ready"]:
            rospy.loginfo("⏳ Đang đợi AI...")
            while not vision_data["ready"] and not rospy.is_shutdown():
                rospy.sleep(0.1)

        dyn_P = vision_data["P"]
        dyn_M = vision_data["M"]

        # 4. HẠ XUỐNG GẮP
        rospy.loginfo(f"Hạ xuống gắp {vision_data['category']}...")
        wpose = copy.deepcopy(pose_A_ref)
        wpose.position.z -= DOWN_DISTANCE 
        (plan, fraction) = move_group.compute_cartesian_path([wpose], 0.01, True)
        move_safe(move_group, plan, is_plan=True)

        # --- BẮT ĐẦU ĐO LỰC (START LOGGING F) ---
        print(">>> BẮT ĐẦU ĐO LỰC (Max Force Tracking)...")
        force_data["current_max_f"] = 0.0
        force_data["is_measuring"] = True

        # 5. GẮP
        rospy.loginfo(f"GẮP: Move {dyn_M}mm, Press {dyn_P}kPa")
        send_command(arduino_control, dyn_M, dyn_P)
        time.sleep(4.0) # Đợi kẹp xong

        # 6. NHẤC LÊN & SANG B'
        rospy.loginfo("Nhấc lên và sang B'...")
        move_safe(move_group, pose_A_ref)
        move_safe(move_group, joint_B_prime)

        # 7. HẠ XUỐNG B
        rospy.loginfo("Hạ xuống B...")
        pose_B_ref = move_group.get_current_pose().pose
        wpose_B = copy.deepcopy(pose_B_ref)
        wpose_B.position.z -= DOWN_DISTANCE
        (plan_B, fraction) = move_group.compute_cartesian_path([wpose_B], 0.01, True)
        move_safe(move_group, plan_B, is_plan=True)

        # 8. THẢ
        release_M = -dyn_M 
        rospy.loginfo(f"THẢ: Move {release_M}mm, Press 0kPa")
        send_command(arduino_control, release_M, 0)
        time.sleep(1.5)

        # --- KẾT THÚC ĐO LỰC & LƯU LOG ---
        force_data["is_measuring"] = False
        print(f">>> DỪNG ĐO LỰC. Max Force thu được: {force_data['current_max_f']:.3f} N")
        
        # Lưu vào CSV
        save_log_csv(
            vision_data['category'], 
            force_data['current_max_f'], 
            dyn_M, 
            dyn_P
        )

        # 9. VỀ HOME
        move_safe(move_group, pose_B_ref)
        move_safe(move_group, joint_home)

        rospy.loginfo("✅ Hoàn thành chu trình.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass