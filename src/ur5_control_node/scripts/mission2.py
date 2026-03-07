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
from math import pi
from vision_brain2 import get_grasp_parameters # Sử dụng vision_brain2
from std_msgs.msg import Bool

# --- [CẤU HÌNH CỔNG] ---
SERIAL_PORT = '/dev/ttyUSB1'   # Step/Valve
LOADCELL_PORT = '/dev/ttyUSB0' # Cân
BAUD_RATE = 115200
DOWN_DISTANCE = 0.069 
MAX_MOVE_TIME = 120.0 

# Biến toàn cục
vision_data = {"category": None, "M": -20.0, "ready": False}
force_data = {"current_max_f": 0.0, "is_measuring": False}

# --- [HÀM DI CHUYỂN AN TOÀN] ---
def move_safe(move_group, target, is_plan=False):
    def start_motion():
        if is_plan: move_group.execute(target, wait=False)
        else: move_group.go(target, wait=False)

    start_motion()
    rospy.sleep(0.2)
    start_time = time.time()
    while not rospy.is_shutdown():
        try:
            if emergency_stop:
                move_group.stop()
                while emergency_stop and not rospy.is_shutdown(): rospy.sleep(0.1)
                start_motion()
                rospy.sleep(0.2)
        except: pass
        curr_joints = move_group.get_current_joint_values()
        if is_plan: target_joints = target.joint_trajectory.points[-1].positions
        else:
            if isinstance(target, list): target_joints = target
            else: 
                cp = move_group.get_current_pose().pose.position
                if ((cp.x-target.position.x)**2 + (cp.y-target.position.y)**2 + (cp.z-target.position.z)**2)**0.5 < 0.01: break
                rospy.sleep(0.05); continue
        if all(abs(a - b) < 0.008 for a, b in zip(curr_joints, target_joints)): break
        if time.time() - start_time > MAX_MOVE_TIME: break
        rospy.sleep(0.02)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(0.4)

# --- [LUỒNG VISION 2] ---
def vision_worker():
    global vision_data
    rospy.loginfo("--> [Vision2] Đang tính toán góc kẹp M...")
    try:
        # vision_brain2 chỉ trả về m và category (áp suất sẽ loop trong mission)
        m, category = get_grasp_parameters()
        vision_data["category"] = category
        vision_data["M"] = m 
        print(f"\n✨ [AI RESULT]: {str(category).upper()} | M_calc = {m} mm\n")
    except Exception as e:
        rospy.logerr("Lỗi Vision: %s", str(e))
    vision_data["ready"] = True

# --- [LUỒNG LOADCELL] ---
def force_worker(ser_loadcell):
    global force_data
    while not rospy.is_shutdown():
        if ser_loadcell and ser_loadcell.in_waiting:
            try:
                line = ser_loadcell.readline().decode('utf-8', errors='ignore').strip()
                if "Tong F:" in line:
                    val_str = line.split("Tong F:")[1].replace("N", "").strip()
                    current_f = float(val_str)
                    if force_data["is_measuring"] and current_f > force_data["current_max_f"]:
                        force_data["current_max_f"] = current_f
            except: pass
        time.sleep(0.01)

# --- [HÀM LƯU CSV 2] ---
def save_log2_csv(obj_name, max_force, m_val, p_val):
    filename = "log2.csv"
    header = ["Timestamp", "Object", "Max F (N)", "M (mm)", "P (kPa)"]
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    new_row = [now, obj_name, f"{max_force:.3f}", f"{m_val:.2f}", f"{p_val}"]
    
    rows = []
    if os.path.exists(filename):
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            rows = list(reader)
    
    data_rows = rows[1:] if rows and rows[0] == header else rows
    data_rows.insert(0, new_row)

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(data_rows)
    print(f"✅ Log2: P={p_val}, F={max_force:.3f}")

# --- [ARDUINO] ---
def init_arduino(port, baud, name):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        rospy.loginfo(f"✅ {name} connected")
        return ser
    except: return None

def send_command(ser, mm_move, pressure):
    if not ser: return
    cmd = f"{mm_move:.2f},{pressure:.2f}\n"
    ser.write(bytes(cmd, 'utf-8'))
    time.sleep(0.5)

# --- [MAIN MISSION 2] ---
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_experiment_2', anonymous=True)
    
    arduino_control = init_arduino(SERIAL_PORT, BAUD_RATE, "Control")
    arduino_loadcell = init_arduino(LOADCELL_PORT, BAUD_RATE, "Loadcell")

    if arduino_loadcell:
        t_force = threading.Thread(target=force_worker, args=(arduino_loadcell,))
        t_force.daemon = True
        t_force.start()

    global emergency_stop, vision_data, force_data
    emergency_stop = False
    def _emergency_cb(msg): 
        global emergency_stop
        emergency_stop = bool(msg.data)
    rospy.Subscriber('/emergency_stop', Bool, _emergency_cb)

    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(0.4) # Tăng tốc tí cho nhanh
    def d2r(deg): return deg * (pi / 180.0)

    # Tọa độ
    joint_home    = [d2r(0), d2r(-90), d2r(0), d2r(-90), d2r(0), d2r(0)]
    joint_A_prime = [d2r(85), d2r(-77), d2r(75), d2r(-87), d2r(-90), d2r(0)]
    joint_B_prime = [d2r(-5), d2r(-77), d2r(75), d2r(-87), d2r(-90), d2r(-90)]

    while not rospy.is_shutdown():
        print("\n" + "🚀"*15 + " THÍ NGHIỆM 2 " + "🚀"*15)
        u_cmd = input("ENTER: Bắt đầu vật mới | q: Thoát: ")
        if u_cmd == 'q': break

        # 1. Quay về HOME & Reset
        move_safe(move_group, joint_home)
        send_command(arduino_control, 0, 0)

        # 2. Đến A' để AI quan sát 1 lần
        rospy.loginfo("Đi tới điểm quan sát A'...")
        move_safe(move_group, joint_A_prime)
        pose_A_ref = move_group.get_current_pose().pose
        
        vision_data["ready"] = False
        vision_worker() # Chạy trực tiếp để lấy M ngay

        if not vision_data["ready"]: continue
        
        # Lấy M cố định cho vật này
        fixed_M = vision_data["M"]
        obj_label = vision_data["category"]
        
        # --- BẮT ĐẦU VÒNG LẶP ÁP SUẤT ---
        pressure_list = [10, 30, 50, 70, 90, 110, 130, 150]
        
        for p in pressure_list:
            print(f"\n--- ⚡ ĐANG CHẠY MỨC: {p} kPa ---")
            
            # A. Hạ xuống A
            wpose_A = copy.deepcopy(pose_A_ref)
            wpose_A.position.z -= DOWN_DISTANCE
            (plan_down, _) = move_group.compute_cartesian_path([wpose_A], 0.01, True)
            move_safe(move_group, plan_down, is_plan=True)

            # B. Bắt đầu đo & Gắp
            force_data["current_max_f"] = 0.0
            force_data["is_measuring"] = True
            
            rospy.loginfo(f"Gắp với M={fixed_M}, P={p}")
            send_command(arduino_control, fixed_M, p)
            time.sleep(3.5)

            # C. Nhấc lên & Sang B'
            move_safe(move_group, joint_A_prime)
            move_safe(move_group, joint_B_prime)
            pose_B_ref = move_group.get_current_pose().pose

            # D. Hạ xuống B & Thả
            wpose_B = copy.deepcopy(pose_B_ref)
            wpose_B.position.z -= DOWN_DISTANCE
            (plan_release, _) = move_group.compute_cartesian_path([wpose_B], 0.01, True)
            move_safe(move_group, plan_release, is_plan=True)
            
            send_command(arduino_control, -fixed_M, 0) # Nhả vật
            time.sleep(1.5)
            force_data["is_measuring"] = False
            
            # E. Lưu Log lượt này
            save_log2_csv(obj_label, force_data["current_max_f"], fixed_M, p)

            # F. Quay lại A' chuẩn bị lượt kế
            move_safe(move_group, joint_B_prime)
            move_safe(move_group, joint_A_prime)

            if p != 150:
                print(f"--- Đã xong {p}kPa ---")
                next_cmd = input(f"👉 ENTER để lên {p+20}kPa | q để bỏ qua vật này: ")
                if next_cmd == 'q': break
        
        # Kết thúc vật
        move_safe(move_group, joint_home)
        rospy.loginfo(f"✅ Hoàn thành 8 mức áp suất cho {obj_label}")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try: main()
    except: pass