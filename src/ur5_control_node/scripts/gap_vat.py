#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import serial
import time
from math import pi

# --- [CẤU HÌNH THÔNG SỐ] ---
SERIAL_PORT = '/dev/ttyUSB0'  # Cổng Arduino điều khiển van & step
BAUD_RATE = 115200

DOWN_DISTANCE = 0.08  # Khoảng cách hạ xuống (m)
BASE_X = 59.0         # <-- NHẬT TỰ ĐIỀN: Khoảng cách kẹp sâu nhất khi P=0 (mm)

# --- [CẤU HÌNH THÍ NGHIỆM 6 LẦN] ---
# Nhật tự điền Áp suất và Offset tương ứng cho 6 lần ở 2 mảng dưới đây:
PRESSURES = [0,     30,    60,    80,    105,  130]  # Áp suất (kPa)
OFFSETS   = [0.0,   0.0,   2.0,   2.0,   4.0,  4.0]  # <-- NHẬT TỰ ĐIỀN OFFSET (mm)

def init_arduino():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        rospy.loginfo(f"✅ Đã kết nối Arduino tại {SERIAL_PORT}")
        return ser
    except Exception as e:
        rospy.logerr(f"❌ LỖI ARDUINO: {e}")
        return None

def send_command(ser, move_val, pressure_val):
    cmd = f"{move_val:.3f},{pressure_val}\n"
    if ser:
        ser.write(bytes(cmd, 'utf-8'))
        rospy.loginfo(f"📤 Gửi lệnh: {cmd.strip()}")
    else:
        rospy.loginfo(f"💻 [SIMULATOR] Gửi lệnh: {cmd.strip()}")
    time.sleep(1.0)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place_experiment', anonymous=True)
    
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(0.3)
    move_group.set_max_acceleration_scaling_factor(0.3)

    ser = init_arduino()

    # --- TỌA ĐỘ (ĐÃ CHUYỂN SANG RADIAN) ---
    joint_home = [0.0, -1.5708, 0.0, -1.5708, 0.0, 1.5708]
    joint_A    = [1.5, -1.3500, 1.3000, -1.5200, -1.5708, 0.0]
    
    # Tọa độ B đã được quy đổi từ độ sang radian
    joint_B    = [-0.0873, -1.35, 1.3000, -1.52, -1.5708, -1.5708] 

    print("\n" + "="*50)
    print("THÍ NGHIỆM GẮP ĐẶT (PICK & PLACE): 6 CHU KỲ")
    print("="*50)

    # Khởi động: Đi từ Home -> A
    rospy.loginfo("Đang di chuyển tới HOME...")
    move_group.go(joint_home, wait=True)
    rospy.loginfo("Đang di chuyển tới A...")
    move_group.go(joint_A, wait=True)
    print("\n✅ Robot đã sẵn sàng ở vị trí A (Trên cao).")

    num_cycles = len(PRESSURES)
    
    for i in range(num_cycles):
        print("\n" + "-"*40)
        print(f"🔄 CHU KỲ {i+1}/{num_cycles}")
        
        pressure = PRESSURES[i]
        offset = OFFSETS[i]
        
        # Tính toán khoảng cách kẹp (-) và nhả (+)
        close_val = -(BASE_X) + offset
        open_val  = BASE_X - offset

        print(f"   + Áp suất: {pressure} kPa")
        print(f"   + Offset bù trừ: {offset} mm")
        print(f"   + Lệnh kẹp (tại A): {close_val:.3f} | Lệnh nhả (tại B): {open_val:.3f}")
        print("-" * 40)

        # ---------------------------------------------------------
        # BƯỚC 1: HẠ XUỐNG VÀ KẸP TẠI A
        # ---------------------------------------------------------
        cmd = input(f"👉 Nhấn [ENTER] để HẠ XUỐNG & KẸP tại A | [q] để THOÁT: ")
        if cmd.strip().lower() == 'q': break

        rospy.loginfo("Đang hạ thẳng đứng tại A...")
        pose_A_high = move_group.get_current_pose().pose
        target_pose_down_A = copy.deepcopy(pose_A_high)
        target_pose_down_A.position.z -= DOWN_DISTANCE
        
        (plan_down_A, _) = move_group.compute_cartesian_path([target_pose_down_A], 0.01, True)
        move_group.execute(plan_down_A, wait=True)

        print(f">>> Đang KẸP: {-BASE_X} + {offset} = {close_val:.3f} mm, Áp suất: {pressure} kPa")
        send_command(ser, close_val, pressure)
        time.sleep(2.0) # Đợi hệ thống khí nén & gripper ổn định

        # ---------------------------------------------------------
        # BƯỚC 2: NHẤC LÊN, SANG B VÀ THẢ
        # ---------------------------------------------------------
        cmd = input("👉 Nhấn [ENTER] để NHẤC LÊN, MANG SANG B & THẢ | [q] để THOÁT: ")
        if cmd.strip().lower() == 'q': break

        # Nhấc thẳng lên lại vị trí A cao
        rospy.loginfo("Đang nhấc thẳng lên tại A...")
        (plan_up_A, _) = move_group.compute_cartesian_path([pose_A_high], 0.01, True)
        move_group.execute(plan_up_A, wait=True)

        # Chạy sang B (trên cao)
        rospy.loginfo("Đang di chuyển sang vị trí B...")
        move_group.go(joint_B, wait=True)

        # Hạ xuống tại B
        rospy.loginfo("Đang hạ xuống tại B...")
        pose_B_high = move_group.get_current_pose().pose
        target_pose_down_B = copy.deepcopy(pose_B_high)
        target_pose_down_B.position.z -= DOWN_DISTANCE
        
        (plan_down_B, _) = move_group.compute_cartesian_path([target_pose_down_B], 0.01, True)
        move_group.execute(plan_down_B, wait=True)

        # Thả vật (Nhả tay kẹp và xả áp suất về 0)
        print(f">>> Đang THẢ: {BASE_X} - {offset} = {open_val:.3f} mm, Áp suất: 0 kPa")
        send_command(ser, open_val, 0)
        time.sleep(1.5)

        # Nhấc thẳng lên lại vị trí B cao
        rospy.loginfo("Đang nhấc thẳng lên tại B...")
        (plan_up_B, _) = move_group.compute_cartesian_path([pose_B_high], 0.01, True)
        move_group.execute(plan_up_B, wait=True)

        # ---------------------------------------------------------
        # BƯỚC 3: QUAY VỀ A ĐỂ CHUẨN BỊ LẦN TIẾP THEO
        # ---------------------------------------------------------
        if i < num_cycles - 1:
            rospy.loginfo("Đang quay về A để chuẩn bị chu kỳ mới...")
            move_group.go(joint_A, wait=True)
            print(f"✅ Hoàn thành chu kỳ {i+1}.")
        else:
            print(f"✅ Hoàn thành chu kỳ cuối cùng ({i+1}).")

    # --- KẾT THÚC ---
    print("\n" + "="*50)
    rospy.loginfo("Đang quay về HOME...")
    move_group.go(joint_home, wait=True)
    print("✅ Đã về Home. Kết thúc thí nghiệm.")

    if ser: ser.close()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass