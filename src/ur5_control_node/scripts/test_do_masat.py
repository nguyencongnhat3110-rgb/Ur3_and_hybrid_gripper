#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import serial
import time

# --- [CẤU HÌNH THÔNG SỐ] ---
SERIAL_PORT = '/dev/ttyUSB0'  # Cổng Arduino
BAUD_RATE = 115200

DOWN_DISTANCE = 0.07  # Khoảng cách hạ xuống (m)
BASE_X = 28.0         # Khoảng cách kẹp vào lớn nhất khi P=0 (mm)

# --- [CẤU HÌNH THÍ NGHIỆM 6 LẦN] ---
# Nhật có thể tùy chỉnh chính xác áp suất và offset cho từng lần ở đây:
# Index:        Lần 1, Lần 2, Lần 3, Lần 4, Lần 5, Lần 6
PRESSURES =    [0,     25,    50,    75,    100,   125]  # kPa
OFFSETS   =    [0.0,   1.0,   1.5,   3.0,   4.0,   4.0]  # mm (Nhật tự chỉnh lại nếu cần)

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
    rospy.init_node('friction_experiment_logic', anonymous=True)
    
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(0.3)
    move_group.set_max_acceleration_scaling_factor(0.3)

    ser = init_arduino()

    joint_home = [0.0, -1.57, 0.0, -1.57, 0.0, 1.57]
    joint_A    = [1.5, -1.35, 1.30, -1.52, -1.57, 0.0]

    print("\n" + "="*50)
    print("ĐO MA SÁT: 6 CHU KỲ VỚI OFFSET TÙY CHỈNH")
    print("="*50)

    move_group.go(joint_home, wait=True)
    move_group.go(joint_A, wait=True)
    print("\n✅ Robot đã sẵn sàng ở A.")

    # Chạy vòng lặp đúng bằng số lượng phần tử trong mảng
    num_cycles = len(PRESSURES)
    
    for i in range(num_cycles):
        print("\n" + "-"*40)
        print(f"🔄 CHU KỲ {i+1}/{num_cycles}")
        
        # Lấy giá trị từ Mảng
        pressure = PRESSURES[i]
        offset = OFFSETS[i]
        
        close_val = -(BASE_X) + offset
        open_val  = BASE_X - offset

        print(f"   + Áp suất: {pressure} kPa")
        print(f"   + Offset bù trừ: {offset} mm")
        print(f"   + Lệnh kẹp: {close_val:.3f} | Lệnh nhả: {open_val:.3f}")
        print("-" * 40)

        cmd = input(f"👉 Nhấn [ENTER] để HẠ XUỐNG & KẸP (Chu kỳ {i+1}) | [q] để THOÁT: ")
        if cmd.strip().lower() == 'q': break

        # 1. Hạ xuống
        rospy.loginfo("Đang hạ thẳng đứng...")
        current_pose = move_group.get_current_pose().pose
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.z -= DOWN_DISTANCE
        
        (plan_down, fraction) = move_group.compute_cartesian_path([target_pose], 0.01, True)
        move_group.execute(plan_down, wait=True)

        # 2. Kẹp vào
        print(f">>> Đang KẸP: {-BASE_X} + {offset} = {close_val:.3f} mm")
        send_command(ser, close_val, pressure)
        time.sleep(2.0) 

        cmd = input("👉 Nhấn [ENTER] để KÉO LÊN đo ma sát & NHẢ | [q] để THOÁT: ")
        if cmd.strip().lower() == 'q': break

        # 3. Kéo thẳng lên (Cartesian)
        rospy.loginfo("Đang nhấc thẳng đứng...")
        target_pose_up = copy.deepcopy(target_pose)
        target_pose_up.position.z += DOWN_DISTANCE
        (plan_up, fraction) = move_group.compute_cartesian_path([target_pose_up], 0.01, True)
        move_group.execute(plan_up, wait=True)

        # 4. Nhả ra
        print(f">>> Đang NHẢ: {BASE_X} - {offset} = {open_val:.3f} mm")
        send_command(ser, open_val, pressure)
        time.sleep(1.0)
        
        print(f"✅ Hoàn thành chu kỳ {i+1}.")

    print("\n" + "="*50)
    rospy.loginfo("Đang quay về HOME...")
    move_group.go(joint_home, wait=True)
    print("✅ Đã về Home. Kết thúc thí nghiệm.")

    if ser: ser.close()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass