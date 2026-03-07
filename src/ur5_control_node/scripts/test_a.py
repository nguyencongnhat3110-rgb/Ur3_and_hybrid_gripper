#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
from math import pi

def main():
    # --- 1. KHỞI TẠO ---
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_position_A_logic', anonymous=True)
    
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    # Chỉnh tốc độ chậm cho an toàn
    move_group.set_max_velocity_scaling_factor(0.3)
    move_group.set_max_acceleration_scaling_factor(0.3)

    # Cấu hình khoảng cách hạ xuống (m)
    DOWN_DISTANCE = 0.082 

    # --- 2. TỌA ĐỘ ---
    # Tọa độ Joint (Radian)
    joint_home = [0.0, -1.57, 0.0, -1.57, 0.0, 1.57]
    joint_A    = [1.5, -1.35, 1.30, -1.52, -1.57, 0.0]

    # --- 3. THỰC HIỆN ---
    print("\n" + "="*50)
    print("CHẾ ĐỘ KIỂM TRA: HOME -> A -> DOWN -> A -> HOME")
    print("="*50)

    # B1: Đi đến A (Trên cao)
    rospy.loginfo("Đang di chuyển tới điểm A...")
    move_group.go(joint_A, wait=True)
    move_group.stop()
    print("\n✅ Robot đã đến A (Vị trí quan sát).")

    # B2: Lựa chọn HẠ XUỐNG
    print("\n[BƯỚC 1] LỰA CHỌN:")
    print("👉 Nhấn [ENTER] để HẠ XUỐNG")
    print("👉 Nhấn [q] để QUAY VỀ HOME ngay lập tức")
    
    u_cmd1 = input("\nNhập lệnh: ").strip().lower()

    if u_cmd1 == 'q':
        rospy.loginfo("Đang quay về Home...")
        move_group.go(joint_home, wait=True)
        move_group.stop()
        print("✅ Đã về Home.")
        sys.exit(0)

    # B3: Hạ xuống theo phương thẳng đứng (Cartesian Path)
    rospy.loginfo(f"Đang hạ xuống {DOWN_DISTANCE*1000} mm...")
    
    current_pose = move_group.get_current_pose().pose
    target_pose = copy.deepcopy(current_pose)
    target_pose.position.z -= DOWN_DISTANCE
    
    (plan, fraction) = move_group.compute_cartesian_path(
        [target_pose],   # waypoints
        0.01,            # eef_step
        True             # avoid_collisions
    )
    
    if fraction > 0.9: 
        move_group.execute(plan, wait=True)
        print(f"✅ Đã hạ xuống vị trí gắp.")
    else:
        rospy.logerr(f"Không thể tính toán đường hạ xuống an toàn! (Fraction: {fraction})")
        sys.exit(1)

    # B4: Lựa chọn NHẤC LÊN A
    print("\n[BƯỚC 2] LỰA CHỌN:")
    print("👉 Nhấn [ENTER] để NHẤC LÊN A")
    print("👉 Nhấn [q] để QUAY VỀ HOME ngay lập tức")
    
    u_cmd2 = input("\nNhập lệnh: ").strip().lower()

    if u_cmd2 == 'q':
        rospy.loginfo("Đang quay về Home...")
        move_group.go(joint_home, wait=True)
        move_group.stop()
        print("✅ Đã về Home.")
        sys.exit(0)
        
    # B5: Nhấc trở lại điểm A
    rospy.loginfo("Đang nhấc lên vị trí A...")
    move_group.go(joint_A, wait=True)
    move_group.stop()
    print("✅ Đã trở lại A.")

    # B6: Lựa chọn VỀ HOME
    print("\n[BƯỚC 3] LỰA CHỌN:")
    print("👉 Nhấn [ENTER] để kết thúc và QUAY VỀ HOME")
    print("👉 Nhấn [q] để QUAY VỀ HOME")
    
    u_cmd3 = input("\nNhập lệnh: ").strip().lower()
    
    # Ở bước cuối cùng, dù ấn Enter hay 'q' thì cũng về Home
    rospy.loginfo("Đang quay về Home...")
    move_group.go(joint_home, wait=True)
    move_group.stop()
    print("✅ Hoàn thành chu trình. Đã về Home.")

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass