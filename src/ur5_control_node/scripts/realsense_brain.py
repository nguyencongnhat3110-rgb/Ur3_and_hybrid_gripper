import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCamera:
    def __init__(self):
        print("📷 [RealSense] Đang khởi động Camera D435...")
        
        # 1. Cấu hình Pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # Cấu hình tối ưu cho D435: 640x480 @ 30fps
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 2. Khởi tạo các bộ lọc (FILTERS) để khử nhiễu
        self.decimation = rs.decimation_filter()
        self.decimation.set_option(rs.option.filter_magnitude, 1)
        
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 2)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial.set_option(rs.option.filter_smooth_delta, 20)
        
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()
        
        # 3. Start Camera
        profile = self.pipeline.start(config)
        
        # Lấy thông số thấu kính
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Tăng công suất Laser
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1.0)
        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, 300.0)
            
        # Align
        self.align = rs.align(rs.stream.color)
        
        # Warm-up
        for _ in range(15): self.pipeline.wait_for_frames()
        print("✅ [RealSense] Sẵn sàng đo đạc!")

    def get_data(self):
        try:
            frames = self.pipeline.wait_for_frames()
            
            # --- ÁP DỤNG BỘ LỌC (QUAN TRỌNG) ---
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return None, 0, 0

            # Lọc nhiễu tuần tự
            # Lưu ý: Các filter trả về 'frame' thường, không có get_distance
            filtered_depth = self.decimation.process(depth_frame)
            filtered_depth = self.spatial.process(filtered_depth)
            filtered_depth = self.temporal.process(filtered_depth)
            filtered_depth = self.hole_filling.process(filtered_depth)
            
            # ⚠️ QUAN TRỌNG: Ép kiểu ngược lại về depth_frame để dùng get_distance
            depth_frame = filtered_depth.as_depth_frame()

            # Lấy Intrinsics sau khi align
            intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()

            # Convert sang Numpy
            color_image = np.asanyarray(color_frame.get_data())

            # --- THUẬT TOÁN ĐO CHIỀU RỘNG TẠI TÂM ---
            h, w, _ = color_image.shape
            cx, cy = w // 2, h // 2 
            
            # 1. Lấy độ sâu tại tâm
            # Bây giờ hàm này sẽ chạy ngon lành vì đã có .as_depth_frame()
            dist_center = depth_frame.get_distance(cx, cy) 
            
            if dist_center < 0.05 or dist_center > 1.0:
                cv2.putText(color_image, "NO OBJECT CENTER", (cx-50, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                return color_image, 0, 0

            # 2. Quét ngang tìm mép vật
            depth_threshold = 0.02 
            
            left_x = cx
            while left_x > 0:
                d = depth_frame.get_distance(left_x, cy)
                if d == 0: 
                    left_x -= 1
                    continue 
                if abs(d - dist_center) > depth_threshold: 
                    break 
                left_x -= 2
            
            right_x = cx
            while right_x < w:
                d = depth_frame.get_distance(right_x, cy)
                if d == 0:
                    right_x += 1
                    continue
                if abs(d - dist_center) > depth_threshold:
                    break 
                right_x += 2
                
            # 3. Tính toán kích thước thật
            point_left = rs.rs2_deproject_pixel_to_point(intrinsics, [left_x, cy], dist_center)
            point_right = rs.rs2_deproject_pixel_to_point(intrinsics, [right_x, cy], dist_center)
            
            width_meter = point_right[0] - point_left[0]
            width_mm = abs(width_meter * 1000)
            
            # --- VẼ DEBUG ---
            cv2.line(color_image, (left_x, cy), (right_x, cy), (0, 255, 0), 2)
            cv2.circle(color_image, (left_x, cy), 4, (0, 0, 255), -1)
            cv2.circle(color_image, (right_x, cy), 4, (0, 0, 255), -1)
            cv2.putText(color_image, f"{width_mm:.1f}mm", (cx-30, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            return color_image, width_mm, dist_center*1000

        except Exception as e:
            # Chỉ in lỗi nếu không phải là do tắt chương trình
            print(f"❌ RealSense Error: {e}")
            return None, 0, 0

    def stop(self):
        self.pipeline.stop()

if __name__ == "__main__":
    cam = RealSenseCamera()
    try:
        while True:
            img, w, d = cam.get_data()
            if img is not None:
                cv2.imshow("D435 Filtered View", img)
            print(f"\r📏 Width: {w:.1f} mm | Dist: {d:.1f} mm   ", end="")
            if cv2.waitKey(1) == ord('q'): break
    finally:
        cam.stop()