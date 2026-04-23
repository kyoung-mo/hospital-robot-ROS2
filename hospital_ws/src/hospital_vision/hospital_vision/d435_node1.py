import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
import time

class D435Node(Node):
    def __init__(self):
        super().__init__('d435_node')
        self.bridge = CvBridge()

        self.fall_pub = self.create_publisher(String, '/hospital/fall_suspected', 10)
        self.facility_pub = self.create_publisher(String, '/hospital/facility_status', 10)
        self.viz_pub = self.create_publisher(Image, '/camera/viz/image_raw', 10)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        self.fall_status_r1 = "NONE"
        self.fall_status_r2 = "NONE"
        self.waste_status = {"101": "OK", "102": "OK"}
        self.waste_pub_count = {"101": 0, "102": 0}
        self.waste_last_pub_time = {"101": 0.0, "102": 0.0}
        self.fall_status = "NONE"

        self.depths = {
            "101_bed": 0.0, "101_waste": 0.0,
            "102_bed": 0.0, "102_waste": 0.0
        }

        self.timer = self.create_timer(1.0/30.0, self.process_frames)
        self.get_logger().info("D435 Node Started — ROI 4구역 모드")

    def get_roi_depth(self, roi):
        flat = roi.flatten()
        flat = flat[~np.isnan(flat)]
        if len(flat) > 0:
            return float(np.nanmedian(flat))
        return np.nan

    def process_frames(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth[depth == 0] = np.nan

            h, w = depth.shape

            rois = {
                "101_bed":   depth[0:h//2,                  w//4:w//2],
                "102_bed":   depth[0:h//2,                  w//2:3*w//4],
                "101_waste": depth[h//2:h//2+(h//2)*2//3,   w//4+40:w//2-40],
                "102_waste": depth[h//2:h//2+(h//2)*2//3,   w//2+40:3*w//4-40],
            }

            for key, roi in rois.items():
                self.depths[key] = self.get_roi_depth(roi)

            self.check_waste()
            self.check_fall()
            self.publish_viz(color, h, w)
        except Exception as e:
            self.get_logger().error(f"Frame Error: {e}")

    def check_waste(self):
        msg = String()
        now = time.time()

        for room in ["101", "102"]:
            depth_key = f"{room}_waste"
            prev = self.waste_status[room]

            if not np.isnan(self.depths[depth_key]) and self.depths[depth_key] < 750:
                if prev != "FULL":
                    self.waste_status[room] = "FULL"
                    self.waste_pub_count[room] = 0
                    self.waste_last_pub_time[room] = 0.0
                    self.get_logger().warn(f"ROOM{room} WASTE FULL ({self.depths[depth_key]:.0f}mm)")

                if self.waste_pub_count[room] < 5 and (now - self.waste_last_pub_time[room]) >= 1.0:
                    msg.data = f"{room}_WASTE_FULL"
                    self.facility_pub.publish(msg)
                    self.waste_pub_count[room] += 1
                    self.waste_last_pub_time[room] = now
                    self.get_logger().info(f"ROOM{room} WASTE FULL 발행 {self.waste_pub_count[room]}/5")
            else:
                if prev != "OK":
                    self.get_logger().info(f"ROOM{room} WASTE OK")
                self.waste_status[room] = "OK"
                self.waste_pub_count[room] = 0
                self.waste_last_pub_time[room] = 0.0

    def check_fall(self):
        FALL_THRESHOLD = 750
        bed1 = self.depths["101_bed"]
        bed2 = self.depths["102_bed"]

        falls = {
            "101": (not np.isnan(bed1) and bed1 > FALL_THRESHOLD),
            "102": (not np.isnan(bed2) and bed2 > FALL_THRESHOLD)
        }

        for room, is_fall in falls.items():
            prev = self.fall_status_r1 if room == "101" else self.fall_status_r2
            
            if is_fall:
                if room == "101": self.fall_status_r1 = "SUSPECTED"
                else: self.fall_status_r2 = "SUSPECTED"

                # [중요] Task Manager 호환 데이터 전송
                msg = String()
                msg.data = room # "101" 또는 "102" 전송
                self.fall_pub.publish(msg)

                if prev != "SUSPECTED":
                    # 로그는 요청하신 101_FALL 형식으로 출력
                    self.get_logger().warn(f"🚨 {room}_FALL 감지됨! (데이터 {room} 전송)")
            else:
                if room == "101": self.fall_status_r1 = "NONE"
                else: self.fall_status_r2 = "NONE"
                
                if prev != "NONE":
                    self.get_logger().info(f"✅ {room} FALL 상황 해제")

        self.fall_status = "SUSPECTED" if (falls["101"] or falls["102"]) else "NONE"

    def publish_viz(self, color, h, w):
        viz = color.copy()
        blink = int(time.time() * 2) % 2 == 0

        # 시각화 로직 (기존과 동일)
        if self.fall_status_r1 == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//4, 0), (w//2, h//2), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
        if self.fall_status_r2 == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//2, 0), (3*w//4, h//2), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
        
        # ... (중략: 기존 사각형 그리기 및 텍스트 로직 유지) ...
        cv2.line(viz, (w//2, 0), (w//2, h), (255, 255, 255), 2)
        cv2.line(viz, (0, h//2), (w, h//2), (255, 255, 255), 2)

        def safe(key):
            v = self.depths[key]
            return f"{v:.0f}mm" if not np.isnan(v) else "---"

        cv2.putText(viz, f"R1 Bed: {safe('101_bed')}", (w//4, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.putText(viz, f"R2 Bed: {safe('102_bed')}", (w//2+10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
        
        msg = self.bridge.cv2_to_imgmsg(viz, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_pub.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = D435Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
