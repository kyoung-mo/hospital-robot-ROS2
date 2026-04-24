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

        # 퍼블리셔 설정
        self.fall_pub = self.create_publisher(String, '/hospital/fall_suspected', 10)
        self.facility_pub = self.create_publisher(String, '/hospital/facility_status', 10)
        self.viz_pub = self.create_publisher(Image, '/camera/viz/image_raw', 10)

        # 리얼센스 설정
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
        except Exception as e:
            self.get_logger().error(f"RealSense Init Error: {e}")

        # 상태 변수
        self.fall_status_r1 = "NONE"
        self.fall_status_r2 = "NONE"
        self.waste_status = {"101": "OK", "102": "OK"}
        self.waste_pub_count = {"101": 0, "102": 0}
        self.waste_last_pub_time = {"101": 0.0, "102": 0.0}
        
        self.depths = {
            "101_bed": 0.0, "101_waste": 0.0,
            "102_bed": 0.0, "102_waste": 0.0
        }

        self.timer = self.create_timer(1.0/15.0, self.process_frames)
        self.get_logger().info("✅ D435 Node Started — ROI Optimized Mode")

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

            # ROI 영역 설정 (화면 4분할 기반) - 인덱싱 정수 변환 포함
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
        now = time.time()
        for room in ["101", "102"]:
            depth_key = f"{room}_waste"
            prev = self.waste_status[room]

            # 750mm 보다 가까우면 꽉 찬 것으로 판단
            if not np.isnan(self.depths[depth_key]) and self.depths[depth_key] < 750:
                if prev != "FULL":
                    self.waste_status[room] = "FULL"
                    self.waste_pub_count[room] = 0
                    self.get_logger().warn(f"⚠️ ROOM {room} WASTE FULL!")

                if self.waste_pub_count[room] < 5 and (now - self.waste_last_pub_time[room]) >= 1.0:
                    msg = String()
                    msg.data = f"{room}_WASTE_FULL"
                    self.facility_pub.publish(msg)
                    self.waste_pub_count[room] += 1
                    self.waste_last_pub_time[room] = now
            else:
                self.waste_status[room] = "OK"
                self.waste_pub_count[room] = 0

    def check_fall(self):
        FALL_THRESHOLD = 750
        
        # 101 낙상 판단
        bed1 = self.depths["101_bed"]
        prev1 = self.fall_status_r1
        if not np.isnan(bed1) and bed1 > FALL_THRESHOLD:
            self.fall_status_r1 = "SUSPECTED"
            msg = String()
            msg.data = "101"
            self.fall_pub.publish(msg)
            if prev1 != "SUSPECTED": 
                self.get_logger().warn("🚨 101 FALL 감지!")
        else:
            self.fall_status_r1 = "NONE"

        # 102 낙상 판단
        bed2 = self.depths["102_bed"]
        prev2 = self.fall_status_r2
        if not np.isnan(bed2) and bed2 > FALL_THRESHOLD:
            self.fall_status_r2 = "SUSPECTED"
            msg = String()
            msg.data = "102"
            self.fall_pub.publish(msg)
            if prev2 != "SUSPECTED": 
                self.get_logger().warn("🚨 102 FALL 감지!")
        else:
            self.fall_status_r2 = "NONE"

    def publish_viz(self, color, h, w):
        viz = color.copy()
        blink = int(time.time() * 2) % 2 == 0

        # 구역 오버레이 (낙상/쓰레기통) - 좌표 정수형 변환 필수
        if self.fall_status_r1 == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (int(w//4), 0), (int(w//2), int(h//2)), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
            
        if self.fall_status_r2 == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (int(w//2), 0), (int(3*w//4), int(h//2)), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
            
        if self.waste_status["101"] == "FULL" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (int(w//4), int(h//2)), (int(w//2), int(h)), (0, 165, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
            
        if self.waste_status["102"] == "FULL" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (int(w//2), int(h//2)), (int(3*w//4), int(h)), (0, 255, 0), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)

        # 가이드 라인 및 박스 테두리
        cv2.line(viz, (int(w//2), 0), (int(w//2), int(h)), (255, 255, 255), 2)
        cv2.line(viz, (0, int(h//2)), (int(w), int(h//2)), (255, 255, 255), 2)
        cv2.rectangle(viz, (int(w//4), 0), (int(w//2), int(h//2)), (0, 0, 255), 2)
        cv2.rectangle(viz, (int(w//2), 0), (int(3*w//4), int(h//2)), (255, 0, 0), 2)
        cv2.rectangle(viz, (int(w//4), int(h//2)), (int(w//2), int(h)), (0, 165, 255), 2)
        cv2.rectangle(viz, (int(w//2), int(h//2)), (int(3*w//4), int(h)), (0, 255, 0), 2)

        def safe(key):
            v = self.depths[key]
            return f"{v:.0f}mm" if not np.isnan(v) else "---"

        # 텍스트 출력
        cv2.putText(viz, f"101 Bed: {safe('101_bed')}", (int(w//4+5), 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(viz, f"102 Bed: {safe('102_bed')}", (int(w//2+5), 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(viz, f"101 Waste: {safe('101_waste')}", (int(w//4+5), int(h//2+30)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        cv2.putText(viz, f"102 Waste: {safe('102_waste')}", (int(w//2+5), int(h//2+30)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 상태 텍스트
        if self.fall_status_r1 == "SUSPECTED":
            cv2.putText(viz, "101 FALL!", (10, int(h//4)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if self.fall_status_r2 == "SUSPECTED":
            cv2.putText(viz, "102 FALL!", (int(3*w//4+5), int(h//4)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if self.waste_status["101"] == "FULL":
            cv2.putText(viz, "101 WASTE FULL!", (10, int(h//2+h//4)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        if self.waste_status["102"] == "FULL":
            cv2.putText(viz, "102 WASTE FULL!", (int(3*w//4+5), int(h//2+h//4)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 이미지 발행
        msg = self.bridge.cv2_to_imgmsg(viz, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.viz_pub.publish(msg)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
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
