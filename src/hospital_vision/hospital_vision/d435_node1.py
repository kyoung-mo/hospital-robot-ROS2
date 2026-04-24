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
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        # [상태 관리 변수]
        self.waste_status = {"101": "OK", "102": "OK"}
        self.waste_pub_count = {"101": 0, "102": 0}
        self.waste_last_pub_time = {"101": 0.0, "102": 0.0}

        self.fall_status = {"101": "NONE", "102": "NONE"} # 낙상 상태 관리
        self.fall_pub_count = {"101": 0, "102": 0}       # 낙상 카운트
        self.fall_last_pub_time = {"101": 0.0, "102": 0.0} # 낙상 마지막 발행 시간

        self.depths = {
            "101_bed": 0.0, "101_waste": 0.0,
            "102_bed": 0.0, "102_waste": 0.0
        }

        self.timer = self.create_timer(1.0/30.0, self.process_frames)
        self.get_logger().info("D435 Node Started — 5-Shot Limit Mode Enabled")

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
        now = time.time()
        for room in ["101", "102"]:
            depth_key = f"{room}_waste"
            current_depth = self.depths[depth_key]
            
            # 쓰레기 감지 기준 (750mm 미만)
            if not np.isnan(current_depth) and current_depth < 750:
                if self.waste_status[room] != "FULL":
                    self.waste_status[room] = "FULL"
                    self.waste_pub_count[room] = 0 # 상태 변환 시 카운트 리셋
                    self.get_logger().warn(f"⚠️ ROOM {room} WASTE FULL 감지!")

                # 1초 간격으로 최대 5번 발행
                if self.waste_pub_count[room] < 5 and (now - self.waste_last_pub_time[room]) >= 1.0:
                    msg = String()
                    msg.data = f"{room}_WASTE_FULL"
                    self.facility_pub.publish(msg)
                    self.waste_pub_count[room] += 1
                    self.waste_last_pub_time[room] = now
                    self.get_logger().info(f"📤 {room} WASTE FULL 송신 ({self.waste_pub_count[room]}/5)")
            else:
                # 쓰레기통이 비워지면 상태 복구 및 카운트 준비
                if self.waste_status[room] == "FULL":
                    self.get_logger().info(f"✅ ROOM {room} WASTE OK (Reset)")
                self.waste_status[room] = "OK"
                self.waste_pub_count[room] = 0

    def check_fall(self):
        FALL_THRESHOLD = 750
        now = time.time()

        for room in ["101", "102"]:
            bed_depth = self.depths[f"{room}_bed"]
            
            # 낙상 감지 기준 (베드 깊이가 Threshold보다 클 때)
            if not np.isnan(bed_depth) and bed_depth > FALL_THRESHOLD:
                if self.fall_status[room] != "SUSPECTED":
                    self.fall_status[room] = "SUSPECTED"
                    self.fall_pub_count[room] = 0 # 상태 변환 시 카운트 리셋
                    self.get_logger().error(f"🚨 ROOM {room} FALL 발생!")

                # 1초 간격으로 최대 5번 발행
                if self.fall_pub_count[room] < 5 and (now - self.fall_last_pub_time[room]) >= 1.0:
                    msg = String()
                    msg.data = room # 친구 C++ 데이터용
                    self.fall_pub.publish(msg)
                    self.fall_pub_count[room] += 1
                    self.fall_last_pub_time[room] = now
                    self.get_logger().info(f"📤 {room} FALL 신호 송신 ({self.fall_pub_count[room]}/5)")
            else:
                # 정상으로 돌아오면 상태 복구 및 카운트 준비
                if self.fall_status[room] == "SUSPECTED":
                    self.get_logger().info(f"✅ ROOM {room} FALL 상황 해제 (Reset)")
                self.fall_status[room] = "NONE"
                self.fall_pub_count[room] = 0

    def publish_viz(self, color, h, w):
        viz = color.copy()
        blink = int(time.time() * 2) % 2 == 0

        # 구역 오버레이 (낙상/쓰레기통)
        if self.fall_status["101"] == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//4, 0), (w//2, h//2), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
        if self.fall_status["102"] == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//2, 0), (3*w//4, h//2), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
        if self.waste_status["101"] == "FULL" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//4, h//2), (w//2, h), (0, 165, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)
        if self.waste_status["102"] == "FULL" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//2, h//2), (3*w//4, h), (0, 255, 0), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)

        # 가이드 라인 및 박스 레이아웃 유지
        cv2.line(viz, (w//2, 0), (w//2, h), (255, 255, 255), 2)
        cv2.line(viz, (0, h//2), (w, h//2), (255, 255, 255), 2)
        cv2.rectangle(viz, (w//4, 0), (w//2, h//2), (0, 0, 255), 2)
        cv2.rectangle(viz, (w//2, 0), (3*w//4, h//2), (255, 0, 0), 2)
        cv2.rectangle(viz, (w//4, h//2), (w//2, h), (0, 165, 255), 2)
        cv2.rectangle(viz, (w//2, h//2), (3*w//4, h), (0, 255, 0), 2)

        def safe(key):
            v = self.depths[key]
            return f"{v:.0f}mm" if not np.isnan(v) else "---"

        cv2.putText(viz, f"101 Bed: {safe('101_bed')}", (w//4+5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(viz, f"102 Bed: {safe('102_bed')}", (w//2+5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(viz, f"101 Waste: {safe('101_waste')}", (w//4+5, h//2+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        cv2.putText(viz, f"102 Waste: {safe('102_waste')}", (w//2+5, h//2+30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self.fall_status["101"] == "SUSPECTED":
            cv2.putText(viz, "101 FALL!", (10, h//4), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if self.fall_status["102"] == "SUSPECTED":
            cv2.putText(viz, "102 FALL!", (3*w//4+5, h//4), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        if self.waste_status["101"] == "FULL":
            cv2.putText(viz, "101 WASTE FULL!", (10, h//2+h//4), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        if self.waste_status["102"] == "FULL":
            cv2.putText(viz, "102 WASTE FULL!", (3*w//4+5, h//2+h//4), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

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
