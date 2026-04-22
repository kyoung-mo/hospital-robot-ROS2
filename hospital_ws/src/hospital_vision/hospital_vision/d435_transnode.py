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

        # 퍼블리셔 (비교용 2개)
        self.fall_pub_abs = self.create_publisher(String, '/hospital/fall_suspected_abs', 10)
        self.fall_pub_delta = self.create_publisher(String, '/hospital/fall_suspected_delta', 10)
        self.viz_pub = self.create_publisher(Image, '/camera/viz/image_raw', 10)

        # 카메라 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        # 상태 변수
        self.depths = {"room1_bed": 0.0, "room2_bed": 0.0}

        # 절대값 방식
        self.fall_start_time_abs = None

        # 변화량 방식
        self.baseline = {"room1_bed": None, "room2_bed": None}
        self.baseline_buffer1 = []
        self.baseline_buffer2 = []
        self.baseline_ready = False
        self.fall_start_time_delta = None

        self.FALL_CONFIRM_SEC = 3.0

        self.timer = self.create_timer(1.0/30.0, self.process_frames)
        self.get_logger().info("D435 Node Started (ABS vs DELTA 비교 모드)")

    def get_roi_depth(self, roi):
        flat = roi.flatten()
        flat = flat[~np.isnan(flat)]
        if len(flat) > 0:
            threshold = np.percentile(flat, 10)
            return float(np.mean(flat[flat <= threshold]))
        return np.nan

    def process_frames(self):
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

        # ROI
        room1 = depth[0:h//2, w//4:w//2]
        room2 = depth[0:h//2, w//2:3*w//4]

        self.depths["room1_bed"] = self.get_roi_depth(room1)
        self.depths["room2_bed"] = self.get_roi_depth(room2)

        # baseline 수집
        if not self.baseline_ready:
            if not np.isnan(self.depths["room1_bed"]):
                self.baseline_buffer1.append(self.depths["room1_bed"])
            if not np.isnan(self.depths["room2_bed"]):
                self.baseline_buffer2.append(self.depths["room2_bed"])

            if len(self.baseline_buffer1) > 30 and len(self.baseline_buffer2) > 30:
                self.baseline["room1_bed"] = np.mean(self.baseline_buffer1)
                self.baseline["room2_bed"] = np.mean(self.baseline_buffer2)
                self.baseline_ready = True
                self.get_logger().info("Baseline 설정 완료")

        # 두 방식 실행
        self.check_fall_absolute()
        self.check_fall_delta()

        self.publish_viz(color, h, w)

    # --------------------------
    # 1️⃣ 절대값 방식
    # --------------------------
    def check_fall_absolute(self):
        FALL_THRESHOLD = 800

        fall = False
        for key in ["room1_bed", "room2_bed"]:
            if not np.isnan(self.depths[key]) and self.depths[key] < FALL_THRESHOLD:
                fall = True

        now = time.time()
        if fall:
            if self.fall_start_time_abs is None:
                self.fall_start_time_abs = now
            elif now - self.fall_start_time_abs > self.FALL_CONFIRM_SEC:
                msg = String()
                msg.data = "FALL_ABS"
                self.fall_pub_abs.publish(msg)
        else:
            self.fall_start_time_abs = None

    # --------------------------
    # 2️⃣ 변화량 방식 ⭐
    # --------------------------
    def check_fall_delta(self):
        if not self.baseline_ready:
            return

        fall = False

        for key in ["room1_bed", "room2_bed"]:
            if not np.isnan(self.depths[key]):
                delta = self.baseline[key] - self.depths[key]

                if delta > 80:  # 약 8cm
                    fall = True

        now = time.time()
        if fall:
            if self.fall_start_time_delta is None:
                self.fall_start_time_delta = now
            elif now - self.fall_start_time_delta > self.FALL_CONFIRM_SEC:
                msg = String()
                msg.data = "FALL_DELTA"
                self.fall_pub_delta.publish(msg)
        else:
            self.fall_start_time_delta = None

    # --------------------------
    # 시각화
    # --------------------------
    def publish_viz(self, color, h, w):
        viz = color.copy()

        cv2.rectangle(viz, (w//4, 0), (w//2, h//2), (0, 0, 255), 2)
        cv2.rectangle(viz, (w//2, 0), (3*w//4, h//2), (255, 0, 0), 2)

        def safe(v):
            return f"{v:.0f}" if not np.isnan(v) else "---"

        cv2.putText(viz, f"R1: {safe(self.depths['room1_bed'])}",
                    (w//4, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.putText(viz, f"R2: {safe(self.depths['room2_bed'])}",
                    (w//2+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

        if self.baseline_ready:
            cv2.putText(viz, "DELTA MODE ACTIVE",
                        (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        else:
            cv2.putText(viz, "CALIBRATING...",
                        (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        msg = self.bridge.cv2_to_imgmsg(viz, encoding="bgr8")
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
