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
        self.waste_status = {"room1": "OK", "room2": "OK"}
        self.waste_pub_count = {"room1": 0, "room2": 0}
        self.waste_last_pub_time = {"room1": 0.0, "room2": 0.0}
        self.fall_status = "NONE"

        self.depths = {
            "room1_bed": 0.0, "room1_waste": 0.0,
            "room2_bed": 0.0, "room2_waste": 0.0
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
            "room1_bed":   depth[0:h//2,                   w//4:w//2],
            "room2_bed":   depth[0:h//2,                   w//2:3*w//4],
            "room1_waste": depth[h//2:h//2+(h//2)*2//3,    w//4+40:w//2-40],
            "room2_waste": depth[h//2:h//2+(h//2)*2//3,    w//2+40:3*w//4-40],
        }

        for key, roi in rois.items():
            self.depths[key] = self.get_roi_depth(roi)

        self.check_waste()
        self.check_fall()
        self.publish_viz(color, h, w)

    def check_waste(self):
        msg = String()
        now = time.time()

        for room in ["room1", "room2"]:
            depth_key = f"{room}_waste"
            prev = self.waste_status[room]

            if not np.isnan(self.depths[depth_key]) and self.depths[depth_key] < 750:
                if prev != "FULL":
                    self.waste_status[room] = "FULL"
                    self.waste_pub_count[room] = 0
                    self.waste_last_pub_time[room] = 0.0
                    self.get_logger().warn(f"{room.upper()} WASTE FULL ({self.depths[depth_key]:.0f}mm)")

                if self.waste_pub_count[room] < 5 and (now - self.waste_last_pub_time[room]) >= 1.0:
                    msg.data = f"{room.upper()}_WASTE_FULL"
                    self.facility_pub.publish(msg)
                    self.waste_pub_count[room] += 1
                    self.waste_last_pub_time[room] = now
                    self.get_logger().info(f"{room.upper()} WASTE FULL 발행 {self.waste_pub_count[room]}/5")
            else:
                if prev != "OK":
                    self.get_logger().info(f"{room.upper()} WASTE OK")
                self.waste_status[room] = "OK"
                self.waste_pub_count[room] = 0
                self.waste_last_pub_time[room] = 0.0

    def check_fall(self):
        FALL_THRESHOLD = 750

        bed1 = self.depths["room1_bed"]
        bed2 = self.depths["room2_bed"]

        fall_r1 = (not np.isnan(bed1) and bed1 > FALL_THRESHOLD)
        fall_r2 = (not np.isnan(bed2) and bed2 > FALL_THRESHOLD)

        # ROOM1
        prev = self.fall_status_r1
        if fall_r1:
            self.fall_status_r1 = "SUSPECTED"
            msg = String()
            msg.data = "ROOM1_FALL"
            self.fall_pub.publish(msg)
            if prev != "SUSPECTED":
                self.get_logger().warn("ROOM1 FALL 시작")
        else:
            self.fall_status_r1 = "NONE"
            if prev != "NONE":
                self.get_logger().info("ROOM1 FALL 해제")

        # ROOM2
        prev = self.fall_status_r2
        if fall_r2:
            self.fall_status_r2 = "SUSPECTED"
            msg = String()
            msg.data = "ROOM2_FALL"
            self.fall_pub.publish(msg)
            if prev != "SUSPECTED":
                self.get_logger().warn("ROOM2 FALL 시작")
        else:
            self.fall_status_r2 = "NONE"
            if prev != "NONE":
                self.get_logger().info("ROOM2 FALL 해제")

        self.fall_status = "SUSPECTED" if (fall_r1 or fall_r2) else "NONE"

    def publish_viz(self, color, h, w):
        viz = color.copy()
        blink = int(time.time() * 2) % 2 == 0

        if self.fall_status_r1 == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//4, 0), (w//2, h//2), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)

        if self.fall_status_r2 == "SUSPECTED" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//2, 0), (3*w//4, h//2), (0, 0, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)

        if self.waste_status["room1"] == "FULL" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//4, h//2), (w//2, h), (0, 165, 255), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)

        if self.waste_status["room2"] == "FULL" and blink:
            overlay = viz.copy()
            cv2.rectangle(overlay, (w//2, h//2), (3*w//4, h), (0, 255, 0), -1)
            viz = cv2.addWeighted(overlay, 0.4, viz, 0.6, 0)

        cv2.line(viz, (w//2, 0), (w//2, h), (255, 255, 255), 2)
        cv2.line(viz, (0, h//2), (w, h//2), (255, 255, 255), 2)

        cv2.rectangle(viz, (w//4, 0), (w//2, h//2), (0, 0, 255), 2)
        cv2.rectangle(viz, (w//2, 0), (3*w//4, h//2), (255, 0, 0), 2)
        cv2.rectangle(viz, (w//4, h//2), (w//2, h), (0, 165, 255), 2)
        cv2.rectangle(viz, (w//2, h//2), (3*w//4, h), (0, 255, 0), 2)

        cv2.rectangle(viz, (w//4+40, h//2), (w//2-40, h//2+(h//2)*2//3), (0, 255, 255), 2)
        cv2.rectangle(viz, (w//2+40, h//2), (3*w//4-40, h//2+(h//2)*2//3), (0, 255, 255), 2)

        def safe(key):
            v = self.depths[key]
            return f"{v:.0f}mm" if not np.isnan(v) else "---"

        cv2.putText(viz, f"R1 Bed: {safe('room1_bed')}",    (w//4, 25),         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255),   2)
        cv2.putText(viz, f"R2 Bed: {safe('room2_bed')}",    (w//2+10, 25),      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0),   2)
        cv2.putText(viz, f"R1 Waste: {safe('room1_waste')}", (w//4, h//2+25),   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,165,255), 2)
        cv2.putText(viz, f"R2 Waste: {safe('room2_waste')}", (w//2+10, h//2+25),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),   2)

        if self.fall_status_r1 == "SUSPECTED":
            cv2.putText(viz, "ROOM1 FALL!", (10, h//4),        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),   2)
        if self.fall_status_r2 == "SUSPECTED":
            cv2.putText(viz, "ROOM2 FALL!", (3*w//4+5, h//4),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),   2)
        if self.waste_status["room1"] == "FULL":
            cv2.putText(viz, "R1 WASTE FULL!", (10, h//2+h//4),       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)
        if self.waste_status["room2"] == "FULL":
            cv2.putText(viz, "R2 WASTE FULL!", (3*w//4+5, h//2+h//4), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0),   2)

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
