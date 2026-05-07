import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()
        self.model = YOLO("yolo11n-pose.pt")

        self.sub = self.create_subscription(
            Image,
            '/camera/viz/image_raw',   # 터틀봇 카메라
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/hospital/detection',
            10
        )

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(frame, verbose=False)

        status = "normal"

        for r in results:
            if r.keypoints is None:
                continue

            kpts = r.keypoints.xy.cpu().numpy()

            for person in kpts:
                head = person[0]
                hip = person[11]

                if abs(head[1] - hip[1]) < 60:
                    status = "fall"

        out = String()
        out.data = status
        self.pub.publish(out)


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
