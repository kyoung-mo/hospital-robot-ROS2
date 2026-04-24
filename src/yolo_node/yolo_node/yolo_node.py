import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import cv2
from ultralytics import YOLO
import numpy as np
import time
import threading

class PoseNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        try:
            self.model = YOLO("yolo11n-pose.engine")
            self.get_logger().info("🚀 YOLO11 Engine Loaded on GPU")
        except:
            self.model = YOLO("yolo11n-pose.pt").to('cuda')
            self.get_logger().warn("⚠️ Engine file not found. Using .pt on CUDA")

        self.current_location = "Corridor"
        self.last_publish_time = 0
        self.latest_frame = None
        self.lock = threading.Lock()

        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.get_logger().info(f"🤖 Robot ID: {self.robot_id}")

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 압축 토픽으로 변경
        self.sub = self.create_subscription(
            CompressedImage,
            f'/{self.robot_id}/camera/image_raw/compressed',
            self.image_callback,
            qos_profile
        )
        self.sub_d435 = self.create_subscription(String, '/hospital/fall_suspected', self.d435_callback, 10)
        self.fall_pub = self.create_publisher(String, '/hospital/emergency_call', 10)
        self.viz_pub = self.create_publisher(CompressedImage, f'/hospital/yolo_viz/{self.robot_id}/compressed', 1)

        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

        self.get_logger().info("✅ YOLO Pose Node Optimized Started")

    def d435_callback(self, msg):
        data = msg.data
        if data == "101":
            self.current_location = "101"
        elif data == "102":
            self.current_location = "102"
        else:
            self.current_location = "Corridor"

    def image_callback(self, msg):
        try:
            # 압축 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Callback Error: {e}")

    def inference_loop(self):
        while rclpy.ok():
            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                    self.latest_frame = None

            if frame is None:
                time.sleep(0.005)
                continue

            try:
                results = self.model.predict(frame, verbose=False, device=0)
                annotated_frame = frame.copy()

                for r in results:
                    annotated_frame = r.plot()

                    if r.keypoints is None:
                        continue

                    keypoints = r.keypoints.xy.cpu().numpy()
                    for person in keypoints:
                        if len(person) < 17:
                            continue

                        head_y = person[0][1]
                        hip_y = (person[11][1] + person[12][1]) / 2
                        height_diff = abs(head_y - hip_y)

                        if 0.1 < height_diff < 30.0:
                            current_time = time.time()
                            if current_time - self.last_publish_time > 3.0:
                                msg_out = String()
                                msg_out.data = self.current_location
                                self.fall_pub.publish(msg_out)
                                self.get_logger().error(f"🚨 [REAL FALL] {self.robot_id} at {self.current_location}!")
                                self.last_publish_time = current_time

                cv2.putText(annotated_frame, f"[{self.robot_id}] LOC: {self.current_location}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                cv2.imshow(f"Monitor - {self.robot_id}", annotated_frame)
                cv2.waitKey(1)

                if self.viz_pub.get_subscription_count() > 0:
                    _, buf = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    viz_msg = CompressedImage()
                    viz_msg.header.stamp = self.get_clock().now().to_msg()
                    viz_msg.format = 'jpeg'
                    viz_msg.data = buf.tobytes()
                    self.viz_pub.publish(viz_msg)

            except Exception as e:
                self.get_logger().error(f"Inference Error: {e}")

def main():
    rclpy.init()
    node = PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
