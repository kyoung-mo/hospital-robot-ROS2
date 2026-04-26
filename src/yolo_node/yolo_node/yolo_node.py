#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
from ultralytics import YOLO
import numpy as np
import time
import threading

class PoseNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # 1. 로봇 ID 파라미터
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.get_logger().info(f"🤖 Starting YOLO Node for: {self.robot_id}")

        # 2. YOLO 모델 로드
        try:
            self.model = YOLO("yolo11n-pose.engine")
            self.get_logger().info("🚀 YOLO11 Engine Loaded on GPU")
        except:
            self.model = YOLO("yolo11n-pose.pt").to('cuda')
            self.get_logger().warn("⚠️ Engine file not found. Using .pt on CUDA")

        # 3. 상태 변수
        self.current_location = "Corridor"
        self.latest_frame = None
        self.last_publish_time = 0
        self.emergency_sent = False
        self.fall_count = 0
        self.missing_count = 0
        self.lock = threading.Lock()

        # 4. QoS 설정
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 5. 구독자 설정
        self.sub_img = self.create_subscription(
            CompressedImage,
            f'/{self.robot_id}/camera/image_raw/compressed',
            self.image_callback,
            qos_profile
        )

        task_topic = "/task_assignment" if self.robot_id == "robot_1" else "/robot2/task_assignment"
        self.sub_task = self.create_subscription(
            String,
            task_topic,
            self.task_callback,
            10
        )

        # 6. 퍼블리셔 설정
        self.fall_pub = self.create_publisher(String, '/hospital/emergency_call', 10)
        self.viz_pub  = self.create_publisher(
            CompressedImage,
            f'/hospital/yolo_viz/{self.robot_id}/compressed',
            1
        )

        # 7. 추론 스레드 실행
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

    def task_callback(self, msg):
        self.current_location = msg.data if (msg.data and msg.data != "None") else "Corridor"
        self.get_logger().info(f"📍 [{self.robot_id}] Location: {self.current_location}")

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                with self.lock:
                    self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Image Decode Error: {e}")

    def inference_loop(self):
        while rclpy.ok():
            # [렉 개선] 추론 주기 제한 - 최대 20fps
            time.sleep(0.05)

            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                    self.latest_frame = None

            if frame is None:
                continue

            try:
                results = self.model.predict(frame, verbose=False, device=0, conf=0.35)
                fall_detected_in_frame = False

                for r in results:
                    if r.keypoints is None:
                        continue
                    keypoints = r.keypoints.xy.cpu().numpy()
                    for person in keypoints:
                        if len(person) < 17:
                            continue
                        head_y = person[0][1]
                        hip_y  = (person[11][1] + person[12][1]) / 2
                        if 0.1 < abs(head_y - hip_y) < 30.0:
                            fall_detected_in_frame = True

                # 연속 감지 필터 (3프레임 연속 시 확정)
                if fall_detected_in_frame:
                    self.fall_count += 1
                    self.missing_count = 0
                else:
                    self.missing_count += 1
                    if self.missing_count > 10:
                        self.fall_count = 0

                # 낙상 확정 발행
                if self.fall_count >= 3:
                    now = time.time()
                    if not self.emergency_sent and (now - self.last_publish_time > 3.0):
                        msg_out = String()
                        msg_out.data = self.current_location
                        self.fall_pub.publish(msg_out)
                        self.emergency_sent = True
                        self.last_publish_time = now
                        self.get_logger().error(
                            f"🚨 [FALL DETECTED] at {self.current_location}"
                        )

                if self.missing_count > 10:
                    self.emergency_sent = False

                # [렉 개선] 구독자 있을 때만 plot() + 인코딩
                if self.viz_pub.get_subscription_count() > 0:
                    annotated_frame = results[0].plot() if results else frame

                    display_text = f"[{self.robot_id}] LOC: {self.current_location}"
                    if self.emergency_sent:
                        display_text += " | EMERGENCY"
                        cv2.rectangle(annotated_frame, (0, 0), (640, 480), (0, 0, 255), 10)
                    cv2.putText(annotated_frame, display_text, (20, 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                    # [렉 개선] 화질 70 → 50
                    _, buf = cv2.imencode(
                        '.jpg', annotated_frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 50]
                    )
                    viz_msg = CompressedImage()
                    viz_msg.header.stamp = self.get_clock().now().to_msg()
                    viz_msg.format = 'jpeg'
                    viz_msg.data = buf.tobytes()
                    self.viz_pub.publish(viz_msg)

            except Exception as e:
                self.get_logger().error(f"Inference Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()