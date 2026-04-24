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

        # 1. 로봇 ID 파라미터 (실행 시 설정: robot_1 또는 robot_2)
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.get_logger().info(f"🤖 Starting YOLO Node for: {self.robot_id}")

        # 2. YOLO 모델 로드 (GPU 우선)
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
        self.lock = threading.Lock()

        # 4. QoS 설정 (이미지 전송 최적화)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 5. 구독자 설정 (C++ Task Manager 토픽 이름과 매칭)
        # 카메라 영상 구독
        self.sub_img = self.create_subscription(
            CompressedImage,
            f'/{self.robot_id}/camera/image_raw/compressed',
            self.image_callback,
            qos_profile
        )

        # ✅ [중요] 친구의 C++ 코드 토픽 이름 규칙에 맞춤
        task_topic = "/task_assignment" if self.robot_id == "robot_1" else "/robot2/task_assignment"
        
        self.sub_task = self.create_subscription(
            String,
            task_topic,
            self.task_callback,
            10
        )

        # 6. 퍼블리셔 설정
        self.fall_pub = self.create_publisher(String, '/hospital/emergency_call', 10)
        self.viz_pub = self.create_publisher(CompressedImage, f'/hospital/yolo_viz/{self.robot_id}/compressed', 1)

        # 7. 추론 스레드 실행
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()

    def task_callback(self, msg):
        """ Task Manager로부터 받은 목적지를 화면에 표시하기 위해 저장 """
        task_data = msg.data
        if task_data == "None" or not task_data:
            self.current_location = "Corridor"
        else:
            self.current_location = task_data
        self.get_logger().info(f"📍 [{self.robot_id}] Location Updated: {self.current_location}")

    def image_callback(self, msg):
        """ 압축 이미지 디코딩 및 프레임 최신화 """
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
            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    frame = self.latest_frame.copy()
                    self.latest_frame = None

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                # YOLO 추론 (GPU 사용)
                results = self.model.predict(frame, verbose=False, device=0)
                annotated_frame = frame.copy()

                for r in results:
                    annotated_frame = r.plot()
                    if r.keypoints is None: continue

                    keypoints = r.keypoints.xy.cpu().numpy()
                    for person in keypoints:
                        if len(person) < 17: continue

                        # 낙상 판별 알고리즘 (임계값 조절 가능)
                        head_y = person[0][1]
                        hip_y = (person[11][1] + person[12][1]) / 2
                        height_diff = abs(head_y - hip_y)

                        if 0.1 < height_diff < 30.0:
                            now = time.time()
                            if now - self.last_publish_time > 3.0:
                                # 낙상 발생 시 현재 위치(Task Manager가 준 곳)를 포함해 전송
                                msg_out = String()
                                msg_out.data = self.current_location
                                self.fall_pub.publish(msg_out)
                                self.get_logger().error(f"🚨 [FALL DETECTED] at {self.current_location}")
                                self.last_publish_time = now

                # 화면 상단에 로봇 정보 및 현재 목표 위치 표시
                display_text = f"[{self.robot_id}] LOC: {self.current_location}"
                cv2.putText(annotated_frame, display_text, (20, 45), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)

                # 시각화 영상 퍼블리시
                if self.viz_pub.get_subscription_count() > 0:
                    _, buf = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
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
