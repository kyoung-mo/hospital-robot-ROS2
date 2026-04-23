import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from ultralytics import YOLO
import numpy as np
import time

class PoseNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # 모델 로드 (경로 확인 필요)
        self.model = YOLO("yolo11n-pose.pt")
        self.current_location = "Corridor"
        self.last_publish_time = 0

        # QoS 설정: 실시간성 확보
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(Image, '/robot_1/camera/image_raw', self.image_callback, qos_profile)
        self.sub_d435 = self.create_subscription(String, '/hospital/fall_suspected', self.d435_callback, 10)
        self.fall_pub = self.create_publisher(String, '/hospital/emergency_call', 10)
        
        self.get_logger().info("✅ Pose Node Started (Sync with D435: 101/102 Mode)")

    def d435_callback(self, msg):
        # D435가 보낸 "101" 또는 "102"를 그대로 위치로 저장
        self.current_location = msg.data
        self.get_logger().info(f"📍 Target Room Updated: {self.current_location}")

    def image_callback(self, msg):
        try:
            # NV21 -> BGR 변환
            raw_data = np.frombuffer(msg.data, dtype=np.uint8)
            yuv_img = raw_data.reshape((msg.height + msg.height // 2, msg.width))
            frame = cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR_NV21)

            results = self.model(frame, verbose=False)
            annotated_frame = frame.copy()

            for r in results:
                annotated_frame = r.plot()
                if r.keypoints is None: continue
                
                keypoints = r.keypoints.xy.cpu().numpy()
                for person in keypoints:
                    if len(person) < 17: continue
                    
                    # 낙상 판별 로직
                    head_y = person[0][1]
                    hip_y = (person[11][1] + person[12][1]) / 2
                    height_diff = abs(head_y - hip_y)

                    # 🚨 임계값 (상황에 맞게 조절)
                    if 0.1 < height_diff < 30.0:
                        current_time = time.time()
                        if current_time - self.last_publish_time > 3.0:
                            # [중요] 확정된 낙상 방 번호를 Task Manager에 전송
                            msg_out = String()
                            msg_out.data = self.current_location # "101" 또는 "102" 전송
                            self.fall_pub.publish(msg_out)
                            
                            self.get_logger().error(f"🚨🚨 [REAL FALL CONFIRMED] Room {self.current_location}!")
                            self.last_publish_time = current_time

            # 화면 출력
            cv2.putText(annotated_frame, f"LOC: {self.current_location}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Hospital AI Monitor", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"YOLO Processing Error: {e}")

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
