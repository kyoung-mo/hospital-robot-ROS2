import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray
import whisper
import tempfile
import os
import time

KEYWORDS = {
    'medicine': ['약', '약 주세요', '약주세요'],
    'emergency': ['간호사', '의사', '도와줘', '도와주세요', '살려줘'],
    'ok': ['괜찮아', '괜찮아요', '됐어', '됐어요'],
    'trash': ['쓰레기', '쓰레기통', '비워줘'],
}

ROOM_MAP = {
    '101': 'room1',
    '102': 'room2',
}

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        self.get_logger().info('[Whisper] 모델 로딩 중...')
        t0 = time.time()
        self.model = whisper.load_model('tiny')
        self.get_logger().info(f'[Whisper] 모델 로드 완료 ({time.time() - t0:.2f}초)')

        # 현재 호출된 방 (mic_trigger로 업데이트)
        self.current_room = 'room1'

        # 구독: 터틀봇1, 터틀봇2 오디오 토픽
        self.create_subscription(ByteMultiArray, '/robot_1/audio', self.audio_callback, 10)
        self.create_subscription(ByteMultiArray, '/robot_2/audio', self.audio_callback, 10)

        # 구독: mic_trigger → room_id 저장
        self.create_subscription(String, '/robot_1/mic_trigger', self.mic_trigger_callback, 10)
        self.create_subscription(String, '/robot_2/mic_trigger', self.mic_trigger_callback, 10)

        # 발행: 키워드 매칭 결과
        self.pub_medicine  = self.create_publisher(String, '/hospital/medicine_request', 10)
        self.pub_emergency = self.create_publisher(String, '/hospital/emergency_call', 10)
        self.pub_trash     = self.create_publisher(String, '/hospital/trash_request', 10)

        # 발행: emergency_event (room별)
        self.pub_event_room1 = self.create_publisher(String, '/hospital/emergency_event/room1', 10)
        self.pub_event_room2 = self.create_publisher(String, '/hospital/emergency_event/room2', 10)

        self.get_logger().info('[Whisper] 노드 준비 완료. 오디오 토픽 대기 중...')

    def mic_trigger_callback(self, msg: String):
        room_id = msg.data
        self.current_room = ROOM_MAP.get(room_id, 'room1')
        self.get_logger().info(f'[Whisper] 호출 방 업데이트: {room_id} → {self.current_room}')

    def audio_callback(self, msg: ByteMultiArray):
        self.get_logger().info('[Whisper] 오디오 수신. STT 변환 중...')

        audio_bytes = b''.join(msg.data)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            f.write(audio_bytes)
            tmp_path = f.name

        try:
            t1 = time.time()
            result = self.model.transcribe(tmp_path, language='ko')
            stt_time = time.time() - t1
            text = result['text'].strip()
            self.get_logger().info(f'[STT 결과] "{text}" ({stt_time:.2f}초)')

            category = self.match_keyword(text)
            self.handle_result(category)
        finally:
            os.remove(tmp_path)

    def match_keyword(self, text: str) -> str:
        for category, keywords in KEYWORDS.items():
            for kw in keywords:
                if kw in text:
                    return category
        return 'unknown'

    def publish_event(self, event: str):
        msg = String()
        msg.data = event
        if self.current_room == 'room1':
            self.pub_event_room1.publish(msg)
        else:
            self.pub_event_room2.publish(msg)
        self.get_logger().info(f'[→] /hospital/emergency_event/{self.current_room} → "{event}" 발행')

    def handle_result(self, category: str):
        msg = String()

        if category == 'medicine':
            msg.data = 'medicine'
            self.pub_medicine.publish(msg)
            self.get_logger().info('[→] /hospital/medicine_request 발행')
            self.publish_event('idle')

        elif category == 'emergency':
            msg.data = 'emergency'
            self.pub_emergency.publish(msg)
            self.get_logger().info('[→] /hospital/emergency_call 발행')
            self.publish_event('emergency')

        elif category == 'trash':
            msg.data = 'trash'
            self.pub_trash.publish(msg)
            self.get_logger().info('[→] /hospital/trash_request 발행')
            self.publish_event('idle')

        elif category == 'ok':
            self.get_logger().info('[→] 이상 없음. 순찰 복귀')
            self.publish_event('idle')

        else:
            self.get_logger().info('[→] 키워드 미인식. 순찰 복귀')
            self.publish_event('idle')


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
