import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray

import whisper
import tempfile
import os
import time


KEYWORDS = {
    'medicine': ['약 주세요', '약주세요'],
    'emergency': ['도와줘', '도와주세요', '살려줘'],
    'ok': ['괜찮아', '괜찮아요', '됐어', '됐어요'],
}


class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        self.get_logger().info('[Whisper] 모델 로딩 중...')
        t0 = time.time()
        self.model = whisper.load_model('tiny')
        self.get_logger().info(f'[Whisper] 모델 로드 완료 ({time.time() - t0:.2f}초)')

        # 구독: 터틀봇1, 터틀봇2 오디오 토픽
        self.create_subscription(ByteMultiArray, '/robot_1/audio', self.audio_callback, 10)
        self.create_subscription(ByteMultiArray, '/robot_2/audio', self.audio_callback, 10)

        # 발행: 키워드 매칭 결과
        self.pub_medicine  = self.create_publisher(String, '/hospital/medicine_request', 10)
        self.pub_emergency = self.create_publisher(String, '/hospital/emergency_call', 10)

        self.get_logger().info('[Whisper] 노드 준비 완료. 오디오 토픽 대기 중...')

    def audio_callback(self, msg: ByteMultiArray):
        self.get_logger().info('[Whisper] 오디오 수신. STT 변환 중...')

        # ByteMultiArray → 임시 wav 파일로 저장
        audio_bytes = bytes(msg.data)
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

    def handle_result(self, category: str):
        msg = String()

        if category == 'medicine':
            msg.data = 'medicine'
            self.pub_medicine.publish(msg)
            self.get_logger().info('[→] /hospital/medicine_request 발행')

        elif category == 'emergency':
            msg.data = 'emergency'
            self.pub_emergency.publish(msg)
            self.get_logger().info('[→] /hospital/emergency_call 발행')

        elif category == 'ok':
            self.get_logger().info('[→] 이상 없음. 순찰 복귀')

        else:
            self.get_logger().info('[→] 키워드 미인식. 순찰 복귀')


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
