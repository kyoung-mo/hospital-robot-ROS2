import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray

from gtts import gTTS
import tempfile
import os
import time
import threading


TTS_TEXT = "필요한 거 있으실까요?"


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # 구독: task_manager에서 도착 신호 수신 (data: room_id ex. "101")
        self.create_subscription(String, '/hospital/tts_trigger', self.tts_callback, 10)

        # 발행: 터틀봇1 TTS 재생 명령
        self.pub_tts_play = self.create_publisher(ByteMultiArray, '/robot_1/tts_play', 10)

        # 발행: 마이크 녹음 트리거
        self.pub_mic_trigger = self.create_publisher(String, '/robot_1/mic_trigger', 10)

        self.get_logger().info('[TTS] 노드 준비 완료. tts_trigger 대기 중...')

    def tts_callback(self, msg: String):
        room_id = msg.data
        self.get_logger().info(f'[TTS] 도착 신호 수신. room_id: "{room_id}"')

        # 별도 스레드에서 실행 (spin 블로킹 방지)
        thread = threading.Thread(target=self.run_interaction, args=(room_id,))
        thread.start()

    def run_interaction(self, room_id: str):
        # 1. TTS 음성 생성
        self.get_logger().info(f'[TTS] 음성 생성 중: "{TTS_TEXT}"')
        t0 = time.time()

        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            tmp_path = f.name

        try:
            tts = gTTS(TTS_TEXT, lang='ko')
            tts.save(tmp_path)
            self.get_logger().info(f'[TTS] 음성 생성 완료 ({time.time() - t0:.2f}초)')

            with open(tmp_path, 'rb') as f:
                audio_bytes = f.read()

            out_msg = ByteMultiArray()
            out_msg.data = [bytes([b]) for b in audio_bytes]
            self.pub_tts_play.publish(out_msg)
            self.get_logger().info('[→] /robot_1/tts_play 발행 완료')

        finally:
            os.remove(tmp_path)

        # 2. TTS 재생 대기 (약 3초)
        self.get_logger().info('[TTS] 재생 대기 중... (3초)')
        time.sleep(3.0)

        # 3. 마이크 트리거 자동 발행
        mic_msg = String()
        mic_msg.data = room_id
        self.pub_mic_trigger.publish(mic_msg)
        self.get_logger().info(f'[→] /robot_1/mic_trigger 발행 완료 (room: {room_id})')


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
