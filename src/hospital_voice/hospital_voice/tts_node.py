import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ByteMultiArray

from gtts import gTTS
import tempfile
import os
import time

class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # 구독: task_manager에서 TTS 트리거 수신
        self.create_subscription(String, '/hospital/tts_trigger', self.tts_callback, 10)

        self.pub_tts_play = self.create_publisher(ByteMultiArray, '/robot_1/tts_play', 10)
        # 발행: 터틀봇1 TTS 재생 명령
        self.get_logger().info('[TTS] 노드 준비 완료. tts_trigger 대기 중...')

    def tts_callback(self, msg: String):
        text = msg.data
        self.get_logger().info(f'[TTS] 음성 생성 중: "{text}"')

        t0 = time.time()

        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            tmp_path = f.name

        try:
            tts = gTTS(text, lang='ko')
            tts.save(tmp_path)
            self.get_logger().info(f'[TTS] 음성 생성 완료 ({time.time() - t0:.2f}초)')

            with open(tmp_path, 'rb') as f:
                audio_bytes = f.read()

            out_msg = ByteMultiArray()
            out_msg.data = list(bytes([b]) for b in audio_bytes)
            self.pub_tts_play.publish(out_msg)
            self.get_logger().info('[→] /robot_1/tts_play 발행 완료')


        finally:
            os.remove(tmp_path)

def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

   
