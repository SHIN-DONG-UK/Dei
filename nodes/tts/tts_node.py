import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import openai
from playsound import playsound
import threading
import os
from dotenv import load_dotenv
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve()
SRC_DIR = CURRENT_DIR.parent.parent.parent

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        qos_profile = QoSProfile(depth=10)
        self.llm_result_subscriber = self.create_subscription(
            String,
            'llm_result',
            self.subscribe_llm_result,
            qos_profile
        )
        self.tts_control_subscriber = self.create_subscription(
            String,
            'tts_control',
            self.subscribe_tts_control,
            qos_profile
        )
        # OpenAI API 키 설정
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("OpenAI API 키가 설정되지 않았습니다. 환경변수를 확인하세요.")
        print("TTS 준비 완료!")
    
    def subscribe_llm_result(self, msg):
        def _play_sound():
            res = openai.audio.speech.create(
                model="tts-1",
                voice="alloy",
                input=msg.data,)
            
            file_path = SRC_DIR / 'audio' / 'tts.mp3'
            
            with open(file_path, "wb") as f:
                f.write(res.content)

            playsound(file_path)  # TTS 재생

        t = threading.Thread(target=_play_sound)
        t.start()

    def subscribe_tts_control(self, msg):
        def _play_sound():
            res = openai.audio.speech.create(
                model="tts-1",
                voice="alloy",
                input=msg.data,)
            
            file_path = SRC_DIR / 'audio' / 'tts.mp3'
            
            with open(file_path, "wb") as f:
                f.write(res.content)

            playsound(file_path)  # TTS 재생

        t = threading.Thread(target=_play_sound)
        t.start()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()