import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import openai
import threading
import os
from dotenv import load_dotenv
from pathlib import Path

from pydub import AudioSegment
import simpleaudio as sa

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
        self.hotword_subscriber = self.create_subscription(
            String,
            'hotword',
            self.subscribe_hotword,
            qos_profile
        )

        # TTS 재생 객체 및 플래그
        self.play_obj = None
        self.stop_flag = False
        self.play_lock = threading.Lock()

        # OpenAI API 키 설정
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("OpenAI API 키가 설정되지 않았습니다. .env 파일을 확인하세요.")

        print("TTS 준비 완료!")

    # hotword 수신 → 재생 중지
    def subscribe_hotword(self, msg):
        self.get_logger().info("Hotword detected — TTS 중지합니다")

        with self.play_lock:
            self.stop_flag = True

            if self.play_obj is not None and self.play_obj.is_playing():
                self.play_obj.stop()
                self.get_logger().info("재생 중단 완료")

    def subscribe_llm_result(self, msg):
        self.start_tts_thread(msg.data)

    def subscribe_tts_control(self, msg):
        self.start_tts_thread(msg.data)

    def start_tts_thread(self, text):
        t = threading.Thread(target=self.tts_and_play, args=(text,))
        t.start()

    def tts_and_play(self, text):
        # 이전 재생이 있으면 중지
        with self.play_lock:
            if self.play_obj is not None and self.play_obj.is_playing():
                self.play_obj.stop()
            self.stop_flag = False

        # OpenAI TTS 요청
        res = openai.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=text
        )

        # 음성 파일 저장
        file_path = SRC_DIR / 'audio' / 'tts.wav'
        with open(file_path, "wb") as f:
            f.write(res.content)

        # 재생 시작
        self.play_audio(file_path)

    # 재생 + 중단 체크
    def play_audio(self, filepath):
        with self.play_lock:
            if self.stop_flag:
                return

        audio = AudioSegment.from_file(filepath)
        raw = audio.raw_data

        play_obj = sa.play_buffer(
            raw,
            num_channels=audio.channels,
            bytes_per_sample=audio.sample_width,
            sample_rate=audio.frame_rate
        )

        # 현재 재생 객체 등록
        with self.play_lock:
            self.play_obj = play_obj

        # 재생 중 stop_flag 수시 체크
        while play_obj.is_playing():
            with self.play_lock:
                if self.stop_flag:
                    play_obj.stop()
                    return

        # 재생 완료 후 정리
        with self.play_lock:
            self.play_obj = None


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
