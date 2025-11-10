import rclpy
from rclpy.node import Node

from interfaces.srv import Tts
import openai
from playsound import playsound

class TtsService(Node):
    def __init__(self):
        super().__init__('tts_service')
        self.srv = self.create_service(Tts, 'tts', self.callback)
        self.get_logger().info("TTS 서비스 준비 완료!")
        # OpenAI API 키 설정
        openai.api_key = ""

    def callback(self, request, response):
        # TTS 수행
        # TTS 요청
        res = openai.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=request.text,
        )
        file_path = "../audio/tts.mp3"
        with open(file_path, "wb") as f:
            f.write(res.content)

        playsound(file_path)
        response.done = True
        return response
    
def main():
    rclpy.init()
    node = TtsService()
    rclpy.spin(node)  # 노드를 계속 실행 (서비스 요청 대기)
    rclpy.shutdown()

if __name__ == '__main__':
    main()