import rclpy
from rclpy.node import Node
from faster_whisper import WhisperModel
from interfaces.srv import Stt
 
class SttService(Node):
    def __init__(self):
        super().__init__('stt_service')
        self.srv = self.create_service(Stt, 'stt', self.callback)
        self.get_logger().info("STT 준비 완료!")
        self.model = WhisperModel("large-v2", device="cuda", compute_type="int8")
        
    def callback(self, request, response):
        # STT 수행
        log_list = []
        full_text = ""
        segments, info = self.model.transcribe(request.file_dir, beam_size=5, language='ko')

        for segment in segments:
            log_list.append("OUTPUT" + "[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))
            full_text += segment.text

        self.get_logger().info(f"STT 결과: {full_text}")
        response.result = full_text
        return response
     
def main():
    rclpy.init()
    node = SttService()
    rclpy.spin(node)  # 노드를 계속 실행 (서비스 요청 대기)
    rclpy.shutdown()

if __name__ == '__main__':
    main()