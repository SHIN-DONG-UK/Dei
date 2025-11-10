import rclpy
from rclpy.node import Node
from interfaces.srv import LlmPropose

from langchain_ollama import OllamaLLM # LLM 모델


class LlmProposeService(Node):
    def __init__(self):
        super().__init__('llm_propose_service')
        self.srv = self.create_service(LlmPropose, 'llm_propose', self.callback)
        self.llm = OllamaLLM(model="exaone-2.4b-4bit")
        self.get_logger().info("LLM 서비스 준비 완료!")
    
    def callback(self, request, response):
        prompt = f"""
                너는 집사로봇 AI야.
                너가 사용할 수 있는 기기는 {request.devices}가 있어.
                현재 집 내부 온도는 {request.temp}, 습도는 {request.humi}야.
                사용자에게 기기를 제어해서 최적의 온도 {request.opti_temp}와 최적의 습도 {request.opti_humi}로 집안 환경을 개선할 것인지 물어봐.
                최대 3문장으로 말해.
                """
        # LLM 수행
        response.message = self.llm.invoke(prompt)
        return response

def main():
    rclpy.init()
    node = LlmProposeService()
    rclpy.spin(node)  # 노드를 계속 실행 (서비스 요청 대기)
    rclpy.shutdown()

if __name__ == '__main__':
    main()