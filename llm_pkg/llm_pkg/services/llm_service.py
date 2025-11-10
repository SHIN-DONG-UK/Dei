import rclpy
from rclpy.node import Node
from interfaces.srv import Llm

from langchain_ollama import OllamaLLM # LLM 모델
from langchain_core.prompts import ChatPromptTemplate

class LlmService(Node):
    def __init__(self):
        super().__init__('llm_service')
        self.srv = self.create_service(Llm, 'llm', self.callback)
        self.llm = OllamaLLM(model="exaone-2.4b-4bit")
        self.get_logger().info("LLM 서비스 준비 완료!")
    
    def callback(self, request, response):
        # LLM 수행
        response.output = self.llm.invoke(request.input)
        return response

def main():
    rclpy.init()
    node = LlmService()
    rclpy.spin(node)  # 노드를 계속 실행 (서비스 요청 대기)
    rclpy.shutdown()

if __name__ == '__main__':
    main()