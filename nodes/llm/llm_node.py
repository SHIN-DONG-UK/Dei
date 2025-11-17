import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from langchain_ollama import OllamaLLM
from langchain_core.prompts import PromptTemplate
from langchain_community.vectorstores import Chroma
from langchain_openai import OpenAIEmbeddings
import os
from dotenv import load_dotenv
import json, re

DEVICE = ["TV", "에어컨"]

class RAGEngine():
    def __init__(self, model_name, chroma_path):

        # 환경 변수 로드
        load_dotenv()

        # OpenAI API 키
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("OpenAI API 키가 설정되지 않았습니다. 환경변수를 확인하세요.")
        
        # LLM 구성
        self.llm = OllamaLLM(model=model_name)

        # 벡터 DB 설정
        self.CHROMA_DB_PATH = chroma_path
        self.embedding_model = OpenAIEmbeddings(model="text-embedding-ada-002")
        self.vector_db = Chroma(
            persist_directory=self.CHROMA_DB_PATH, 
            embedding_function=self.embedding_model)
        
        print("RAG Engine 준비 완료!")
    
    def getResult(self, text):
        return self.llm.invoke(text)
    
    def _rank_documents(self, documents, query, top_k=5):
        """문서들을 한 번의 LLM 호출로 평가"""
        docs_text = "\n\n".join(
        [f"[DOC_{i}]\n{doc.page_content}" for i, doc in enumerate(documents)]
        )

        scoring_prompt = f"""
        당신은 문서 관련도를 평가하는 전문가입니다.
        질문: {query}

        아래 문서 각각에 대해 관련도를 1~10 사이 숫자로 평가하세요.
        반드시 JSON 형태로만 출력하세요.
        JSON 이외에 어떠한 출력도 하지 마세요:

        예시:
        {{
        "DOC_0": 7,
        "DOC_1": 3,
        "DOC_2": 9
        }}

        문서 목록:
        {docs_text}
        """

        scores_json = self.llm.invoke(scoring_prompt)
        cleaned = re.sub(r"```(json)?", "", scores_json, flags=re.IGNORECASE).strip()
        cleaned = cleaned.strip()
        print(cleaned)
        scores = json.loads(cleaned)

        # JSON의 키(DOC_0 등)로 정렬
        scored_docs = sorted(
            [(documents[int(k.split("_")[1])], score) for k, score in scores.items()],
            key=lambda x: x[1],
            reverse=True
        )

        return [doc for doc, _ in scored_docs[:top_k]]
    
    def generate_rag_response(self, prompt, query, top_k=5):
        """사용자의 질의(query)에 대해 RAG 기반 답변 생성"""

        # 1. mmr 검색
        search_results = self.vector_db.max_marginal_relevance_search(query, k=top_k, fetch_k=10)

        # 2. 문서 재평가 후 상위 문서 선택
        ranked_docs = self._rank_documents(search_results, query)
        context = "\n\n".join([doc.page_content for doc in ranked_docs])

        # 3. 사용자 정의 prompt를 활용하여 최종 프롬프트 생성
        final_prompt = prompt.format(query=query, context=context)

        # 4. 답변 생성
        answer = self.llm.invoke(final_prompt)

        return answer
    
    def stop(self):
        self.llm.stop()

from pathlib import Path
CURRENT_DIR = Path(__file__).resolve()
SRC_DIR = CURRENT_DIR.parent.parent.parent

class LLMNode(Node):

    def __init__(self):
        super().__init__('llm_node')
        qos_profile = QoSProfile(depth=10)
        self.string_publisher = self.create_publisher(String, 'llm_result', qos_profile)
        self.json_publisher = self.create_publisher(String, 'llm_json', qos_profile)
        self.stt_subscription = self.create_subscription(
            String,
            'stt_stream',
            self.subscribe_stt,
            qos_profile)
        self.engine = RAGEngine('exaone3.5:2.4b', SRC_DIR / 'resource' / 'rag_db')
        self.llm = OllamaLLM(model='exaone3.5:2.4b')
        
        with open(SRC_DIR / 'resource' / 'template' / 'template_classifier.txt', 'r', encoding='utf-8') as f:
            self.template_classifier = f.read()

        with open(SRC_DIR / 'resource' / 'template' / 'template_rag.txt', 'r', encoding='utf-8') as f:
            self.template_rag = f.read()

        with open(SRC_DIR / 'resource' / 'template' / 'template_general.txt', 'r', encoding='utf-8') as f:
            self.template_general = f.read()
        
        with open(SRC_DIR / 'resource' / 'template' / 'template_control.txt', 'r', encoding='utf-8') as f:
            self.template_control = f.read()

    def subscribe_stt(self, msg):
        self.get_logger().info(f"stt received: {msg.data}")
        # 1. 분류기
        prompt = PromptTemplate(
            input_variables=["input_text}"],
            template=self.template_classifier
        )
        chain = prompt | self.llm
        input = prompt.format(input_text=msg.data)
        result = chain.invoke(input)

        # 2. JSON or 텍스트 생성
        if result == 'qna_iot':
            print(result)
            # RAG 기반 답변
            prompt = PromptTemplate(
                input_variables=["context", "query"],
                template=self.template_rag
            )

            output = self.engine.generate_rag_response(prompt, msg.data)

            print(output)

            msg = String()
            msg.data = output
            self.string_publisher.publish(msg)
            
        elif result == 'qna_general':
            print(result)
            # web 검색 허용해서 답변해야 하지만 여기서는 안할거임
            prompt = PromptTemplate(
                input_variables=["query"],
                template=self.template_general
            )

            chain = prompt | self.llm
            output = chain.invoke(msg.data)

            print(output)

            msg = String()
            msg.data = output
            self.string_publisher.publish(output)

        elif result == 'control':
            print(result)
            # 제어 명령 수행
            prompt = PromptTemplate(
                input_variables=["device_list", "user_input"],
                template=self.template_control
            )
            chain = prompt | self.llm
            result = chain.invoke({
                "device_list": ", ".join(DEVICE), 
                "user_input": msg.data
            })

            print(result)

            msg = String()
            msg.data = result
            self.json_publisher.publish(result)

        else:
            print(result)
            # 적당한 답변 생성해서 말하기
            msg = String()
            msg.data = "이런 질문은 아직 처리할 수 없어요. 다른 질문을 해주세요."
            self.string_publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()