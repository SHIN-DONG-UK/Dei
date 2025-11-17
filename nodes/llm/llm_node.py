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


class TextToJSONPublisher(Node):

    def __init__(self):
        super().__init__('textToJSON_publisher')
        qos_profile = QoSProfile(depth=10)
        self.textToJSON_publisher = self.create_publisher(String, 'texttojson', qos_profile)
        self.stt_subscription = self.create_subscription(
            String,
            'stt_stream',
            self.subscribe_stt,
            qos_profile)
        self.engine = RAGEngine('exaone3.5:2.4b', '/home/god/robot_ws/src/resource/rag_db')
        self.llm = OllamaLLM(model='exaone3.5:2.4b')
        
        self.template_classifier = """
        당신의 임무는 사용자의 입력을 네 가지 중 하나로 분류하는 것입니다.

        [출력 규칙]
        - 오직 다음 중 하나만 출력해야 합니다.
        - qna_iot
        - qna_general
        - control
        - error
        - 다른 문구, 설명, 이유, 따옴표는 절대 출력하지 마세요.

        [분류 규칙]

        1. qna_iot  
        - 사용자가 집 내부의 IoT 기기, 스마트홈 기기, 센서, 가전제품 등에 대한 정보를 질문하는 경우  
        - 예:
            - "우리 집 온도 센서가 어떻게 작동해?"
            - "IoT 에어컨 자동모드가 뭐야?"
            - "스마트락 배터리는 얼마나 가?"

        2. qna_general  
        - 일반 지식/설명/정보를 요청하는 경우
        - IoT 기기 및 스마트홈과 무관한 질문
        - 예:
            - "세종대왕은 누구야?"
            - "광합성이 뭐야?"
            - "로마 제국 역사를 알려줘"

        3. control  
        - 장치 또는 시스템의 행동을 요구하는 명령
        - IoT 기기 제어 포함
        - 예:
            - "불 켜"
            - "에어컨 꺼"
            - "문 열어줘"
            - "TV 볼륨 올려"

        4. error  
        - 위 세가지 중 어디에도 명확히 속하지 않는 경우
        - 잡담, 감정 표현, 상태 묻기
        - 예:
            - "기분 어때?"
            - "너 누구야?"
            - "고마워"

        [중요]
        - 절대 질문에 대답하지 마세요.
        - 당신은 '분류기'이며, 입력을 분류하는 것 외에 아무것도 해서는 안 됩니다.

        [예시]
        입력: "세종대왕에 대해 알려줘" → qna_general
        입력: "에어컨 필터는 어떻게 관리해?" → qna_iot
        입력: "전등 켜줘" → control
        입력: "오늘 기분 좋다" → error

        이제 아래 입력을 분류하세요.
        입력: "{input_text}"
        """

        self.template_rag = """
        당신은 스마트홈 AI 비서입니다. 제공된 문서 내용(context)만 참고하여 답변하세요.

        규칙:
        - 문서 내용만 사용하고, 외부 지식은 사용하지 마세요.
        - 사용자가 IoT 기기 제어 또는 문제 해결을 물으면 단계별로 안내하세요.
        - 문서에 답이 없으면 "제공된 문서 내에서는 알 수 없습니다."라고 답하세요.
        - 문서에 없는 기기 기능을 추측하거나 만들어내지 마세요.

        [문서 내용]
        {context}

        [사용자 질문]
        {query}

        [답변]
        """

        self.template_general = """
        당신은 스마트홈 AI 비서입니다. 사용자의 질문에 정확하고 구체적이지만 너무 길지 않게 답변하세요.
        
        규칙:
        - 답변을 세 문장으로 요약하세요.
        - 더 궁굼한 사항이 있으면 구체적으로 질문해달라고 하세요.

        [사용자 질문]
        {query}

        [답변]
        """

        self.template_control = """
        당신은 스마트홈 제어 AI입니다. 사용자가 입력한 제어 명령을 JSON으로 반환하세요.

        규칙:
        1. 사용자가 제어할 수 있는 기기는 아래 리스트에만 해당됩니다.  
        2. 리스트에 없는 기기는 제어할 수 없으므로 device와 command, value 모두 null로 설정하세요.
        3. 제어 명령이 값(예: 온도, 채널, 밝기)을 포함하면 value에 숫자 또는 문자열로 넣어주세요.
        4. 항상 JSON 형식으로 출력하세요. 백틱이나 마크다운은 사용하지 마세요.

        제어 가능한 기기 목록:
        {device_list}

        사용자 입력:
        {user_input}

        출력 형식 예시:
        {{"device": "<기기 이름 또는 null>", "command": "<명령 또는 null>", "value": "<값 또는 null>"}}

        """

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

        # 2. JSON 생성
        if result == 'qna_iot':
            print(result)
            # RAG 기반 답변
            prompt = PromptTemplate(
                input_variables=["context", "query"],
                template=self.template_rag
            )

            output = self.engine.generate_rag_response(prompt, msg.data)
            print(output)
            
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

        else:
            print(result)
            # 적당한 답변 생성해서 말하기




def main(args=None):
    rclpy.init(args=args)
    node = TextToJSONPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()