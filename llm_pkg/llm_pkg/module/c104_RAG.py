from langchain_ollama import OllamaLLM # LLM 모델
from langchain_core.prompts import ChatPromptTemplate
from langchain_community.vectorstores import Chroma
from langchain.embeddings.openai import OpenAIEmbeddings
import os
from dotenv import load_dotenv

class C104_RAG():
    def __init__(self):
        # .env 파일 로드
        load_dotenv()
        # 1. OpenAI API 키 설정 (환경변수에서 가져오기)
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
<<<<<<< HEAD
<<<<<<< HEAD
            raise ValueError("🔴 OpenAI API 키가 설정되지 않았습니다. 환경변수를 확인하세요.")
        # ollamaLLM
        self.llm = OllamaLLM(model="exaone-2.4b-4bit")
        # ChromaDB 컬렉션 초기화
        self.CHROMA_DB_PATH = "/home/god/integration_ws/src/llm_pkg/500_manuals_db"
=======
            raise ValueError("OpenAI API 키가 설정되지 않았습니다. 환경변수를 확인하세요.")
        # ollamaLLM
        self.llm = OllamaLLM(model="exaone-2.4b-4bit")
        # ChromaDB 컬렉션 초기화
        self.CHROMA_DB_PATH = "/home/c104/S12P11C104/ros/src/AI/llm_pkg/rag_db"
>>>>>>> ros2
=======
            raise ValueError("OpenAI API 키가 설정되지 않았습니다. 환경변수를 확인하세요.")
        # ollamaLLM
        self.llm = OllamaLLM(model="exaone-2.4b-4bit")
        # ChromaDB 컬렉션 초기화
        self.CHROMA_DB_PATH = "/home/c104/S12P11C104/ros/src/AI/llm_pkg/rag_db"
>>>>>>> 961d28f (AI : vector store update)
        self.embedding_model = OpenAIEmbeddings(model="text-embedding-ada-002")
        self.vector_db = Chroma(persist_directory=self.CHROMA_DB_PATH, embedding_function=self.embedding_model)
        print("RAG 서비스 준비 완료!")
    
    def getResult(self, text):
        # LLM 수행
        return self.llm.invoke(text)
    
    def generate_rag_response(self, prompt, query, top_k=5):
        """사용자 질의(query)에 대한 RAG 기반 답변 생성"""
        # 1. ChromaDB에서 관련 문서 검색
        results = self.vector_db.max_marginal_relevance_search(
            query, 
            k=5,       # 최종적으로 LLM에게 제공할 문서 5개 가져오기
            fetch_k=10, # 유사도 높은 상위 문서 10개 가져오기
            # filter={"category": "air_conditioner"}  # 에어컨 관련 문서만 검색
        )
        # search_results = self.vector_db.similarity_search(query, k=top_k)

        # 2. 검색된 문서를 기반으로 컨텍스트 생성
        retrieved_context = "\n\n".join([doc.page_content for doc in results])

        # 3. 사용자 정의 prompt를 활용하여 최종 프롬프트 생성
        final_prompt = prompt.format(query=query, context=retrieved_context)

        # 4. OpenAI LLM을 사용하여 응답 생성
        response = self.llm.invoke(final_prompt)
        return response
    
    def rank_documents(self, documents, query):
        """LLM을 활용하여 문서의 관련성을 평가하고 상위 5개 문서 선택"""
        def get_score(doc):
            score_str = self.llm.predict(
                f"질문: {query}\n아래 문서와 질문의 관련성을 1부터 10까지의 숫자로 평가해줘. 오직 숫자만 출력해줘:\n{doc.page_content}"
            ).strip()
            try:
                return float(score_str)
            except ValueError:
                return 0.0  # 변환 실패 시 기본 점수 0 부여

        ranked_docs = sorted(documents, key=get_score, reverse=True)
        return ranked_docs[:5]
    
    def generate_rag_response(self, prompt, query, top_k=5):
        """사용자의 질의(query)에 대해 RAG 기반 답변 생성"""
        # 1. mmr 검색
        search_results = self.vector_db.max_marginal_relevance_search(query, k=top_k, fetch_k=10)

        # 2. 문서 재평가 후 상위 문서 선택
        ranked_results = self.rank_documents(search_results, query)
        retrieved_context = "\n\n".join([doc.page_content for doc in ranked_results])

        # 3. 사용자 정의 prompt를 활용하여 최종 프롬프트 생성
        final_prompt = prompt.format(query=query, context=retrieved_context)

        # 4. 답변 생성
        answer = self.llm.invoke(final_prompt)

        return answer
    
    def stop(self):
        self.llm.stop()