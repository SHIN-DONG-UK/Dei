from langchain_ollama import OllamaLLM # LLM 모델
from langchain_core.prompts import ChatPromptTemplate

class C104_Llm():
    def __init__(self):
        self.llm = OllamaLLM(model="exaone-2.4b-4bit")
        print("LLM 서비스 준비 완료!")
    
    def getResult(self, text):
        # LLM 수행
        return self.llm.invoke(text)
    
    def stop(self):
        self.llm.stop()