from langchain_ollama import OllamaLLM # LLM 모델
from langchain.prompts.chat import SystemMessagePromptTemplate, HumanMessagePromptTemplate, ChatPromptTemplate

import openai
import os
import json
import requests
from datetime import datetime
from playsound import playsound
import re
from llm_pkg.stream_stt_node import RTZROpenAPIClient
import asyncio

openai.api_key = ""
client = openai.OpenAI(api_key="")  # 새 클라이언트 인스턴스 생성

async def get_transcript():
    CLIENT_ID = "4sddV_p8Qp8fAHrgH3E5"
    CLIENT_SECRET = "P6Z-sKtoUbw_H450F3qjpNOMmM3WrOr3GSz9GM4T"
    client = RTZROpenAPIClient(CLIENT_ID, CLIENT_SECRET)
    transcript = await client.streaming_transcribe()
    return transcript

os.environ["OLLAMA_ACCELERATE"] = "1"
class C104_Tts():
    def __init__(self):
        print("TTS 서비스 준비 완료!")
        # OpenAI API 키 설정
        openai.api_key = ""

    def Do(self, text):
        res = openai.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=text,
        )
        file_path = "../audio/tts.mp3"
        with open(file_path, "wb") as f:
            f.write(res.content)

        playsound(file_path)

llm = OllamaLLM(model="exaone-2.4b-4bit")

def get_current_season():
    today = datetime.today()
    month = today.month
    if month in [12, 1, 2]:
        return "겨울"
    elif month in [3, 4, 5]:
        return "봄"
    elif month in [6, 7, 8]:
        return "여름"
    elif month in [9, 10, 11]:
        return "가을"


# 데이터 가져오기
def get_request():
    url = "https://ssafyc104.duckdns.org/api/weathers/rooms/3"
    
    try:
        response = requests.get(url)
        response.raise_for_status()  # HTTP 오류 발생 시 예외 발생
        data = response.json()  # JSON 응답을 파싱
        print(data)

        room_id = data.get('id')
        name =  data.get('name')
        devices = data.get('devices', {}) 
        weather = data.get('weather', {})
        temp = weather.get('temperature')
        humi = weather.get('humidity')
        matter = weather.get('particulateMatter')

        print(f"현재 온도: {temp}, 습도: {humi}, 미세먼지 농도: {matter}")
        
        return room_id, name, devices, temp, humi, matter
    except requests.exceptions.RequestException as e:
        print(f"요청 오류: {e}")
        return None, None, None

# 제안 데이터 보내기
def post_request(room_id, temp_fix, humi_fix, matter_fix):
    url = f"https://ssafyc104.duckdns.org/api/weathers/inner-sensor/temperature-humidity-to-reach/rooms/{room_id}"
    
    # data = {
    #     "weatherToReach": {
    #         "temperature": temp_fix,
    #         "humidity": int(humi_fix),
    #         # "particulateMatter": matter_fix,
    #     }
    # }
    data = {
        "temperature": temp_fix,
        "humidity": int(humi_fix),
        "particulateMatter": matter_fix,
    }

    try:
        response = requests.post(url, json=data)
        response.raise_for_status()  # HTTP 오류 발생 시 예외 발생
        print(f"POST 요청 성공: {response.status_code}")
        print(f"응답 데이터: {response.json()}")
    except requests.exceptions.RequestException as e:
        print(f"요청 오류: {e}")

# few-show learning prompting
def ask_exaone_yesorno(text):
    system_template = (
        f"""
        사용자 입력의 "긍정", "부정", "요청"으로 구분할 것.
        출력 부분만 출력할 것.
        예시1:
        입력: "좋아"
        출력: 긍정

        예시2:
        입력: "아니"
        출력: 부정
        
        예시3:
        입력: "아니, 에어컨 25도로 해줘"
        출력:
        네, 에어컨 온도를 25도로 맞추겠습니다.
        
        사용자 입력:
        입력: {text}
        출력:
        """
    )
    system_message_prompt = SystemMessagePromptTemplate.from_template(system_template)
    # 사용자 메시지 템플릿 생성
    human_message_prompt = HumanMessagePromptTemplate.from_template("{user_input}")
    chat_prompt = ChatPromptTemplate.from_messages([system_message_prompt, human_message_prompt])
    
    formatted_prompt = chat_prompt.format(user_input=text)
    res = llm.invoke(formatted_prompt)
    print(res)
    return res

# instruction prompting
def ask_exaone(temp, humidity, season):
    # 시스템 프롬프트 템플릿 생성
    system_template = (
        "너는 실내 온도와 습도 조절 AI야. 사용자가 현재 온도와 습도를 입력하면, 온도와 습도를 모두 조정해서 최적의 상태로 변경해야 해. "
        "다른 모든 부가 텍스트 없이 오직 JSON 본문만 출력해 주세요."
        "1단계: 계절에 따른 적정 온도 설정\n"
        "- 계절에 맞는 적정 온도 범위는 다음과 같아:\n"
        "  * 겨울 (12월 ~ 2월)는 18~20°C\n"
        "  * 봄 (3월 ~ 5월)는 20~22°C\n"
        "  * 여름 (6월 ~ 8월)는 24~26°C\n"
        "  * 가을 (9월 ~ 11월)는 20~22°C\n\n"
        "2단계: 온도에 맞는 습도 조정\n"
        "- 최적의 습도는 다음 공식에 따라 계산해: optimal_humidity = 70 - (optimal temperature - 15) * 3.33\n\n"
        "최종적으로 아래와 같은 JSON을 출력해.:\n"
        "{{ \"answer\": (사용자에게 제안하는 말)\"temperature\": (최적의 온도), \"humidity\": (최적의 습도) }}"
    )

    system_message_prompt = SystemMessagePromptTemplate.from_template(system_template)
    # 사용자 메시지 템플릿 생성
    human_message_prompt = HumanMessagePromptTemplate.from_template("{user_input}")
    # 전체 채팅 프롬프트 템플릿 구성
    chat_prompt = ChatPromptTemplate.from_messages([system_message_prompt, human_message_prompt])
    # 사용 예시
    q = f"현재 계절은 {season}이며, 온도는 {temp}도, 습도는 {humidity}%야."
    formatted_prompt = chat_prompt.format(user_input=q)
    # print(formatted_prompt)
    res = llm.invoke(formatted_prompt)
    # ```json 과 ``` 를 제거합니다.
    clean_text = re.sub(r"^```json\n|```$", "", res, flags=re.MULTILINE)
    print(clean_text)

    try:
        parsed_result = json.loads(clean_text)
        print(parsed_result["temperature"], parsed_result["humidity"])
        return parsed_result["answer"], parsed_result["temperature"], parsed_result["humidity"]
    except json.JSONDecodeError as e:
        print("JSON 변환 오류:", e)
        return clean_text
    
# Offset을 구하는 함수
def temp_humi_offset(opti_temp, opti_humi, temp, humi):

    # 최적 온습도값 변환 온도는 3.33의 가중치를 더 줌
    opti_value = opti_temp * 3.33 + opti_humi
    value = temp * 3.33 + humi

    return abs(opti_value - value)

if __name__ == "__main__":
    tts = C104_Tts() # tts
    room_id, name, devices, temp, humi, matter = get_request()  # 현재 데이터 취득(동기)
    
    current_term = get_current_season() # 현재 계절(동기)
    
    response, opti_temp, opti_humi = ask_exaone(temp, humi, current_term) # 취득한 데이터로 답변 생성(동기)

    if temp_humi_offset(opti_temp,opti_humi,temp,humi) >= 10:
        tts.Do(response) # 제안(동기)
        
        transcript_result = asyncio.run(get_transcript()) # 사용자에게 답변 받아오기(동기)

        print(transcript_result)
        
        text = ask_exaone_yesorno(transcript_result)
        print(text)

        if text == "긍정":
            print(1)
        elif text == "부정":
            print(2)
        else:
            tts.Do(text)
            # 요청된 자동화 루틴 실행

