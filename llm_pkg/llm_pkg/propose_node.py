import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llm_pkg.module.c104_tts import C104_Tts
from langchain_ollama import OllamaLLM # LLM 모델

from datetime import datetime
import requests
import random
from playsound import playsound
import openai
import threading
import json
import re
import time

class ProposeNode(Node):
    def __init__(self):
        super().__init__('polling_requester')
        self.timer = self.create_timer(5.0, self.timer_callback)  # 10초마다 실행
        self.subscription_stt = self.create_subscription(String, 'stt_stream', self.listener_callback, 1)
        self.tts = C104_Tts()
        self.llm = OllamaLLM(model="exaone-2.4b-4bit")
        print("모델 준비 완료!")

        # var
        self.DEVICES = ["에어컨", "가습기", "제습기", "히터"]
        self.TTS_END = False
        self.USER_TEXT = None
        self.ROUTINE = False
        self.user_turn = False
        self.prompt_yesorno = None

        # optimal propose
        self.temp = None
        self.humi = None
        self.opti_temp = None
        self.opti_humi = None
        self.prompt_propose = None
        self.devices = f"{', '.join(map(str, self.DEVICES))}"
        print(self.devices)
        
        # self.system_message_prompt = SystemMessagePromptTemplate.from_template(self.system_template)
        # self.human_message_prompt = HumanMessagePromptTemplate.from_template("{user_input}")
        # self.chat_prompt = ChatPromptTemplate.from_messages([self.system_message_prompt, self.human_message_prompt])

    # 데이터 가져오기
    def timer_callback(self):
        url = "https://ssafyc104.duckdns.org/api/weathers/rooms/3"
        
        try:
            response = requests.get(url)
            response.raise_for_status()  # HTTP 오류 발생 시 예외 발생
            data = response.json()  # JSON 응답을 파싱
            print(data)

            self.room_id = data.get('id')
            self.name =  data.get('name')
            self.devices = data.get('devices', {}) 
            self.weather = data.get('weather', {})
            self.temp = self.weather.get('temperature')
            self.humi = self.weather.get('humidity')
            self.matter = self.weather.get('particulateMatter')

            self.get_logger().info(f"현재 온도: {self.temp}, 습도: {self.humi}, 미세먼지 농도: {self.matter}")

            self.opti_temp = self.get_optimal_temp()
            self.get_logger().info(f"opti_temp : {self.opti_temp}")
            self.opti_humi = 70 - (self.opti_temp - 15) * 3.33

            # service call을 해야 함
            if self.temp_humi_offset(self.opti_temp, self.opti_humi, self.temp, self.humi) >= 10.0:
                print("자동화 루틴 x")
                # 루틴을 실행하고 있으면, 자동화 루틴 제안하지 않음
                if self.ROUTINE == False:
                    print("자동화 루틴 o")
                    self.ROUTINE = True
                    self.prompt_propose = f"""
                            너는 집사로봇 AI야.
                            너가 사용할 수 있는 기기는 {self.devices}가 있어.
                            현재 집 내부 온도는 {self.temp}, 습도는 {self.humi}야.
                            사용자에게 기기를 제어해서 최적의 온도 {self.opti_temp}와 최적의 습도 {self.opti_humi}로 집안 환경을 개선할 것인지 물어봐.
                            
                            예시1: 
                            사용자님, 현재 집 내부 온도가 {self.temp}, 습도가 {self.humi}입니다. 최적의 온도인 {self.opti_temp}도와 최적의 습도 {self.opti_humi}%로 조정해 드릴까요?
                            
                            예시2: 
                            사용자님, 집 내부 환경을 개선하기 위해 {self.opti_temp}도와 습도 {self.opti_humi}%로 조절해드릴까요?

                            마지막 문장은 항상 변경 사항에 동의할 것인지 물어보는 의문문으로 끝나야 해.
                            """
                    answer = self.llm.invoke(self.prompt_propose)
                    print("answer : ", answer)
                    self.tts.speak(answer, callback=self.tts_done_callback)
                    
        except requests.exceptions.RequestException as e:
            # 아무 것도 하지 않음
            print(f"요청 오류: {e}")
            
    
    # user_turn이면 이 함수 잡고 있고, 뒤에 들어오는거 계속 무시
    def listener_callback(self, msg):
        # print(msg.data)
        if self.user_turn == True:
            self.prompt_yesorno = f"""
                                단어의 정의:
                                "동의" : 요청에 대한 수락(긍정)
                                "비동의" : 요청에 대한 거절(부정)
                                "수정" : 요청을 수정하여 동의

                                answer:
                                사용자 입력을 "동의", "비동의", "수정"으로 구분할 것.
                                사용자가 "동의"했을 경우, 최적의 온도와 습도로 제어하겠다는 대답을 할 것.
                                사용자가 "비동의"했을 경우, 필요할 때 다시 불러달라는 말을 할 것.
                                사용자가 "수정"을 했을 경우, 수정된 요청으로 제어하겠다는 대답을 할 것.

                                반드시 JSON 형식으로만 답변할 것.
                                이외의 텍스트는 생성하지 말 것.

                                동의:
                                {{
                                    "type": "동의",
                                    "user": "좋아 그렇게 해",
                                    "answer": "네 최적의 온도로 제어하겠습니다.",
                                    "value": {{
                                                "temperature" : {self.opti_temp},
                                                "humidity" : {self.opti_humi}
                                            }}
                                }}

                                비동의:
                                {{
                                    "type": "비동의",
                                    "user": "아니",
                                    "answer": "네 알겠습니다.",
                                    "value": null
                                }}
                                
                                수정:
                                {{
                                    "type": "수정",
                                    "user": "아니, 온도 25도로 해줘",
                                    "answer": "네, 온도를 25도로 맞추겠습니다.",
                                    "value":
                                            {{
                                                "temperature" : "25",
                                                "humidity" : "{self.opti_humi}"
                                            }}
                                }}
                                
                                
                                사용자 입력: {msg.data}
                                출력:
                                """
            
            answer = self.llm.invoke(self.prompt_yesorno)
            clean_text = re.sub(r"```json\s*|\s*```", "", answer)
            print(clean_text)
            parsed_result = json.loads(clean_text)

            if parsed_result["type"] == "동의":
                # answer speak
                self.tts.speak(parsed_result["answer"])
                # 서버에 JSON 전송
                data = {
                    "temperature" : self.opti_temp,
                    "humidity" : self.opti_humi,
                }
                json_data = json.dumps(data)  # JSON 문자열로 변환
                #response = requests.post('http://localhost:8000/receive', json=json_data)

            elif parsed_result["type"] == "비동의":
                print("사용자가 동의하지 않았습니다.")
                self.tts.speak(parsed_result["answer"])
                # 얼마동안 제안하지 않기
                self.disagree = True
            else:
                # 수정된 명령
                self.tts.speak(parsed_result["answer"])
                json_data = json.dumps(parsed_result["value"])  # JSON 문자열로 변환
                #response = requests.post('http://localhost:8000/receive', json=json_data)

            self.user_turn = False

    # Offset을 구하는 함수
    def temp_humi_offset(self, opti_temp, opti_humi, temp, humi):
        # 최적 온습도값 변환 온도는 3.33의 가중치를 더 줌
        opti_value = opti_temp * 3.33 + opti_humi
        value = temp * 3.33 + humi

        return abs(opti_value - value)
    
    def get_optimal_temp(self):
        today = datetime.today()
        month = today.month
        if month in [12, 1, 2]:
            return random.randint(18, 20)
        elif month in [3, 4, 5]:
            return random.randint(20, 22)
        elif month in [6, 7, 8]:
            return random.randint(24, 26)
        elif month in [9, 10, 11]:
            return random.randint(20, 22)
    
    def tts_done_callback(self):
        print("TTS 종료, 이제 user_turn을 True로 설정")
        time.sleep(0.7)
        self.user_turn = True

def main(args=None):
    rclpy.init(args=args)
    node = ProposeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()