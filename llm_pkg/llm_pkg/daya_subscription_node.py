import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from playsound import playsound
import threading

from llm_pkg.module.c104_tts import C104_Tts
from llm_pkg.module.c104_RAG import C104_RAG
from llm_pkg.module.c104_smartthings import C104_SmartThings
from llm_pkg.module.c104_custom_iot import C104_CustomIoT

import json
import requests
import re
import time

class DayaSubscriberNode(Node):
    def __init__(self):
        super().__init__('daya_subscription_node')
        self.subscription_daya = self.create_subscription(String, 'daya_topic', self.listener_callback_daya, 10)
        # self.subscription_command = self.create_subscription(String, 'command_topic', self.listener_callback_command, 10)
        self.subscription_stream = self.create_subscription(String, 'stt_stream', self.listener_callback_stream, 1)
        self.publisher_user_command = self.create_publisher(String, 'user_info', 10)
        self.get_logger().info("Master Subscriber Node 시작!")

        # rager
        self.rag = C104_RAG()
        # ttser
        self.tts = C104_Tts()
        # smart things
        self.smart_things = C104_SmartThings()
        # Custom IoT
        self.custom_iot = C104_CustomIoT()
        # var
        self.DEVICES = ["TV", "모니터"] # 집 내부 기기 리스트
        self.daya_flag = False
        self.command_flag = False
        self.threading_daya = None
        # urls

    def listener_callback_daya(self, msg):
        if self.daya_flag == False:
            self.tts.play_audio_file('/home/c104/S12P11C104/ros/src/AI/llm_pkg/audio/answer.wav', callback=self.tts_done_daya)
        
    def listener_callback_stream(self, msg):
        print(msg.data)
        if self.daya_flag == True:

            command_msg = String()

            if "따라와" in msg.data:
                command_msg.data = 'Follow'
                self.publisher_user_command.publish(command_msg)
                print("네 따라가겠습니다.")
                self.tts.play_audio_file('/home/c104/S12P11C104/ros/src/AI/llm_pkg/audio/follow.mp3')

            elif "이리와" in msg.data:
                command_msg.data = 'Come'
                self.publisher_user_command.publish(command_msg)
                print("네 가겠습니다")
                self.tts.play_audio_file('/home/c104/S12P11C104/ros/src/AI/llm_pkg/audio/come.mp3')

            elif "멈춰" in msg.data:
                command_msg.data = 'Stop'
                self.publisher_user_command.publish(command_msg)
                print("네 멈출게요")
                self.tts.play_audio_file('/home/c104/S12P11C104/ros/src/AI/llm_pkg/audio/stop.mp3')
            
            elif "청소" in msg.data:
                command_msg.data = 'Clean'
                self.publisher_user_command.publish(command_msg)
                print("네 청소를 시작합니다.")
                self.tts.play_audio_file('/home/c104/S12P11C104/ros/src/AI/llm_pkg/audio/stop.mp3')
            
            else:
                self.QnAorControl(query=msg.data)
            self.daya_flag = False
        return
    
    def QnAorControl(self, query):
        first_prompt = f"""
                사용자의 입력을 보고 가장 가능성이 높은 말로 해석해.
                예를 들어, "TV 켜죠"는 "TV 켜줘"로 해석할 수 있어.
                또, "TV 꺼져"는 "TV 꺼줘"로 해석할 수 있어.
                뿐만 아니라, 다양한 입력에 대해 한국말의 문맥과 문법에 맞는 해석을 해야 해.
                해석된 내용만 출력해야 해. 다른 텍스트는 붙이지 마.

                만약 도저히 이해할 수 없으면, "error"만 출력해.
                사용자 입력: {query}
                해석:
                """
        
        query = self.rag.getResult(first_prompt)
        print(query)
        if query == "error":
            return

        # propmt로 QnA인지 제어명령인지 LLM이 판단하도록
        prompt = f"""
        너는 집사로봇 AI야. 사용자의 명령을 듣고 집안 IoT 기기를 제어해야 해. 
        예를 들어, TV, 에어컨, 공기청정기, 제습기, 가습기를 제어할 수 있어. 
        사용자의 명령이 들어오면 JSON 형식으로 정리해야 해. 
        반드시 JSON 형식으로만 대답해야 해.

        그런데 만약 사용자가 명령을 내리지 않고 궁금한 사항을 물어본다면,
        너는 JSON 형식으로 답하지 않고 QnA에 대한 답변을 해야 해.
        너는 사용자의 질문이 QnA인지, 제어 명령인지 구분하고 적당한 텍스트를 생성해.

        그리고 제어 명령이든, QnA이든 답변은 다음 JSON 형식으로 답변해.
        제어 명령이면 제어를 하겠다는 답변도 생성해야 해.
        JSON 이외에는 출력하지 마. 아래 JSON 형식으로만 답변해.
        {{
            "response_type": "QnA",
            "question": "{query}",
            "answer": ""
        }}
        {{
            "response_type": "control",
            "question": "{query}",
            "answer": {{
                            "answer": "네, {query}을(를) 수행하겠습니다",
                            "device": "",
                            "value": ""
                    }}
        }}
        {{
            "response_type": "error",
            "question": "{query}",
            "answer": "이해하지 못했어요."
        }}

        질문 : {query}
        
        """

        answer = self.rag.getResult(prompt)
        
        print(answer)
        clean_text = re.sub(r"```json\s*|\s*```", "", answer)
        print(clean_text)
        
        parsed_result = json.loads(clean_text)
        rtype = parsed_result["response_type"]
        
        if rtype == "QnA":
            # answer 음성 재생
            answer = parsed_result["answer"]["answer"]
            self.tts.speak(answer, callback=self.tts_done_callback)

        elif rtype == "control":
            # answer를 다시 parsing해서 제어 함수 실행
            device = parsed_result["answer"]["device"]
            value = parsed_result["answer"]["value"]
            answer_speak = parsed_result["answer"]["answer"]
            
            self.tts.speak(answer_speak, callback=self.tts_done_callback)
            # TV == smart Things / 에어컨 == Custom IoT
            if device == "TV" or device == "티비":
                if value == "켜기" or value == "on" or value == "On" or value=="ON":
                    self.smart_things.turn_on_tv()
                elif value == "끄기" or value == "off" or value == "Off" or value == "OFF":
                    self.smart_things.turn_off_tv()
                else:
                    self.tts.speak("답변을 이해하지 못했습니다.", callback=self.tts_done_callback)
            elif device == "에어컨":
                if value == "켜기" or value == "on" or value == "On" or value=="ON" or value == "켜짐":
                    self.custom_iot.send_command("power_on")
                elif value == "끄기" or value == "off" or value == "Off" or value == "OFF" or value == "꺼짐":
                    self.custom_iot.send_command("power_off")
                else:
                    self.tts.speak("답변을 이해하지 못했습니다.", callback=self.tts_done_callback)
            else:
                print("아직 할 수 없어요.")
        else:
            print("아직 할 수 없어요.")
            return
        
    def tts_done_callback(self):
        time.sleep(0.7)
        print("TTS 종료")
        return
    
    def tts_done_daya(self):
        time.sleep(0.7)
        self.daya_flag = True
        return
    

def main():
    rclpy.init()
    node = DayaSubscriberNode()
    rclpy.spin(node)  # 노드를 계속 실행 (서비스 요청 대기)
    rclpy.shutdown()

if __name__ == '__main__':
    main()