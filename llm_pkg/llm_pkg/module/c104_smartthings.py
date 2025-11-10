import requests
import os
from dotenv import load_dotenv

class C104_SmartThings():
    def __init__(self):
        # .env 파일 로드
        load_dotenv()
        # 스마트싱스 API 설정
        self.DEVICE_ID = os.getenv("DEVICE_ID")  # UUID
        print(self.DEVICE_ID)
        self.url_smartthings = "https://ssafyc104.duckdns.org/api/st/devices/commands"

    def send_command(self, command, arguments=None):
        data = {
            "uuid": self.DEVICE_ID,
            "command": command
        }
        try:
            # POST 요청 보내기
            response = requests.post(self.url_smartthings, json=data)

            # 응답 출력
            print("응답 코드:", response.status_code)
            print("응답 데이터:", response.json())

        except requests.exceptions.RequestException as e:
            print("요청 중 오류 발생:", e)

    # TV 전원 켜기
    def turn_on_tv(self):
        self.send_command("on")

    # TV 전원 끄기
    def turn_off_tv(self):
        self.send_command("off")
