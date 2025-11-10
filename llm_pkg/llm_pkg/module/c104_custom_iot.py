import requests
import os
from dotenv import load_dotenv

class C104_CustomIoT():
    def __init__(self):
        self.url_iot = "http://192.168.0.19:8000/"

    def send_command(self, mode):
        # 현재 device를 구분하는 iot 코드는 없음
        try:
            data = {"null"}
            # POST 요청 보내기
            response = requests.post(self.url_iot + mode)

        except requests.exceptions.RequestException as e:
            print("요청 중 오류 발생:", e)