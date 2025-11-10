import openai
from playsound import playsound
import threading
import os
from dotenv import load_dotenv

class C104_Tts():
    def __init__(self):
        print("TTS 서비스 준비 완료!")
        # OpenAI API 키 설정
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            raise ValueError("🔴 OpenAI API 키가 설정되지 않았습니다. 환경변수를 확인하세요.")
    def speak(self, text, callback=None):
        """TTS를 실행하고, 끝난 후 callback을 호출"""
        def _play_sound():
            res = openai.audio.speech.create(
                model="tts-1",
                voice="alloy",
                input=text,
            )

            file_path = "/home/c104/S12P11C104/ros/src/AI/llm_pkg/audio/tts.mp3"
            
            with open(file_path, "wb") as f:
                f.write(res.content)

            playsound(file_path)  # TTS 재생

            if callback:
                callback()  # TTS 종료 후 콜백 실행

        t = threading.Thread(target=_play_sound)
        t.start()
    
    def play_audio_file(self, file_path, callback=None):
        """지정된 오디오 파일을 실행하고 끝난 후 callback 호출"""
        def _play_audio():
            if os.path.exists(file_path):
                print(f"재생 중: {file_path}")
                playsound(file_path)  # 오디오 파일 실행
                if callback:
                    callback()  # 파일 재생 종료 후 콜백 실행
            else:
                print("오류: 파일을 찾을 수 없습니다.")

        t = threading.Thread(target=_play_audio)
        t.start()