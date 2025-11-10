import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from interfaces.srv import Record
from interfaces.srv import Stt
from interfaces.srv import Llm

from playsound import playsound
import threading
import pyaudio
import wave
import sounddevice as sd
import soundfile as sf


class MasterSubscriberNode(Node):
    def __init__(self):
        super().__init__('master_subscriber_node')
        self.subscription_daya = self.create_subscription(String, 'daya_topic', self.listener_callback_daya, 10)
        self.subscription_command = self.create_subscription(String, 'command_topic', self.listener_callback_command, 10)
        self.get_logger().info("Master Subscriber Node 시작!")

        # var
        self.daya = False
        self.command = False

        # record service client
        self.client_rec = self.create_client(Record, '/record')
        while not self.client_rec.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # stt service client
        self.client_stt = self.create_client(Stt, '/stt')
        while not self.client_stt.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        # llm service client
        self.client_stt = self.create_client(Llm, '/llm')
        while not self.client_stt.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def listener_callback_daya(self, msg):
        self.get_logger().info(f"daya_topic: {msg.data}")
        # 1. 대답 음성 재생 (비동기 실행)
        # self.speak2('/home/god/integration_ws/src/llm_pkg/audio/answer.wav')
        # self.get_logger().info(f'테스트1')
        print("대답 음성 재생 했다 치고")

        # 2. Record Service call
        request = Record.Request()
        future = self.client_rec.call_async(request)
        future.add_done_callback(self.record_response_callback)

    def record_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'서비스 응답: {response.file_dir}')
            # STT Service call
            # 만약 command 명령이 없었다면, 해당 음성으로 STT 수행
            if not self.command:
                request = Stt.Request()
                request.file_dir = response.file_dir
                future = self.client_stt.call_async(request)
                future.add_done_callback(self.stt_response_callback)

        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {str(e)} 1')

    def stt_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'서비스 응답: {response.result}')
            # LLM Service call
            # 만약 command 명령이 없었다면, 해당 음성으로 STT 수행
            if not self.command:
                request = Llm.Request()
                request.input = response.result
                future = self.client_stt.call_async(request)
                future.add_done_callback(self.llm_response_callback)
        
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {str(e)} 2')

    def llm_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'서비스 응답: {response.output}')
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {str(e)} 3')
    
    def tts_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'서비스 응답: {response.result}')
            # 3. 대답 음성 재생 (비동기 실행)
            # self.speak2('/home/god/integration_ws/src/llm_pkg/audio/answer.wav')
            # self.get_logger().info(f'테스트2')
            print("대답 음성 재생 했다 치고")

        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {str(e)}')

            
    def listener_callback_command(self, msg):
        self.get_logger().info(f"command_topic: {msg.data}")
        self.command = True

    def speak(self, wav_file):
        # 🔹 재생할 WAV 파일 지정
        WAV_FILE = wav_file
        # 🔹 WAV 파일 열기
        wf = wave.open(WAV_FILE, 'rb')
        # 🔹 PyAudio 객체 생성
        p = pyaudio.PyAudio()
        # 🔹 오디오 스트림 열기
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)

        # 🔹 오디오 데이터를 버퍼 크기만큼 읽어서 재생
        chunk = 1024
        data = wf.readframes(chunk)

        while data:
            stream.write(data)  # 오디오 데이터 출력
            data = wf.readframes(chunk)

        # 🔹 스트림 및 PyAudio 종료
        stream.stop_stream()
        stream.close()
        p.terminate()

        print("Playback finished.")

    def speak2(self, wav_file):
        # 🔹 재생할 WAV 파일 설정
        WAV_FILE = wav_file

        # 🔹 WAV 파일 읽기
        data, samplerate = sf.read(WAV_FILE)

        # 🔹 오디오 출력
        sd.play(data, samplerate)
        sd.wait()  # 재생이 끝날 때까지 대기

        print("Playback finished.")

def main():
    rclpy.init()
    node = MasterSubscriberNode()
    rclpy.spin(node)  # 노드를 계속 실행 (서비스 요청 대기)
    rclpy.shutdown()

if __name__ == '__main__':
    main()