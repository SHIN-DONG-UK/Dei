import rclpy
from rclpy.node import Node
from interfaces.srv import Record
import webrtcvad
import pyaudio
import wave

class RecordService(Node):
    def __init__(self):
        super().__init__('record_service')
        self.srv = self.create_service(Record, 'record', self.callback)
        self.get_logger().info("녹음 준비 완료!")

        # 녹음 설정
        self.output_file = '/home/god/integration_ws/src/llm_pkg/audio/output.wav'
        self.chunk = 640                            # 20ms
        self.format = pyaudio.paInt16               # 16bit
        self.channels = 1                           # mono
        self.rate = 32000                           # 샘플링 레이트 고정
        self.silence_threshold = 500                # 연속된 무음 시간
        self.DEVICE_INDEX = 6                       # 마이크 인덱스
        # VAD 선언
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)

    def callback(self, request, response):
        # 녹음 수행
        p = pyaudio.PyAudio()
        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        input_device_index=self.DEVICE_INDEX,  # 특정 마이크 지정
                        frames_per_buffer=self.chunk)
        
        self.frames = []
        self.running = True
        
        silence_cnt = 0
        #start_time = time.time()  # 시작 시간 기록

        while self.running:
            data = stream.read(self.chunk)
            self.frames.append(data)

            # 현재 프레임이 음성인지 체크
            is_speech = self.vad.is_speech(data, self.rate)

            #elapsed_time = time.time() - start_time  # 경과 시간 계산
            #print(f"[{elapsed_time:.3f}초] 루프 실행됨")  # 실행 시간 출력
            self.get_logger().info(f'silence_cnt : {silence_cnt * 20}')
            if is_speech:
                silence_cnt = 0 # 초기화
            else:
                silence_cnt += 1
                # threshold보다 크거나 같으면, 종료해야 함
                if 20 * silence_cnt >= self.silence_threshold:
                    self.get_logger().info(f"무음 시간이 {self.silence_threshold}를 넘었습니다.")
                    self.get_logger().info(f"종료.")
                    break
         
        # 스트림 종료
        stream.stop_stream()
        stream.close()
        p.terminate()

        # WAV 파일 저장
        with wave.open(self.output_file, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(p.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(self.frames))

        print(f"녹음 완료: {self.output_file}")
        response.file_dir = self.output_file
        return response  # 응답 반환

def main():
    rclpy.init()
    node = RecordService()
    rclpy.spin(node)  # 노드를 계속 실행 (서비스 요청 대기)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
