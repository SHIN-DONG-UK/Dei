import webrtcvad
import pyaudio
import wave

class C104_Record():
    def __init__(self, device_index, threshold=500):
        # 녹음 설정
        self.output_file = '/home/god/integration_ws/src/llm_pkg/audio/output.wav'
        self.chunk = 960
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 48000
        self.silence_threshold = threshold
        self.DEVICE_INDEX = device_index

        # VAD 선언
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)

        # 녹음 상태 변수
        self.running = False

    def getResult(self):
        # 녹음 수행
        p = pyaudio.PyAudio()
        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        input_device_index=self.DEVICE_INDEX,
                        frames_per_buffer=self.chunk)

        self.frames = []
        self.running = True
        silence_cnt = 0

        while self.running:
            data = stream.read(self.chunk)
            self.frames.append(data)

            # 현재 프레임이 음성인지 체크
            is_speech = self.vad.is_speech(data, self.rate)

            print(f'silence_cnt : {silence_cnt * 20}')
            if is_speech:
                silence_cnt = 0
            else:
                silence_cnt += 1
                if 20 * silence_cnt >= self.silence_threshold:
                    print(f"무음 시간이 {self.silence_threshold}ms를 넘었습니다. 종료.")
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
        return True
    
    def stop(self):
        print("녹음을 강제로 중지합니다.")
        self.running = False  # 루프 종료
