#!/home/god/robot_ws/src/llm_pkg/llm_pkg/venv_stt/bin/python3
# ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
# stt
import queue
import re
import sys

from google.cloud import speech

import pyaudio
import threading

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class MicrophoneStream:
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self: object, rate: int = RATE, chunk: int = CHUNK) -> None:
        """The audio -- and generator -- is guaranteed to be on the main thread."""
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self: object) -> object:
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1,
            rate=self._rate,
            input=True,
            output=False, # 추가
            frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
            input_device_index=None
        )

        self.closed = False

        return self

    def __exit__(
        self: object,
        type: object,
        value: object,
        traceback: object,
    ) -> None:
        """Closes the stream, regardless of whether the connection was lost or not."""
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(
        self: object,
        in_data: object,
        frame_count: int,
        time_info: object,
        status_flags: object,
    ) -> object:
        """Continuously collect data from the audio stream, into the buffer.

        Args:
            in_data: The audio data as a bytes object
            frame_count: The number of frames captured
            time_info: The time information
            status_flags: The status flags

        Returns:
            The audio data as a bytes object
        """
        self._buff.put(in_data)
        return None, pyaudio.paContinue
    
    def generator(self: object) -> object:
        """Generates audio chunks from the stream of audio data in chunks.

        Args:
            self: The MicrophoneStream object

        Returns:
            generator 객체 
            (오디오 청크를 리턴하는 generator 객체임 
            -> yeild로 for-loop이나 next가 호출될 때마다 리턴)
        """
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b"".join(data)


def listen_print_loop(responses: object) -> str:
    """Iterates through server responses and prints them.

    The responses passed is a generator that will block until a response
    is provided by the server.

    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.

    Args:
        responses: List of server responses

    Returns:
        The transcribed text.
    """
    num_chars_printed = 0
    for response in responses:
        if not response.results:
            continue

        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = " " * (num_chars_printed - len(transcript))

        if not result.is_final:
            sys.stdout.write(transcript + overwrite_chars + "\r")
            sys.stdout.flush()

            num_chars_printed = len(transcript)

        else:
            print(transcript + overwrite_chars)

            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            if re.search(r"\b(exit|quit)\b", transcript, re.I):
                print("Exiting..")
                break

            num_chars_printed = 0

    return transcript

class StreamSttNode(Node):
    
    def __init__(self):
        super().__init__('stream_stt_node')
        qos_profile = QoSProfile(depth=10)
        self.stream_stt_publisher = self.create_publisher(String, 'stt_stream', qos_profile)
        self.daya_subsciber = self.create_subscription(
            String,
            'hotword',
            self.subscribe_daya,
            qos_profile)
        
        self.stt_thread = None
        self.stt_stop_event = threading.Event()

    """
    [설명]
    daya 호출 -> stream generator -> stt 수행
    """
    def subscribe_daya(self, msg):
        self.get_logger().info(f"Hotword received: {msg.data}")
        if msg.data != 'daya':
            return

        # 기존에 STT가 실행중이면 중단
        if self.stt_thread and self.stt_thread.is_alive():
            self.get_logger().info("기존 STT 중단 중...")
            self.stt_stop_event.set()
            self.stt_thread.join()
            self.get_logger().info("기존 STT 종료 완료")

        # 새 이벤트 초기화 후 새 STT 시작
        self.stt_stop_event = threading.Event()
        self.stt_thread = threading.Thread(target=self.run_stt, args=(self.stt_stop_event,))
        self.stt_thread.start()
        self.get_logger().info("새 STT 시작")


    def run_stt(self, stop_event):
        language_code = "ko-KR"
        client = speech.SpeechClient()
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code,
            enable_automatic_punctuation=True,
        )

        streaming_config = speech.StreamingRecognitionConfig(
            config=config,
            interim_results=True,
            single_utterance=True,
        )

        with MicrophoneStream(RATE, CHUNK) as stream:
            audio_generator = stream.generator()
            requests = (
                speech.StreamingRecognizeRequest(audio_content=content)
                for content in audio_generator
            )

            responses = client.streaming_recognize(streaming_config, requests)
            print("===음성 인식 시작===")

            try:
                for response in responses:
                    if stop_event.is_set():
                        stream.__exit__(None, None, None)
                        break
                    
                    if response.speech_event_type and response.speech_event_type == 1:
                        print("==============================================")
                        print("<음성 중단> 더 이상 CHUNK를 생성하지 않습니다.")
                        print("==============================================")
                        stream.__exit__(None, None, None)
                        
                    for result in response.results:
                        print("[실시간 텍스트]:", result.alternatives[0].transcript)
                        if result.is_final:
                            print("[최종 텍스트]:", result.alternatives[0].transcript)
                            stream.__exit__(None, None, None)
                            return
                        
            except Exception as e:
                print("STT 중 오류:", e)


def main(args=None):
    rclpy.init(args=args)
    node = StreamSttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()