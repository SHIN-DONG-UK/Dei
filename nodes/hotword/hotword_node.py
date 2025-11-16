"""
[REFACTORING]
기존: 타이머 -> 반복 getFrame -> Hotword 검사 -> publish
수정: MicStream -> 새 프레임 준비 -> 콜백 -> Hotword 검사 -> publish
"""
import pyaudio
from typing import Callable
import numpy as np
from eff_word_net import RATE
from eff_word_net.audio_processing import Resnet50_Arc_loss
from eff_word_net.engine import HotwordDetector, MultiHotwordDetector

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory


AudioFrameCallback = Callable[[np.ndarray], None]

class CustomAudioStream :
    """
    CustomAudioStream implementation allows developers to use 
    any 16000Hz sampled audio streams with inference engine

    It tries to add sliding window to audio streams
    """

    """
    [REFACTORING]
    콜백 함수 추가 -> 프레임이 완성되면 바로 callback 함수를 호출하도록 수정

    """
    def __init__(
        self,
        open_stream:Callable[[],None],
        close_stream:Callable[[],None],
        get_next_frame:Callable[[],np.array],
        window_length_secs = 1,
        sliding_window_secs:float = 1/8,
        frame_callback: AudioFrameCallback = None,
        ):

        self._open_stream = open_stream
        self._close_stream = close_stream
        self._get_next_frame = get_next_frame
        self._window_size = int(window_length_secs * RATE)
        self._sliding_window_size = int(sliding_window_secs * RATE)

        self._out_audio = np.zeros(self._window_size) #blank 1 sec audio
        print("Initial S",self._out_audio.shape)

        # [REFACTORING]
        self._callback = frame_callback
        self._running = False

    def start_stream(self):
        self._out_audio = np.zeros(self._window_size)
        self._open_stream()
        self._running = True

        for i in range(RATE//self._sliding_window_size -1):
            self.getFrame()
        
        # [REFACTORING]
        import threading

        def stream_loop():
            while self._running:
                frame = self._get_next_frame()
                self._append_frame(frame)
                if self._callback:
                    self._callback(self._out_audio)

        threading.Thread(target=stream_loop, daemon=True).start()

    def close_stream(self):
        self._close_stream()
        self._out_audio = np.zeros(self._window_size)
        # [REFACTORING]
        self._running = False

    def getFrame(self):
        """
        Returns a 1 sec audio frame with sliding window of 1/8 sec with 
        sampling frequency 16000Hz
        """

        new_frame = self._get_next_frame()

        #print("Prior:", self._out_audio.shape, new_frame.shape )
        assert new_frame.shape == (self._sliding_window_size,), \
            "audio frame size from src doesnt match sliding_window_secs"


        self._out_audio = np.append(
                self._out_audio[self._sliding_window_size:],
            new_frame 
        )

        #print(self._out_audio.shape)

        return self._out_audio
    
    # [REFACTORING]
    def _append_frame(self, new_frame):
        #슬라이딩 윈도우
        assert new_frame.shape == (self._sliding_window_size,)
        self._out_audio = np.append(
                self._out_audio[self._sliding_window_size:],
            new_frame 
        )

class SimpleMicStream(CustomAudioStream) :

    """
    Implements mic stream with sliding window, 
    implemented by inheriting CustomAudioStream
    """
    def __init__(self,window_length_secs=1, sliding_window_secs:float=1/8, device_index:int=0, frame_callback: AudioFrameCallback = None):
        self.p=pyaudio.PyAudio()

        CHUNK = int(sliding_window_secs*RATE)
        print("Chunk size", CHUNK)
        mic_stream=self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=CHUNK,
            input_device_index=device_index
        )

        mic_stream.stop_stream()

        CustomAudioStream.__init__(
            self,
            open_stream = mic_stream.start_stream,
            close_stream = mic_stream.stop_stream,
            get_next_frame = lambda : (
                np.frombuffer(mic_stream.read(CHUNK,exception_on_overflow = False),dtype=np.int16) 
                ),
                 window_length_secs=window_length_secs,
                sliding_window_secs=sliding_window_secs,
                frame_callback=frame_callback,
        )

class WakeupPublisher(Node):
    def __init__(self):
        """
        [publisher]
        node name : wakeup_node
        topic name : hotword
        topic type : ['daya', 'stop', 'follow', 'come']
        """

        super().__init__('wakeup_node')
        qos_profile = QoSProfile(depth=10)
        self.hotword_publisher = self.create_publisher(String, 'hotword', qos_profile)
        # self.publisher_user_command = self.create_publisher(String, 'user_command', 1)
        # self.timer_ = self.create_timer(0.1, self.timer_callback)

        pkg_share_dir = get_package_share_directory('llm_pkg')
        
        # efficientword net
        base_model = Resnet50_Arc_loss()
        stop_hw = HotwordDetector(
            hotword="stop",
            model=base_model,
            reference_file = pkg_share_dir + '/jsons' + '/stop_ref.json',
            threshold=0.71,
            relaxation_time=2,
            # verbose=True
        )
        
        follow_hw = HotwordDetector(
            hotword="follow",
            model=base_model,
            reference_file = pkg_share_dir + '/jsons' + '/follow_ref.json',
            threshold=0.71,
            relaxation_time=2,
            # verbose=True
        )
        come_hw = HotwordDetector(
            hotword="come",
            model=base_model,
            reference_file = pkg_share_dir + '/jsons' + '/come_ref.json',
            threshold=0.71,
            relaxation_time=2,
            #verbose=True
        )
        daya_hw = HotwordDetector(
            hotword="daya",
            model=base_model,
            reference_file = pkg_share_dir + '/jsons' + '/daya_ref.json',
            threshold=0.71,
            relaxation_time=2,
            #verbose=True
        )

        self.multi_hotword_detector = MultiHotwordDetector(
            [stop_hw, follow_hw, come_hw, daya_hw],
            model=base_model,
            continuous=True,
        )

        self.mic_stream = SimpleMicStream(window_length_secs=1.5,
                                          sliding_window_secs=0.5,
                                          device_index=None,
                                          frame_callback=self.process_frame
                                          )
        self.mic_stream.start_stream()

        self.get_logger().info("Say " + " / ".join([x.hotword for x in self.multi_hotword_detector.detector_collection]))

    def process_frame(self, frame):
        result = self.multi_hotword_detector.findBestMatch(frame)
        if None not in result:
            msg = String()
            hotword = result[0].hotword
            msg.data = hotword
            self.hotword_publisher.publish(msg)
            
            self.get_logger().info(f"Detected: {hotword}")

    # def timer_callback(self):
    #     frame = self.mic_stream.getFrame()
    #     result = self.multi_hotword_detector.findBestMatch(frame)
    #     if(None not in result):
    #         msg = String()
    #         hotword = result[0].hotword
    #         msg.data = hotword
    #         self.hotword_publisher.publish(msg)
            
    #         self.get_logger().info(f"Detected: {hotword}")


def main(args=None):
    rclpy.init(args=args)
    node = WakeupPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()