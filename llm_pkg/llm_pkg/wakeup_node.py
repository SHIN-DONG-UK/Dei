"""
[REFACTORING]
기존: 타이머 -> 반복 getFrame -> Hotword 검사 -> publish
수정: MicStream -> 새 프레임 준비 -> 콜백 -> Hotword 검사 -> publish
"""

from eff_word_net.audio_processing import Resnet50_Arc_loss
from eff_word_net.engine import HotwordDetector, MultiHotwordDetector

from llm_pkg.module.my_stream import SimpleMicStream

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

class WakeupPublisher(Node):
    def __init__(self):
        super().__init__('wakeup_node')
        self.publisher_daya = self.create_publisher(String, 'daya_topic', 10)
        self.publisher_user_command = self.create_publisher(String, 'user_command', 1)
        # self.timer_ = self.create_timer(0.02, self.timer_callback)

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
            hotword = result[0].hotword
            msg = String()
            command_msg = String()
            msg.data = hotword

            if hotword == "daya":
                self.publisher_daya.publish(msg)
            else:
                if msg.data == "come":
                    command_msg.data = 'Come'
                    self.publisher_user_command.publish(command_msg)
                elif msg.data == "stop":
                    command_msg.data = 'Stop'
                    self.publisher_user_command.publish(command_msg)
                elif msg.data == "follow":
                    command_msg.data = 'Follow'
                    self.publisher_user_command.publish(command_msg)
            self.get_logger().info(f"Detected: {hotword}")

    # def timer_callback(self):
    #     frame = self.mic_stream.getFrame()
    #     result = self.multi_hotword_detector.findBestMatch(frame)
    #     if(None not in result):
    #         msg = String()
    #         command_msg = String()
    #         msg.data = result[0].hotword

    #         if msg.data == "daya":
    #             self.publisher_daya.publish(msg)
    #         else:
    #             if msg.data == "come":
    #                 command_msg.data = 'Come'
    #                 # self.publisher_user_command.publish(command_msg)
    #             elif msg.data == "stop":
    #                 command_msg.data = 'Stop'
    #                 # self.publisher_user_command.publish(command_msg)
    #             elif msg.data == "follow":
    #                 command_msg.data = 'Follow'
    #                 # self.publisher_user_command.publish(command_msg)
    #         self.get_logger().info(f"Detected: {result[0].hotword}")


def main(args=None):
    rclpy.init(args=args)
    node = WakeupPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()