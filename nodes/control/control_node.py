import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import json, re

def set_aircon_temp(value):
    print("arcon 온도 조절: ", value)

def aircon_on():
    print("aricon on")

def aircon_off():
    print("aircon_off")

def tv_on():
    print("tv_on")

def tv_off():
    print("tv_off")

def tv_channel(value):
    print("channel 변경: ", value)

DEVICE_NORMALIZE = {
    "air_conditioner": ["에어컨", "aircon", "ac", "에이에어컨"],
    "tv": ["티비", "TV", "t v", "television"],
}

ACTION_NORMALIZE = {
    "power_on": ["on", "power_on", "power-on", "turn_on", "켜줘", "켜", "작동해"],
    "power_off": ["off", "power_off", "power-off", "turn_off", "꺼줘", "꺼"],
    "set_temperature": ["set_temp", "temperature", "set_temperature", "temp", "온도", "맞춰"],
}

CONTROL_TABLE = {
    "air_conditioner": {
        "set_temp": set_aircon_temp,
        "power_on": aircon_on,
        "power_off": aircon_off
    },
    "tv": {
        "power_on": tv_on,
        "power_off": tv_off,
        "change_channel": tv_channel,
    }
}


class ControlExectueNode(Node):
    def __init__(self):
        super().__init__('control_execute_node')
        qos_profile = QoSProfile(depth=10)
        self.json_subscriber = self.create_subscription(
            String,
            'llm_json',
            self.subscribe_json,
            qos_profile
        )
        self.tts_request_publisher = self.create_publisher(String, 'tts_control', qos_profile)
    
    def subscribe_json(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))

        # 1. 이상한거 제거
        clean_json = re.sub(r"```json\s*|\s*```", "", msg.data)
        print(clean_json)
        parsed_result = json.loads(clean_json)

        # 2. 파싱
        device = self._normalize_device(parsed_result.get("device", "unknown"))
        command = self._normalize_command(parsed_result.get("command", "unknown"))
        value = parsed_result.get("value")

        func = CONTROL_TABLE.get(device, {}).get(command)
        if func:
            func(value) if value else func()
            pub = String()
            pub.data = "요청이 완료되었습니다."
            self.tts_request_publisher.publish(pub)
        else:
            print("해당 디바이스를 제어할 수 없음!")
            pub = String()
            pub.data = "해당 디바이스를 제어할 수 없습니다."
            self.tts_request_publisher.publish(pub)
    
    def _normalize_command(raw_action: str) -> str:
        raw = raw_action.lower().replace(" ", "").replace("-", "_")

        for standard, variants in ACTION_NORMALIZE.items():
            if raw in variants:
                return standard

        # 못찾으면 예외 처리 또는 "unknown"
        return "unknown"

    def _normalize_device(raw_device: str) -> str:
        raw = raw_device.lower().replace(" ", "")

        for standard, variants in DEVICE_NORMALIZE.items():
            if raw in variants:
                return standard
        return "unknown"

    