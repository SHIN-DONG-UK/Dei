import asyncio
import json
import time
import requests
import websockets
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

API_BASE = "https://openapi.vito.ai"

SAMPLE_RATE = 8000  # 오디오 샘플링 속도
BYTES_PER_SAMPLE = 2  # 한 샘플의 크기 2바이트
CHUNK_SIZE = 1024  # 마이크에서 읽어올 버퍼 크기


class MicrophoneStreamer:
    def __init__(self):
        self.pyaudio = pyaudio.PyAudio()
        self.stream = self.pyaudio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=SAMPLE_RATE,
            input=True,
            frames_per_buffer=CHUNK_SIZE,
            input_device_index=None,
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stream.stop_stream()
        self.stream.close()
        self.pyaudio.terminate()

    async def read(self):
        await asyncio.sleep(CHUNK_SIZE / (SAMPLE_RATE * BYTES_PER_SAMPLE))
        return self.stream.read(CHUNK_SIZE, exception_on_overflow=False)


class StreamSttNode(Node):
    def __init__(self, client_id, client_secret):
        super().__init__('stream_stt_node')
        self.publisher_ = self.create_publisher(String, 'stt_stream', 10)
        self.client_id = client_id
        self.client_secret = client_secret
        self._token = None

    @property
    def token(self):
        if self._token is None or self._token["expire_at"] < time.time():
            resp = requests.post(
                API_BASE + "/v1/authenticate",
                data={"client_id": self.client_id, "client_secret": self.client_secret},
            )
            resp.raise_for_status()
            self._token = resp.json()
        return self._token["access_token"]

    async def streaming_transcribe(self):
        config = {
            "sample_rate": str(SAMPLE_RATE),
            "encoding": "LINEAR16",
            "use_itn": "true",
            "use_disfluency_filter": "false",
            "use_profanity_filter": "false",
            # keyword Boosting : 비슷한 단어이면 이게 가중치 더 높음
            "keywords": "멈춰:4.0, 따라와:4.0, 이리와:4.0, 티비:4.0, 에어컨:4.0"
        }
        STREAMING_ENDPOINT = f"wss://{API_BASE.split('://')[1]}/v1/transcribe:streaming?" + "&".join(f"{k}={v}" for k, v in config.items())
        conn_kwargs = {"extra_headers": [("Authorization", f"bearer {self.token}")]}

        async with websockets.connect(STREAMING_ENDPOINT, **conn_kwargs) as websocket:
            await asyncio.gather(self.streamer(websocket), self.transcriber(websocket))

    async def streamer(self, websocket):
        with MicrophoneStreamer() as mic:
            while True:
                buff = await mic.read()
                if not buff:
                    break
                await websocket.send(buff)
            await websocket.send("EOS")

    async def transcriber(self, websocket):
        async for msg in websocket:
            msg = json.loads(msg)
            if msg.get("final"):
                text = msg["alternatives"][0]["text"]
                self.get_logger().info(f'Publishing: "{text}"')

                # ROS 2 퍼블리시
                ros_msg = String()
                ros_msg.data = text
                self.publisher_.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    
    CLIENT_ID = os.getenv("CLIENT_ID")
    CLIENT_SECRET = os.getenv("CLIENT_SECRET")

    node = StreamSttNode(CLIENT_ID, CLIENT_SECRET)

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.streaming_transcribe())
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
