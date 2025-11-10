# 인공지능 로봇 집사 Dei


# 환경 설정
## 1. wakeup_node.py
>#### ⚠️ 순서대로 설치해야 함

#### 1-1. pyaudio
```bash
sudo apt-get install portaudio19-dev python3-dev
pip install pyaudio
```

#### 1-2. tflite-runtime

```bash
pip install tflite-runtime
```

#### 1-3. EfficientWord-Net

```bash
pip install EfficientWord-Net
```

#### 1-4. 기본 설치된 numpy uninstall하고 다시 깔아야 함

```bash
pip uninstall numpy -y
pip install numpy
```

## 2. stream_stt_node
```bash
pip install websockets==13.1
```

## 3. llm_pkg/llm_pkg/polling_node
### 설명
- 주기적으로 서버에 접근하여 온습도 센서 데이터 취득
- offset을 벗어난 경우, 최적 루틴 생성
- 해당 루틴으로 제어할 것인지 사용자에게 질문
- 응답에 대한 동작 실행

### 의존성 패키지
- langchain
```shell
pip install langchain langchain_ollama
```

- openai
```shell
pip install openai
```

### 경로 수정
- C104_Tts()의 speak 함수 : 아래 경로 수정
```
file_path = "/home/god/integration_ws/src/llm_pkg/audio/tts.mp3"
```

# 환경변수 설정
### python-dotenv 설치
```shell
pip install python-dotenv
```

### 사용 방법
```python
import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# 환경변수 사용
api_key = os.getenv("API_KEY")
database_url = os.getenv("DATABASE_URL")

print(f"API Key: {api_key}")
print(f"Database URL: {database_url}")
```

### ~/.bashrc에 반영
```
gedit ~/.bashrc
```

- 아래와 같이 export 적용
```
# API KEys
# OpenAI
export OPENAI_API_KEY="your"
# SmartThings
export SMARTTHINGS_API_TOKEN="your"
export DEVICE_ID="your"
```