# 인공지능 로봇 집사 Dei

## 프로젝트 의도
이미 컴퓨팅 파워를 갖춘 로봇청소기에 집사 기능을 추가하면 괜찮지 않을까? 하는 생각으로부터 해당 프로젝트를 시작하게 되었습니다. Jeton Orin nano 보드에 로봇청소기 기능과 AI 기능을 통합하고자 했습니다.

본 레포지토리에는 로봇 집사의 AI 파트만 정리되어 있습니다.

## AI 파트
### Hotword + STT 노드 연계

<img src="resource/images/hotword+stt.gif">

### Hotword + STT + LLM 노드 연계

<img src="resource/images/hotword+stt+llm.gif">

### rqt_graph
<img src="./resource/images/all_graph.png">

<br>

## 시나리오

## 노드별 설명

## 설치 방법
### 1. src 폴더 생성 후 git clone
```bash
git clone https://github.com/SHIN-DONG-UK/Dei.git
```

### 2. 노드별 python 가상환경 구축
#### 2-1. src 폴더와 같은 레벨에 envs 폴더 생성
#### 2-2. envs안에 다음 가상환경 생성
#### 2-3. hotword  
  ```bash
  python3 -m venv hotword_venv
  pip install -r ../src/nodes/hotword/requirements.txt
  ```
#### 2-4. stt  
  ```bash
  python3 -m venv stt_venv
  pip install -r ../src/nodes/stt/requirements.txt
  ```
#### 2-5. llm  
  ```bash
  python3 -m venv llm_venv
  pip install -r ../src/nodes/llm/requirements.txt
  ```
#### 2-6. tts
  ```bash
  python3 -m venv tts_venv
  pip install -r ../src/nodes/tts/requirements.txt
  ```

## 환경변수 설정
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