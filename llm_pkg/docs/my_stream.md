# module/my_stream.py
## 1. 상위 클래스: `CustomAudioStream`
### 목적
- **임의의 오디오 소스**(마이크, 파일, 네트워크 등)에서 데이터를 받아
- **슬라이딩 윈도우**로 일정 길이(`window_length_secs`) 오디오 프레임 생성
- ML/Hotword 모델 입력으로 바로 사용 가능하게 만들기 위함

---

### 생성자: `__init__`
```py
def __init__(
        self,
        open_stream:Callable[[],None],
        close_stream:Callable[[],None],
        get_next_frame:Callable[[],np.array],
        window_length_secs = 1,
        sliding_window_secs:float = 1/8
        ):
```
- `open_stream`: 스트림 시작 함수
- `close_stream`: 스트림 종료 함수
- `get_next_frame`: 실제 오디오 데이터를 가져오는 함수
- `window_length_secs`: 출력할 오디오 프레임 길이(예: 1초)
- `sliding_window_secs`: 한 번에 읽는 오디오 조각 길이(예: 1/8초)

### 내부 변수
```py
self._window_size = int(window_length_secs * RATE)
self._sliding_window_size = int(sliding_window_secs * RATE)
self._out_audio = np.zeros(self._window_size)
```

- `_window_size`: 1초짜리 프레임 샘플 수 (16kHz이면 16000)
- `_sliding_window_size`: 한 번에 가져오는 샘플 수
- `_out_audio`: 현재 프레임 저장용 배열, 초기값은 0

---

### `start_stream()`
```py
def start_stream(self):
    self._out_audio = np.zeros(self._window_size)
    self._open_stream()
    for i in range(RATE//self._sliding_window_size -1):
        self.getFrame()
```
- `_out_audio` 초기화
- `open_stream()` 호출 -> 실제 오디오 스트림 시작 (추상화된 함수)
- for문: 초기 프레임을 슬라이딩 윈도우 크기만큼 채움
> 여기서 초기화 안 하면 첫 1초 프레임이 제대로 안 만들어질 수 있음

---

### `get_frame()`
```py
def getFrame(self):
    new_frame = self._get_next_frame()
    assert new_frame.shape == (self._sliding_window_size,)
    self._out_audio = np.append(self._out_audio[self._sliding_window_size:], new_frame)
    return self._out_audio
```
- `_get_next_frame()` 호출 -> 새로운 1/8초 샘플 가져옴
- 슬라이딩 윈도우 방식으로 `_out_audio` 업데이트
  - 기존 프레임에서 오래된 샘플 삭제
  - 새 샘플을 뒤에 붙임
- 결과: 항상 **1초 길이의 최신 오디오 프레임** 반환
> 이렇게 하면 모델이 1초 오디오 단위로 매번 inference 가능

---

## 2. 하위 클래스:`SimpleMicStream`
`CustomeAudioStream`을 상속받아 **실제 마이크 입력 처리** 구현

### 생성자
```py
def __init__(self, window_length_secs=1, sliding_window_secs=1/8, device_index=0):
    self.p = pyaudio.PyAudio()
```
- PyAudio 객체 생성 -> 마이크 사용 준비

```py
CHUNK = int(sliding_window_secs*RATE)
mic_stream = self.p.open(
    format=pyaudio.paInt16,
    channels=1,
    rate=16000,
    input=True,
    frames_per_buffer=CHUNK,
    input_device_index=device_index
)
mic_stream.stop_stream()
```

- `CHUNK` = 1번에 읽을 샘플 수 (예: 1/8초 -> 2000샘플)
- `p.open()` -> PyAudio 마이크 스트림 생성
- `stop_stream()` -> 아직 데이터는 안 가져오고 준비만 함

```py
CustomAudioStream.__init__(
    self,
    open_stream = mic_stream.start_stream,
    close_stream = mic_stream.stop_stream,
    get_next_frame = lambda : np.frombuffer(mic_stream.read(CHUNK, exception_on_overflow=False), dtype=np.int16),
    window_length_secs=window_length_secs,
    sliding_window_secs=sliding_window_secs
)
```
- 부모 클래스 생성자 호출
- 핵심: get_next_frame를 lambda로 정의
  - PyAudio에서 읽은 raw bytes -> numpy array로 변환
- 이렇게 하면 `getFrame()` 호출 시 PyAudio에서 바로 1/8초 오디오 가져와 슬라이딩 윈도우 업데이트

---

## 3. 동작 요약
### 1. `SimpleMicStream.start_stream()` 호출
- PyAudio 스트림 시작
- `_out_audio` 초기화 -> 1초짜리 빈 배열 생성

### 2. `getFrame()` 호출
- 새로운 1/8초 샘플 읽음
- 슬라이딩 윈도우 적용 -> `_out_audio` 업데이트
- 항상 1초 길이 오디오 변환

### 3. ML 모딜(`HotWordDetector`) 입력으로 사용 가능

---

## 4. 그림으로 이해
```diff
MicStream 시작
    │
    ▼
+--------------------+
|  _out_audio (1초)  |
+--------------------+
        │
        ▼
getFrame() 호출
   ┌───────────┐
   │  새로운    │  ← 1/8초 오디오
   │  슬라이딩  │
   └───────────┘
   │ append → _out_audio 업데이트
        │
        ▼
항상 최신 1초 오디오 반환 → HotwordDetector 입력

```