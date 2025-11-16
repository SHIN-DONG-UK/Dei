# stream_stt_node

## 설명
- hotword('daya')를 subscribe하여 그 이후의 음성을 text로 변환하는 노드
- 이 노드의 결과를 publish해서 llm의 입력으로 사용하도록 함

## 문제1
### 상황
- 'daya'가 subscribe되었을 때, 처음 한 번만 stt가 수행되고 그 이후로 daya를 subscribe하지 않음
- stt부분을 주석처리하면 daya를 잘 subscribe함

```python
def subscribe_daya(self, msg):
    self.get_logger().info(f"Hotword received: {msg.data}")
    if msg.data == 'daya':
        text = String()

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

            print("== 음성 인식 시작 ==")
            for response in responses:
                for result in response.results:
                    print("실시간 텍스트:", result.alternatives[0].transcript)
                    if result.is_final:
                        print("최종 텍스트:", result.alternatives[0].transcript)
```
### Blocking 발생 위치 분석
```
audio_generator = stream.generator()
```
- stream 청크를 생산하는 generator 객체

```
requests = (
                speech.StreamingRecognizeRequest(audio_content=content)
                for content in audio_generator
            )
```

- `for content in audio_generator`
  - content는 마이크에서 생성된 chunk
- `speech.StreamingRecognieRequest(audio_content=content)`
  - Google STT API에 StreamingRecognizeRequest 객체를 생성
- 전체를 괄호로 감싸서
  - `requests`는 generator가 됨
  - 필요할 때마다 다음 chunk를 가져와서 `StreamingRecognizeRequest` 객체를 만들어 냄
- generator라서 한 번에 모든 chunk를 만들지 않고, 요청이 필요할 때마다 생성


```
responses = client.streaming_recognize(streaming_config, requests)
for response in responses:
    ...
```
- 여기서 requests를 하나씩 소비하면서 **실시간 STT 처리**
- generator는 요청이 들어오면 그 때 yield하는 lazy 방식


### 결론
- 현재 generator를 종료하는 코드가 없기 때문에, 한 번만 daya를 subscription 후 blocking
- subscribe callback 함수가 blocking되어 다음 'daya'를 sub할 수 없음
- is_final flag를 맞으면 `__exit()__`을 호출해서 generator를 종료시키면 되지 않을까? => 정답
> 해결완료


## 문제2
### 상황
- 'daya'를 반복 호출했을 때 STT가 중첩되는 문제 발생 -> 먹통
  - 아마 마이크 리소스 충돌로 blocking되는게 아닐까?

### 해결 방법
- STT 실행 중 다시 'daya'가 들어오면  
→ 기존 STT 루프를 즉시 중단하고  
→ 새로 STT를 시작하도록 설계하자

### 구현
- `subscribe_daya()`에서 STT를 스레드로 실행하고,  
'daya'가 들어오면 기존 스레드에 종료 신호를 보내고 새로 시작하는 방식으로 구현할 수 있다.

### 설계
- `stt_thread` -> STT 실행 스레드
- `stt_stop_event` -> 기존 STT 중단 신호
- `subscribe_daya()` -> hotword 수신, STT 스레드 제어

>해결 완료

## 문제3
### 상황
- 'daya'를 호출해놓고 아무말도 안하면 blocking됨

### 해결 방법
- google stt에 `END_OF_SINGLE_UTTERANCE` 플래그가 있는데,  
이게 `streaming_config`에서 `single_utterance=True`로 설정했을 때 발생한다고 함
  - 말하기 시작 전: 스트림 시작 후 일정 시간(약 5~10초) 동안 **아무런 음성 활동이 없을 때** 발생
  - 말하기 중/후: 사용자가 발화한 후 일정 시간 동안 **무음이 지속될 때** 발생
- 이걸 활용하면 되겠다

### 설계
- `END_OF_SINGLE_UTTERANCE` 플래그는 말이 끝났음을 알려주는 이벤트 플래그이고,  
`is_final=True`는 내용이 확정되었을 때 발생한다
- 따라서 `END_OF_SINGLE_UTTERANCE` 플래그를 맞으면 chunk 생성만 중단시킨다(이때까지 말한거로만 최종 결과 받기)
- `is_final=True`이면 thread를 종료하도록 한다

>해결 완료

## 한계
- 한 문장만 들을 수 있음
