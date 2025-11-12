# stream_stt_node

## 설명
- hotword('daya')를 subscribe하여 그 이후의 음성을 text로 변환하는 노드
- 이 노드의 결과를 publish해서 llm의 입력으로 사용하도록 함

## 문제 상황
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