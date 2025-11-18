# tts_node.py

### 설명
- 음성 변환 요청이 들어오면 openai API로 stt 후 음성 파일을 재생하는 노드
- blocking 방식으로 노드를 구성하고 음성 출력 중 'daya'가 들어오면 음성을 중단한다