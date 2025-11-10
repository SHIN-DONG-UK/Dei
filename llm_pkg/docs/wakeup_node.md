# wakeup_node

- wakeup_node는 지정된 hotword(예: 데이야, 이리와, 따라와, 멈춰)가 호출되면 해당 단어를 publish하도록 설계된 노드임
- 이를 다른 패키지에서 subscirbe해서 트리거로 사용하도록 하는 노드
- 그런데 이를 timer_callback으로 20ms마다 inference하도록 설계됨
- 이는 CPU 점유율을 높이는 문제가 있음
- 이를 "오디오 프레임 수신 이벤트 기반"처리로 리팩토링할 필요가 있음

### 오디오 프레임 수신 이벤트 기반?
- `SimpleMicStream`은 마이크 입력을 일정 크기의 프레임으로 가져오는 클래스
- 