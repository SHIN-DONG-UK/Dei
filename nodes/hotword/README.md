# hotword_node

### 설명
- wakeup_node는 지정된 hotword(예: 데이야, 이리와, 따라와, 멈춰)가 호출되면 해당 단어를 publish하도록 설계된 노드임
- 이를 다른 패키지에서 subscirbe해서 트리거로 사용하도록 하는 노드

### rqt_graph
<img src="../../resource/images/hotword_graph.png">

### 구현 방법1: timer_callback
- 20ms마다 inference하도록 설계
- 이렇게 했을 때 인식 성능이 좋지 않았음
  - 몇 번 씹힘
- 다만 CPU 점유율은 평균적으로 오디오 프레임 이벤트 기반에 비해 낮음

### 구현 방법2: 오디오 프레임 수신 이벤트
- `SimpleMicStream`은 마이크 입력을 일정 크기의 프레임으로 가져오는 클래스
- 정해진 길이의 스트림이 완성될 때마다 추론을 돌려서 추론 결과를 publish하도록 하는 형태
- 고정된 CPU 점유를 갖지만, 인식 성능이 좋음

### 한계
- 애초에 EfficientWord-Net 자체가 CPU 점유율이 높은 솔루션인듯