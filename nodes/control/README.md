# control_node.py

### 설명
- JSON 입력이 들어오면, 해당 JSON을 파싱해서 제어 함수를 호출하는 노드

### 실행 흐름
1. String에서 정규표현식으로 불필요한 요소를 제거하고 JSON 포맷으로 변경
2. device 정규화 -> 한 device를 가리키는 여러 형태를 하나의 형태로 고정
3. command 정규화 -> 한 command를 가리키는 여러 형태를 하나의 형태로 고정
4. device, command, value값을 테이블에서 찾아 함수 실행
