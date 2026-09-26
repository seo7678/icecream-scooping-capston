# 02. System Concept

## 핵심 철학

**판단은 사람, 반복적인 힘과 궤적은 기계.**

완전자동화하면 아이스크림 표면 재구성, 다음 위치 탐색, 다축 위치제어가 필요하다. 하지만 현재 핵심 문제는 위치판단이 아니라 반복되는 높은 스쿠핑 힘이다.

## 동작 시퀀스

1. 작업자가 헤드를 원하는 아이스크림 통/위치로 이동
2. 풋스위치 또는 버튼 입력
3. 스쿱이 표면까지 접근
4. 접촉/부하 감지
5. 절입
6. sweep + scoop rotation
7. 한 스쿱 성형
8. 상승
9. 컵/콘 위치로 이동
10. ejector 작동

## 모듈 후보

### 1. Horizontal Position Assist
- 수동 슬라이딩 레일
- counterbalance / spring balancer
- low-friction carriage

### 2. Z Surface Approach
- lead screw
- linear actuator
- belt drive
- 접촉력 또는 모터전류 기반 surface detection

### 3. Scooping Mechanism
- 2DOF: translation + scoop rotation
- 3DOF: Z + sweep + rotation
- 1 motor + cam/linkage trajectory 합성도 비교

### 4. Load-Adaptive Control
- motor current ≈ load proxy
- high load → feed speed 감소
- overcurrent → stop/reverse

### 5. Cutting Resistance Reduction
- thin/sharp edge
- attack angle 최적화
- heated edge
- oscillation/vibration

### 6. Portion Control
- penetration depth
- path length
- scoop diameter
- rotation angle

### 7. Auto Ejector
- wiper / trigger mechanism
