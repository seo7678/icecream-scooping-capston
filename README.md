# Ice Cream Scooping Capstone

하드아이스크림 판매 종사자의 반복 스쿠핑 작업부하를 줄이기 위한 **적응형 반자동 스쿠핑 워크스테이션** 종합설계 프로젝트.

## 핵심 연구질문

> 사람의 위치판단 능력은 유지하면서, 높은 grip force와 반복 손목동작을 발생시키는 스쿠핑 구간만 저자유도 기계구조로 동력화하여 수동 스쿠핑 대비 작업자의 기계적 부하를 줄이면서도 작업속도와 portion consistency를 유지할 수 있는가?

## 현재 설계 철학

- **판단은 사람, 반복적인 힘과 궤적은 기계**
- 완전자율 로봇보다 human-in-the-loop 반자동 구조 우선
- AI/비전/다축 로봇팔은 필요성이 입증될 때만 사용
- 사람의 복잡한 스쿠핑 동작을 1~3 DOF로 단순화
- 모터 전류/토크 기반 부하 적응 제어 검토
- 정량 배식과 자동 ejector는 보조 가치로 검토
- 국부가열은 필수 기능이 아니라 절삭저항 저감 후보

## 현재 유력 시스템

1. 작업자가 원하는 아이스크림 통/위치로 헤드를 이동
2. 버튼 또는 풋스위치 입력
3. Z축 또는 접촉감지로 표면 접근
4. 저자유도 메커니즘이 절입 + sweep + scoop rotation 수행
5. 모터 부하에 따라 feed 속도 조절 / jam 시 정지·후퇴
6. 한 스쿱 성형
7. 자동 상승 및 ejector 배출

## 폴더

- `docs/` — 문제정의, 선행근거, 구조설계, 검증계획
- `prompts/` — Claude/Codex용 조사·설계 프롬프트
- `experiments/` — 추후 실험 데이터/프로토콜
- `cad/` — CAD/기구설계 산출물
- `firmware/` — MCU/모터제어 코드
- `references/` — 논문·특허·제품 링크 정리

## 핵심 KPI 후보

- peak / average applied force
- force-time integral
- scoop torque
- cycle time
- scoop mass 및 CV
- successful scoop rate
- motor current / energy per cycle
- cleaning time

## 상태

**Concept validation / architecture definition 단계**
