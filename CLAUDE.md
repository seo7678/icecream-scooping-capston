# my-voice

사용자의 글쓰기 스타일을 분야×형태별로 학습해서 AI 티 없는 초안을 만드는 프로젝트. 계획은 `docs/PLAN.md`.

## 규칙
- `old/` 는 이전 프로젝트(아이스크림 스쿠핑 캡스톤)다. 수정하지 않는다. 그 안의 글은 대부분 에이전트가 쓴 것이라 **스타일 코퍼스로 쓰지 않는다.**
- 이 레포는 public 이다. `corpus/`, `feedback/`, `baseline/`, `profiles/*.md`(실제 프로파일)는 git 제외 대상이고, 사용자의 글 원문을 커밋하지 않는다. 테스트 픽스처(`tests/fixtures/`)는 직접 만든 합성 글이다.
- 사용자의 글을 고쳐 쓰거나 "정리"해서 저장하지 않는다. 원문 그대로 둔다.
- 규칙·프로파일은 관찰한 것만 적고, 근거(파일명, 빈도)를 단다. 사용자가 말하지 않은 취향을 지어내지 않는다.
- 글을 쓸 때 사용자가 안 준 경험담·수치를 지어내지 않는다. 비워 두거나 묻는다.
- 한국어로 소통한다. 코드 주석도 한국어, 짧게.

## 도구
- `python3 pipeline/mv.py -h` : stats, slop, profile, compare, feedback, blind, kakao
- 테스트: `python3 -m unittest discover -s tests`
- 의존성 없음 (표준 라이브러리, Python 3.9+)
- 스킬: `write-as-me`(초안 작성), `style-interview`(코퍼스 수집·프로파일), `slop-diagnose`(AI 글과 대조 진단)

## 구조
`corpus/` 원본 → `pipeline/` 분석 → `profiles/` 프로파일 → `write-as-me` 생성 → `feedback/` 수정 기록 → 프로파일 갱신.
슬롭 패턴은 `slop/patterns.json`(기계용)과 `slop/patterns_ko.md`(설명)를 같이 고친다. 내가 쓰는 표현은 `slop/allow.txt`.
