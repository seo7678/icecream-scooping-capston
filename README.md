# my-voice

AI가 쓴 티가 안 나고, 내가 직접 쓴 것처럼 보이는 결과물을 만드는 프로젝트.

핵심은 "내 문체" 하나를 학습시키는 게 아니다. **분야와 결과물 형태마다 내가 쓰는 방식이 다르다.**
보고서, 카톡, 발표 대본, 코드 주석, 회의록은 각각 따로 학습한다.

- 목표·성공 기준·단계별 계획: [`docs/PLAN.md`](docs/PLAN.md)
- 기존 아이스크림 스쿠핑 캡스톤 자료: [`old/`](old/) (전부 그대로 보존, 건드리지 않음)

## 지금 되는 것

```
python3 pipeline/mv.py slop 글.md                       AI 티 패턴 검사 (정규식 20종 + 구조 신호)
python3 pipeline/mv.py stats 글.md                      문장 길이, 종결어미, 접속사, 불릿 등 수치
python3 pipeline/mv.py kakao 대화.txt --name 내대화명     카톡 내보내기에서 내 말만 추출
python3 pipeline/mv.py profile 분야 형태                 corpus/분야/형태/ 로 프로파일 수치 생성
python3 pipeline/mv.py compare 분야 형태 초안.md          프로파일에서 벗어난 지표
python3 pipeline/mv.py feedback add 분야 형태 --draft a.md --final b.md   내가 고친 기록
python3 pipeline/mv.py blind make --mine .. --ai .. --system .. --out evals/날짜   블라인드 테스트
```

Claude Code 스킬: `/style-interview` (코퍼스·프로파일 만들기), `/slop-diagnose` (AI 글과 대조), `/write-as-me 분야 형태 주제` (초안 작성).
내 글 모으는 법은 [`docs/COLLECT.md`](docs/COLLECT.md). 이 레포는 public 이라 내 글은 git에 안 올라가게 막아 뒀다.

## 폴더

| 폴더 | 역할 |
|---|---|
| `corpus/` | 내가 직접 쓴 원본 글. `분야/형태/` 로 정리. 기본적으로 git 제외 |
| `profiles/` | 분야×형태별 스타일 프로파일 (규칙 + 수치 + 대표 예문) |
| `slop/` | AI 티 나는 패턴 목록 (금지/주의) |
| `pipeline/` | 분석·생성·평가 스크립트 |
| `evals/` | 블라인드 테스트, 슬롭 점수 기록 |
| `feedback/` | AI 초안 vs 내가 고친 최종본 (가장 값진 학습 데이터) |
| `baseline/` | 스타일 지시 없는 AI 초안 (대조용, git 제외) |
| `tests/` | 단위 테스트와 합성 픽스처 |
| `.claude/skills/` | write-as-me, style-interview, slop-diagnose |
| `old/` | 이전 프로젝트 전체 |
