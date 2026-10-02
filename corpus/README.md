# corpus 규칙

```
corpus/
  <분야>/            예: capstone, school, work, personal
    <형태>/          예: report, meeting-note, kakao, slide-script, code-comment
      2026-09-12_제목.md      내가 직접 쓴 글 한 편
      2026-09-12_제목.meta.md  (선택) 누가 읽었나, 얼마나 공들였나, 맥락
```

- **AI가 쓰거나 AI가 고친 글은 넣지 않는다.** 섞이면 프로파일이 슬롭을 학습한다.
- 퇴고 안 한 초안과 최종본을 둘 다 있으면 같이 둔다 (`_draft`, `_final`).
- 형태당 5편이면 시작 가능, 15편이면 안정적.
