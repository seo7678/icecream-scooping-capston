# 내 글 모으는 법

기준은 하나다. **내가 직접 쓴 글만.** AI가 쓰거나 고친 글이 섞이면 프로파일이 슬롭을 학습한다.

## 어디서 얼마나

| 형태 | 출처 | 시작 분량 |
|---|---|---|
| 카톡/메신저 | 카톡 대화 내보내기 | 대화방 1~2개 (내 말 수백 개) |
| 보고서, 과제 | 직접 쓴 한글/워드/노션 파일에서 본문 복사 | 5편 |
| 회의록, 메모 | 노트 앱 내보내기 | 5편 |
| 메일 | 보낸편지함에서 복사 | 5통 |
| 코드 주석, 커밋 메시지 | 아래 git 명령 | 30개 |
| 커뮤니티/블로그 글 | 본문 복사 | 5편 |

## 카톡

1. 카톡 PC/모바일에서 대화방 → 대화 내보내기 → 텍스트 저장
2. 파일을 이 레포 안(예: `corpus/_inbox/팀플방.txt`)에 두고
   ```
   python3 pipeline/mv.py kakao corpus/_inbox/팀플방.txt --name "내 대화명" --domain personal
   ```
   `corpus/personal/kakao/팀플방.md` 가 생긴다.
3. 연달아 보낸 짧은 메시지(5분 안)는 한 덩어리로 묶이고, 사진·이모티콘·삭제된 메시지·링크만 있는 줄은 빠진다.
   옵션: `--gap 분`, `--min-chars 글자수`
4. 이름이 틀리면 대화에 보이는 이름 목록을 알려준다.
5. 다른 사람 말이 섞였는지 열어서 한 번 훑는다. 남의 말투가 코퍼스에 들어가면 안 된다.

## git 커밋 메시지 (내가 직접 쓴 것만)

```
git log --author="내이메일" --pretty=format:%B%n---%n > corpus/dev/commit-message/log.md
```
에이전트가 쓴 커밋(`Co-Authored-By: Claude` 포함)은 제외해야 한다:
```
git log --author="내이메일" --invert-grep --grep="Co-Authored-By: Claude" --pretty=format:%B%n---%n
```

## 파일 이름

`corpus/<분야>/<형태>/YYYY-MM-DD_제목.md`. 날짜 모르면 `undated_01.md`.
퇴고 전 초안이 남아 있으면 `_draft`, 최종본은 `_final` 을 붙여 같이 둔다.

## 개인정보

이 레포는 **public** 이다. 그래서 `.gitignore` 가 `corpus/`, `feedback/`, `baseline/`, 실제 프로파일(`profiles/*.md` 의 원문 예문 포함)을 막고 있다.
- 실수로 올라가는 걸 막는 용도지, 보관 방법이 아니다.
- 클라우드 세션은 끝나면 사라지므로, **내 글은 별도의 private 레포(예: `my-voice-data`)** 에 두고 필요할 때 `corpus/` 로 가져오는 방식을 권장한다. 이 레포를 private 으로 바꾸는 쪽이 더 간단하면 `.gitignore` 의 해당 줄을 지우면 된다.
- 다른 사람의 대화(카톡 상대방 말, 메일 원문)는 올리지 않는다.
