# media/ — 구조도·메커니즘 그림·작동 동영상

설명 문서: [`docs/mechanism_explained.md`](../docs/mechanism_explained.md)

| 파일 | 내용 |
|---|---|
| `fig1_overall_axonometric.png` | 전체 구조 축측 투상도 + 구성 요소 14개 |
| `fig2_three_view.png` | 3면도(정면·평면·측면, 같은 축척) + 주요 치수 |
| `fig3_head_mechanism.png` | 스쿠핑 헤드 평행링크(θ −30° / +30° / +90°), push-rod 축력 곡선 |
| `fig4_scoop_sequence.png` | 한 스쿱의 동작(터치오프 → dive → drag → close → lift), 절삭단면 A |
| `fig5_load_path.png` | 반력 경로와 구성별 스쿱 끝 변위 |
| `operation.mp4` / `operation.gif` | 바닐라 싱글 1회 자동 사이클 시뮬레이션(20 s, 실시간) |
| `machine.py` | 공통 치수·색·그리기 함수(값은 `calc/cartesian_model.py`에서 가져옴) |
| `make_figures.py`, `make_operation_video.py` | 생성 스크립트 |

```bash
pip install numpy matplotlib koreanize-matplotlib imageio-ffmpeg
python3 media/make_figures.py
python3 media/make_operation_video.py
```

치수는 설계값, 하중·재료값은 가정값이다. 영상은 실측이 아니다.
