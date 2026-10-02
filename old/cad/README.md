# CAD

사양 출처: `docs/11_final_mechanism.md`(기구학·배치), `docs/12_workstation_architecture.md`(치수 초안), `docs/13_engineering_calculations.md`(부품 선정), `docs/16_prototype_v1.md`(V1 서브어셈블리).

## 모델링 순서 (제안)

1. SA5 식품 모듈: 스쿱 + 스템(양끝 밀봉) + **외부 노출 push-rod** + 피벗 + 퀵핀 — 공구 없이 분리되는지 먼저 확인
2. SA3/SA4 헤드: x 캐리지(SFU1610) + θ 레버·반력암
3. SA2 Z 축, SA1 프레임(4040), SA6 통 포스 플랫폼
4. 제품 비전: 웰 도킹 플레이트 + 래치(설계하중 ≥ 0.5 kN)

치수 중 통·스쿱·캐비닛은 **가정값**이므로 P0-4 실측 후 확정(`data/assumptions.md` A06, A10–A12).
