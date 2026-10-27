# R&R · Hole fill · Swap 활성화 동작 점검

2026-09-08, `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`의 현재 소스 기준. `simd capmods adaptive structure-first`로 새로 실행한 검증이다. 기존 운영 소스와 Config는 수정하지 않았다. 진단 테스트 `swarm-core/tests/rr_hole_swap_probe.rs`와 이 보고서만 추가했다. 어제의 전체 감사 이후 소스에 들어온 개선도 현재 상태로 점검했다.

## 결론

**세 연산 모두 그룹이 없는 지원 경로에서 실제로 작동한다. 그러나 설정 ON이 모든 실행에서 적용을 보장하지 않는다.** 가장 큰 제약은 강체 PlaceGroup이 있는 lane 전체에서 세 연산의 개별 조각 버전을 건너뛴다는 점이다. R&R의 NFP 재삽입 설정에는 실행 경로 연결 누락도 있다.

`PlaceGroup`은 여러 조각을 하나의 강체로 묶는 제약을 뜻한다. `size_regions`의 영역 구분이나 개별 Place의 잠금과 동일한 개념이 아니다. 개별 잠금 조각은 장애물로 남고 이동 대상에서 제외되며, 잠금끼리 겹침·Strip 이탈을 허용하는 기존 정책도 유지된다.

## 설정과 실행 조건

| 연산 | 설정 전달 | 실제 실행 조건 | 실제 확인 |
|---|---|---|---|
| R&R | `CExplorationConfig.rnr_period/band_fraction/max_items` → `ExplorationConfig.rnr` (`src/ffi.rs:1211`) | 일반 adaptive Explore, period > 0, 강체 그룹 없음, 배치 4개 이상·이동 가능 2개 이상, disruption 주기 도달 | 직접 호출과 Explore 루프 안에서 실제 재배치 확인 |
| Hole fill | `polish.enabled`, `hole_fill_enabled` → `PolishConfig` (`src/ffi.rs:1365`) | Polish 실행, hole fill ON, 그룹 없음, 남은 예산·후보·적절한 공간 존재 | 닫힌 pocket와 오른쪽으로 열린 bay 모두 실제 재배치·정확 검증 통과 |
| NFP Hole fill | `nfp_integration.nfp_hole_filling` (`src/ffi.rs:1118`) | 위 Hole fill 조건 아래에서 raster 대신 exact NFP 경로 선택 | OFF의 raster 동일성, ON에서 좁은 pocket 이동, 들어갈 곳이 없는 경우의 no-op 검사 통과 |
| Separator swap | Explore/Compress 각각의 `separator.swap_period/attempts/area_ratio` (`src/ffi.rs:1075`) | `capmods` 빌드, 충돌이 남아 있고 반복 번호가 주기에 도달, 그룹 없음, 이동 가능한 서로 다른 item_id의 면적 적합 쌍 | 실제 swap 1회 수락, 가중 충돌 손실 감소 및 비가중 손실 비증가 |
| Explore 교란 swap | adaptive selector의 `SwapDisrupt`, 기본 Explore의 disruption | 분리 실패 후 교란 필요. Separator의 `swap_period`와 별개 | 그룹이 있으면 `disrupt_place_units`로 강체 단위 교란을 사용 |

R&R 주기는 시간·전체 이동 횟수가 아니라 **disruption 횟수** 기준이다. 예를 들어 period=3이면 첫 두 번은 R&R 시점이 아니다. 분리가 잘 되거나 실행 예산이 짧으면 ON이어도 R&R 실행이 없을 수 있다. 순수 NFP 순서 탐색, Escape, Structure 등 별도 경로는 일반 adaptive Explore와 진입·주기 의미가 다르다.

Hole fill은 현재 Strip의 오른쪽 후보 밴드에서 잠금이 아닌 조각을 고른다. 왼쪽 조각까지 전부 뒤집어 재배치하는 전역 연산이 아니다. 또한 앞선 LP·slide 이후에 실행하므로 그 단계에서 이미 공간을 채웠거나 시간이 끝나면 적용되지 않는다. 이동한 조각 수가 늘어도 폭을 결정하는 다른 조각이 남으면 최종 폭은 같을 수 있다.

## 개선이 필요한 사항

### 1. 강체 그룹 한 개 때문에 lane 전체에서 세 연산이 제외됨 — 높은 우선순위

소스 근거:

- `optimizer/explore_mods.rs:663`: R&R selector 자체를 제외한다.
- `optimizer/lns.rs:192`, `:1129`, `:1850`: 일반 R&R, profile R&R, Separator swap이 즉시 반환한다.
- `optimizer/polish/mod.rs:188`, `polish/holes.rs:383`, `:856`: raster/NFP Hole fill을 제외한다.

추가 진단에서 FU 입력에 **단 하나의 강체 그룹**을 만들자 `RNR=0`, `swap_pass=0`, `hole_fill=0`이며 모든 pose가 그대로였다. 그룹에 속하지 않은 다른 조각이 있어도 동일하다.

이는 그룹을 분해하는 잘못된 이동을 막는 의도적인 방어다. 방어문만 제거하면 안 된다. 개선은 우선 그룹을 고정 장애물로 취급한 채 **비그룹 조각만** 이 연산의 후보로 허용하는 방식이 현실적이다. 이후 전체 강체 단위를 위한 envelope·regret·쌍 충돌 비용과 원자적 rollback을 구현해야 그룹 자체도 지원할 수 있다. 설정을 조정해서 해결할 문제는 아니다.

### 2. `nfp_reinsertion=true`의 일반 실행 연결 누락 — 높은 우선순위

`src/ffi.rs:1119`에서 설정값은 정상적으로 받아들인다. 그러나 adaptive Explore의 R&R 호출은 `optimizer/explore_mods.rs:939` 이하에서 모두 `ruin_and_recreate(...)`를 호출하고, 이 함수는 `lns.rs:161`에서 NFP 캐시 대신 `None`을 전달한다.

`NfpReinsertionCache::from_integration_config`와 `ruin_and_recreate_with_nfp`는 구현돼 있지만, 일반 실행 경로에서 해당 설정을 사용해 연결하는 호출은 검색되지 않았다. Escape의 `pipeline.rs:300`은 `exact_separation_vector`용 캐시를 만들 뿐이고 `nfp_reinsertion`을 소비하지 않는다. 해당 파일의 NFP 재삽입 지원 주석은 현재 호출 관계와 맞지 않는다.

따라서 **R&R 자체는 작동하지만 이 추가 옵션을 켰다고 재삽입이 NFP로 바뀌지는 않는다.** 직접 Rust API에 캐시를 넘기는 경우는 별개다. 개선 시 핸들/phase 수명의 캐시를 만들고 설정·캐시를 실제 R&R 호출까지 전달해야 한다. 일반/그룹/취소/실패 fallback 경로의 연결도 함께 확인해야 한다.

### 3. Hole fill / Fragmented R&R의 격자 상한과 중단 처리 — 중간 우선순위

`polish/holes.rs:619`의 셀 크기 계산은 `sqrt(strip_area / max_cells)`를 사용한다. 하지만 실제 할당은 `ceil(width/cell) * ceil(height/cell)`이므로 면적식만으로 셀 수 상한을 보장하지 못한다. 같은 `OccupancyGrid`를 `lns.rs:1802`의 Fragmented 점수 계산도 사용한다.

수식 재현 예: width=1,000,000,000, height=1, max_cells=400,000이면 cell=50, nx=20,000,000, ny=1로 **2천만 셀**, 요청 상한의 **50배**다. 이것은 계산 검증이며, 위험한 대규모 할당을 실제로 실행하지는 않았다. 일반적인 종횡비에서도 ceil 때문에 작은 초과가 가능하다.

또한 raster 구축과 `empty_regions()` 전체 연결 성분 순회에는 중단 확인이 없다. 현재 호출부의 후보 루프 중단 확인만으로는 이 구간을 빠르게 중단할 수 없다. 실제 nx·ny의 checked 곱으로 상한을 강제하고, 긴 raster/flood-fill 순회에 중단 확인을 전달하는 코드 보강이 필요하다.

### 4. Hole fill 반복 비용과 실행 관측 — 중간 우선순위

현재 소스에는 이동 후 옛 공간을 해방하는 개선이 이미 반영돼 있다(`holes.rs:477–483`). 따라서 ‘옛 공간이 계속 점유되어 남는다’를 현재 버그로 보고하지 않는다. 다만 이동마다 `pockets_and_bays()`로 전체 격자를 다시 순회한다. 수락 이동 K개·격자 G셀에 대해 이 부분은 대략 `O(KG)`가 될 수 있다. 정확성을 유지하며 변경 영역 중심으로 pocket 정보를 갱신하거나 재계산을 묶는 방식은 추가 성능 검증 가치가 있다.

Separator swap은 이미 `[SEP] swap: passes, accepted` 통계를 제공한다. 반면 그룹 때문에 호출을 건너뛰거나, R&R의 후보가 없거나, Hole fill 전 예산이 끝나는 경우는 사용자에게 일관되게 드러나지 않는다. 설정 상태와 별도로 `eligible / skipped(reason) / attempted / moved / accepted / elapsed`를 phase 결과에 노출하면 ON인데 무효인 상황을 구분할 수 있다.

`swap_period=0`으로 꺼도 Explore의 교란 swap은 별개로 남는다. 이는 현재 코드의 의미이며, 같은 ‘swap’이라는 이름으로 표시하면 설정 오류처럼 보일 수 있다.

## 실제 실행 결과

| 검증 | 결과 |
|---|---|
| 기존 LNS·후처리·그룹·잠금 회귀 | 20개 통과 |
| 새 활성화·전략별 재배치·그룹 제외 진단 | 3개 통과 |
| raster / NFP hole fill 단위 검사 | 6개 통과 |
| R&R 주기·가용성·no-op·취소 gate 검사 | 6개 통과 |

총 35개 테스트가 통과했다. 전체 프로젝트 재검사나 앱/DLL 배포는 이번 점검에 포함하지 않았다.

- **Swap:** 10회 pass에서 1회 수락. 가중 충돌 손실 **16985.470 → 14046.358**. 조각 수·트래커·비가중 손실 비증가 검사 통과. 한 입력에서의 동작 증거이며 전체 마커 속도/품질 향상률은 아니다.
- **R&R 직접 검사:** Rightmost/LowDensity/Fragmented/GapTarget/Random과 별도 DensitySeam 요청에 대해 3개 시드씩 총 18회 호출. 전부 3–4개 pose가 실제 변경됐고 수량·트래커가 유지됐다. DensitySeam은 일반 adaptive selector의 다섯 연산에 추가됐다는 뜻이 아니다.
- **R&R의 일시적 악화:** Random, seed 42에서 충돌 손실 **356.8123 → 406.4332**. 교란 연산이므로 순간 손실 증가 자체는 버그가 아니다. 후속 Separator와 feasible incumbent 검증이 필요하다. swap처럼 매번 손실 감소를 요구하면 R&R의 탈출 역할을 없애게 된다.
- **R&R 실제 Explore:** 진단용 period=1, 15초 예산에서 `disrupt=4, rnr=4, feasible=1`. 운영 Config는 바꾸지 않았으며 활성화 경로를 통과시키기 위한 테스트 조건이다.
- **기존 테스트의 한계:** 기존 15초 종단 테스트는 이번 실행에서 `disrupt=2, rnr=0`인데도 통과했다. 실제 R&R 호출을 assert하지 않기 때문이다. 새 진단은 R&R 로그 발생과 직접 호출의 pose 변경을 검사한다.
- **Hole fill:** 닫힌 pocket·열린 bay의 실제 이동, 결과의 완전성·정확 충돌 검증, NFP ON에서 raster가 놓치는 좁은 pocket 이동을 확인했다. 들어갈 곳이 없을 때 0회 반환도 정상이다.

로그:

- `target/rr-hole-swap-review-2026-09-08.log`
- `target/rr-hole-swap-probe-2026-09-08.log`
- `target/rr-hole-swap-nfp-holes-2026-09-08.log`
- `target/rr-hole-swap-rnr-gates-2026-09-08.log`

이번 요청은 점검·개선 사항 확인으로 해석했다. 위의 운영 알고리즘 개선안은 적용하지 않았고, 진단 테스트만 추가했다.
