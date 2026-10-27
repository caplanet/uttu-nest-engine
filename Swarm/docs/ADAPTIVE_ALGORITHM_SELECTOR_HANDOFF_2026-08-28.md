# Swarm 적응형 알고리즘 선택기 작업 인수인계

작성일: 2026-08-28  
작업 경로: `C:\Caplanet\AiWorks\Claude\SwarmTemp_Anal`  
확인한 기준 리비전: `main` / `c86dbba3a4da5bc88ea3046501cd54fad3ba2c17`

## 1. 요청과 완료 상태

원 요청은 Swarm가 네스팅 중 마주치는 상황을 평가하여, 이미 존재하는 문제 해결
알고리즘 중 가장 적합한 알고리즘을 반환하고 그 반환값을 실제로 실행하는 적응형
알고리즘 적용 경로를 만드는 것이었다.

현재 상태:

- 순수 평가 함수 `evaluate_algorithm()`을 구현했다.
- 평가 결과를 Explore의 disruption 지점에서 실제 알고리즘 실행으로 연결했다.
- 실제 배치 pose를 바꾼 연산만 성과 학습에 반영한다.
- 선택된 R&R이 no-op이면 기존 Swap disruption을 즉시 fallback으로 실행한다.
- 상황 prior가 학습 성과를 영구적으로 압도하지 않도록 점수 범위를 정규화했다.
- GapTarget 진단은 공극 전용 25,000셀 분석으로 제한했다.
- GapTarget 분석 내부에 cooperative Stop 검사를 추가했다.
- 관련 단위·통합·전체 Rust 회귀 테스트를 통과했다.
- 전체 기능 조합의 Release 네이티브 빌드를 완료했다.

중요: 여기서 “최적”은 수학적인 전역 최적 보장이 아니다. 현재 레이아웃 상태와
동일 실행 중 누적된 성과를 기반으로 한 **예측 최적 정책**이다. 실물 마커 코퍼스의
다중 시드 A/B 검증과 계수 튜닝은 아직 남아 있다.

## 2. 적용 범위

이번 선택기의 범위는 **Explore 단계의 disruption/repair 연산**이다.

다음 단계들은 후보군에 섞지 않았다.

- Initial placement
- Compress
- Polish

이 단계들은 호출 시점, 목적, 불변식과 비용 구조가 서로 다르다. 예를 들어 hole fill은
Polish 책임이고, initial placement 알고리즘은 완성된 Explore 충돌 해를 교란하는
연산이 아니다. 이들을 하나의 점수표로 섞으면 phase 책임과 검증 경계가 무너진다.

## 3. 선택 후보

`swarm-core/src/optimizer/adaptive_selector.rs`의 `AdaptiveAlgorithm`이 후보 목록이다.

| 후보 | 적용 의도 |
|---|---|
| `SwapDisrupt` | R&R 비활성/비주기 상황의 기본 교란, R&R no-op fallback |
| `RnrRightmost` | 충돌 hotspot이 우측 packing frontier에 몰린 경우 |
| `RnrLowDensity` | 밀도가 낮고 비어 있는 세로 밴드를 재구성할 여지가 큰 경우 |
| `RnrFragmented` | 고밀도이고 충돌 아이템이 넓게 퍼진 단편화 상황 |
| `RnrGapTarget` | 해 풀이 수렴했지만 구조적 공극·tail 때문에 정체된 경우 |
| `RnrRandom` | 장기 정체와 해 풀 붕괴가 동시에 발생했을 때 다양성 회복 |

R&R cadence가 도래하지 않았거나 안전 조건이 성립하지 않으면
`SwapDisrupt`만 반환한다. R&R cadence가 도래하면 안전한 R&R 후보들을 평가한다.

## 4. 평가 입력

`NestingSituation`은 매 disruption 직전에 현재 rollback된 레이아웃과 collision
tracker에서 계산한다.

- `density`: placed area / strip area
- `collision_hotspot_ratio`: loss 가중 충돌 중심 x / strip width
- `conflict_item_ratio`: movable placed item 중 collision loss가 있는 비율
- `consecutive_failures`: 마지막 feasible 해 이후 연속 separation 실패 수
- `failures_at_width`: 현재 strip width에서의 실패 수
- `pool_loss_spread`: bounded infeasible pool의 최선/최악 loss 상대 spread
- `rnr_available`: PlaceGroup, 배치 수, movable 수, `max_items` 안전 조건
- `gap_target_available`: GapTarget을 시작할 최소 잔여 phase 시간
- `rnr_due`: 설정한 R&R disruption cadence 도래 여부

NaN, infinity와 범위 밖 실수는 평가 함수 내부에서 안전한 값으로 정규화한다.

## 5. 점수와 학습 정책

최종 점수 구조:

```text
score =
    0.45 * normalized_context_prior
  + 1.75 * normalized_learned_reward
  + 0.25 * UCB_exploration_bonus
  - no_op_penalty
```

### 상황 prior

- density, hotspot, conflict ratio, 정체, pool convergence로 후보별 prior를 계산한다.
- 현재 eligible 후보 안에서 0..1로 정규화한다.
- 따라서 hand-tuned prior의 절대 크기가 실행 성과를 영구적으로 압도하지 않는다.

### 실행 성과

- `AlgorithmFeedback.rewards`의 EMA 값을 사용한다.
- selector에서는 `ln(1 + reward) / ln(11)`로 정규화한다.
- Explore 보상:
  - feasible 도달: 10
  - loss 10% 이상 개선: 5
  - loss 소폭 개선: 1
  - 개선 없음: 0
- EMA decay와 최소 weight는 `alns_decay`, `alns_min_weight` 설정을 사용한다.

### 초기 coverage와 exploration

- eligible R&R 후보는 실제 배치 변경 또는 applicability no-op 판정까지 한 번씩 탐색한다.
- no-op은 적용 가능성 probe로 취급해 같은 폭에서 강제 재시도하지 않는다.
- 시도 횟수가 적은 후보에는 UCB exploration bonus를 준다.

### hysteresis

- 정체가 심하지 않고 이전 후보와 새 후보 점수 차이가 0.08 이하이면 이전 R&R을 유지한다.
- 실제 기본 EMA 성공/실패 값인 2.35/0.85를 사용한 테스트에서, 최악의 context prior라도
  실패 후보가 최대 한 번 더 실행된 뒤 성공 후보로 전환되는 것을 확인했다.

### 강제 Random

다음 조건이 동시에 맞으면 deterministic repair에서 벗어나기 위해 Random R&R을 한 번
강제한다.

- `failures_at_width >= 8`
- `consecutive_failures >= 12`
- `pool_loss_spread <= 0.05`
- Random이 아직 실제 성공 시도나 no-op probe를 하지 않음

## 6. 평가에서 실행까지의 흐름

핵심 경로는 `swarm-core/src/optimizer/explore_mods.rs`에 있다.

```text
separation 실패
  -> infeasible pool 저장
  -> 선택한 pool 해로 rollback
  -> NestingSituation 계산
  -> AdaptiveOps::select()
  -> evaluate_algorithm()
  -> 반환된 AdaptiveAlgorithm 실행
  -> 실제 pose 변화 확인
       -> 변경됨: commit_selection + 다음 separation 결과로 보상
       -> no-op R&R: SwapDisrupt fallback
       -> fallback도 no-op/Stop: 성과 귀속 없음
```

실제 변화 판정은 slotmap key나 복사 identity가 아니라 다음 pose multiset으로 비교한다.

```text
(item_id, rotation_bits, x_bits, y_bits)
```

따라서 R&R이 victim을 제거했다가 동일 transform으로 재삽입한 경우에도 성공으로
오판하지 않는다.

## 7. 안전 경계

### PlaceGroup과 freeze

- rigid PlaceGroup이 있으면 현재 individual-polygon R&R은 비활성화된다.
- `is_frozen` 아이템은 R&R victim 후보에서 제외된다.
- movable warm start와 사용자 lock을 같은 의미로 취급하지 않는다.

### R&R 기본 조건

- `rnr.period > 0`
- PlaceGroup 없음
- `rnr.max_items > 0`
- placed item 4개 이상
- movable item 2개 이상

### exact geometry authority

GapTarget raster는 **수리할 밴드를 제안하는 힌트**일 뿐이다. 실제 재삽입 가능 여부는
기존 LBF sampling과 exact CDE가 최종 판단한다. 취소나 실패 시 solution, collision
tracker, RNG, tabu 상태를 transaction rollback한다.

### GapTarget 비용과 Stop

- phase 잔여 시간이 500ms보다 짧으면 GapTarget을 제외한다.
- 전체 구조 리포트 대신 공극 inventory만 계산한다.
- raster work cap은 25,000셀이다.
- raster build와 free-component traversal은 128셀마다 Stop을 확인한다.
- outer traversal과 tail 계산은 256셀마다 Stop을 확인한다.
- 취소 시 `Ok(None)`이 legacy R&R rollback 또는 profiled repair의 mutation 전
  반환으로 이어진다.

## 8. 파일별 변경

### 새 파일

- `swarm-core/src/optimizer/adaptive_selector.rs`
  - `AdaptiveAlgorithm`
  - `NestingSituation`
  - `AlgorithmFeedback`
  - `AlgorithmEvaluation`
  - `evaluate_algorithm()`
  - 상황 정책, coverage, UCB, EMA, hysteresis, no-op penalty 테스트

### 기존 파일

- `swarm-core/src/optimizer/mod.rs`
  - `adaptive` feature일 때 `adaptive_selector` 공개

- `swarm-core/src/optimizer/explore.rs`
  - `disrupt_solution()`과 PlaceGroup helper가 실제 변화 가능 여부를 `bool`로 반환

- `swarm-core/src/optimizer/explore_mods.rs`
  - 상황 지표 계산과 selector 호출
  - enum에서 실제 `RuinStrategy`/Swap으로 dispatch
  - pose 변화 확인과 no-op Swap fallback
  - 실행 후에만 attempts/hysteresis commit
  - no-op penalty와 폭 변경 시 reset
  - width-relief 실패 시 정체 카운터를 보존하고 다음 relief 재시도 지점만 이동

- `swarm-core/src/optimizer/lns.rs`
  - `GapTarget` adaptive dispatch
  - gap-only 25,000셀 진단
  - cancellable gap analysis 사용

- `swarm-core/src/structure/diagnostics.rs`
  - `analyze_layout_gaps()`
  - `analyze_layout_gaps_with_control()`
  - raster/component/tail 내부 cooperative Stop
  - gap-only와 full diagnostics의 `gaps` 동일성 테스트

- `swarm-core/tests/lns_smoke.rs`
  - `GapTarget` invariant 경로와 adaptive Explore end-to-end

- `src/ffi.rs`
  - 기존 dirty config 변경에 새 `ExplorationConfig` 필드가 추가됐지만 complete
    initializer가 누락돼 전체 workspace compile이 막히던 부분을 최소 보완
  - C/managed shrink feedback 기본을 off인 1.0으로 정규화

## 9. 실행한 테스트와 체크

마지막 확인 결과:

```powershell
cargo test -p swarm-core --features "capmods adaptive" adaptive_selector --lib
# 15 passed

cargo test -p swarm-core --features "capmods adaptive" optimizer::explore_mods::tests --lib
# 26 passed

cargo test -p swarm-core --features "capmods adaptive" --test lns_smoke
# 6 passed

cargo test -p swarm-core --features "simd,capmods,adaptive,structure-first" --lib
# 119 passed

cargo test -p swarm --features "simd,capmods,adaptive,structure-first" --lib
# 80 passed

cargo check -p swarm-core --no-default-features
# passed

cargo check --workspace --features "simd,capmods,adaptive,structure-first,jni"
# passed
```

관련 파일의 `git diff --check`도 오류 없이 통과했다. 출력된 것은 Windows checkout의
LF → CRLF 변환 경고뿐이었다.

## 10. Release 빌드

`CMakeLists.txt`를 다시 확인했다. Release의 `cargo-build ALL`은 다음과 동일하다.

```powershell
cargo build --profile release --features "simd,capmods,adaptive,structure-first,jni"
```

실행 결과:

- 상태: 성공
- 프로필: `release`, optimized
- Cargo 보고 시간: 2분 11초
- feature: `simd,capmods,adaptive,structure-first,jni`
- 경고: MSVC linker가 import library/EXP 생성 메시지를 warning channel로 전달한 1건
- Rust compile/link 오류: 없음

생성 산출물:

| 파일 | 크기 | 생성 시각 |
|---|---:|---|
| `target/release/capswarmcore_100.dll` | 3,828,736 bytes | 2026-08-28 16:39:52 KST |
| `target/release/capswarmcore_100.dll.lib` | 10,012 bytes | 2026-08-28 16:39:52 KST |
| `target/release/capswarmcore_100.lib` | 66,872,590 bytes | 2026-08-28 16:39:52 KST |
| `target/release/capswarmcore_100.pdb` | 2,625,536 bytes | 2026-08-28 16:39:52 KST |

SHA-256:

```text
capswarmcore_100.dll
2B7503D64CD2FD900B373CC83741786083D65CA9AFB6A53CD4B7474295B51605

capswarmcore_100.lib
94258EA4D6BF8F4B80A9389E977B4CFA0AFA27B7A0E3E2A3297EE73C46026879

capswarmcore_100.dll.lib
E77CA34BBD01636C5CEA1573861D8BC9CB1A5790E6C905A748AF65A0C71B2F17
```

이 빌드는 Release DLL을 생성한 것이며 제품 설치/복사까지 수행한 것은 아니다.

## 11. 아직 검증하지 않은 것

- 실물 마커 코퍼스 다중 시드 A/B 성능 비교
- C# / WPF build와 managed contract test
- 생성 DLL을 실제 제품 폴더에 복사한 뒤의 ABI/동작 확인
- 실제 제품 프로세스가 새 DLL을 로드했는지 확인

따라서 다음 상태를 구분해야 한다.

- 확인됨: Rust 테스트, feature 조합 check, Release native build
- 미확인: 제품 배포, 제품 프로세스 로딩, 실물 데이터 성능 우위

## 12. 작업 트리 주의사항

이 작업을 시작할 때 이미 대규모 dirty worktree였다.

- WPF/C# UI와 native wrapper 변경
- config, initial placement, structure-first 변경
- FFI와 grouped engine 변경
- 여러 target 디렉터리의 삭제/생성 상태
- 새 initial 관련 파일과 테스트

따라서 다음 명령으로 전체 diff를 무조건 stage하면 안 된다.

```powershell
git add -A
```

특히 `explore.rs`, `explore_mods.rs`, `lns.rs`, `mod.rs`,
`lns_smoke.rs`, `src/ffi.rs`에는 이번 작업 전 변경과 이번 작업이 겹쳐 있다.
커밋이 필요하면 symbol/patch 단위로 검토하고 stage해야 한다.

이번 작업에서 가장 분명하게 소유할 수 있는 경계:

- 새 `adaptive_selector.rs`
- `explore_mods.rs`의 `AdaptiveOps`, 상황 수집, dispatch/fallback
- `diagnostics.rs`의 gap-only cancellable API
- `lns.rs`의 adaptive GapTarget 연결
- selector/dispatcher/cancellation 테스트

## 13. 남은 과제

### 1순위: Release 다중 시드 A/B

동일 입력과 동일 시드 집합에서 baseline과 adaptive를 비교한다.

권장 지표:

- 최종 strip width와 density
- completion count
- wall-clock time
- separation 실패 수
- 알고리즘별 선택/no-op/fallback 횟수
- 알고리즘별 EMA reward
- Stop latency

초기 배치와 최종 Explore/Compress/Polish 품질을 분리해서 보고해야 한다.

### 2순위: telemetry

현재 선택 결과와 scores는 debug log에 있고, weight 요약은 주기적으로 info log에 나온다.
제품 튜닝을 위해 callback/diagnostic JSON에 다음을 노출하는 방안을 검토한다.

- selected algorithm
- contextual score
- learned reward
- attempts/noops
- fallback reason
- outcome score

ABI를 바꾼다면 C header, layout/size handshake, C# marshaling, Java/JNI와 copied DLL까지
별도로 검증해야 한다.

### 3순위: 계수 외부화

현재 context/learned/exploration weight와 임계값은 코드 상수다. 실물 A/B 결과가
누적된 뒤 config로 외부화하는 것이 안전하다.

### 별도 기존 이슈: FFI shrink feedback mode

이번 disruption selector와 독립적인 기존 dirty 변경이다.

`src/ffi.rs`는 `shrink_fail_decay` 값을 전달하지만 `shrink_adapt_mode`를 기본
`DensityLegacy`로 유지한다. `DensityLegacy::uses_feedback()`은 false이므로,
C/managed 호출자가 `0 < fail_decay < 1`을 넣어도 feedback gate가 켜지지 않는다.
이를 고칠 때는 `Feedback`, `DensitySlackFeedback`, 또는 legacy density와 feedback을
결합하는 새 mode 중 어떤 계약을 원하는지 먼저 정해야 한다.

## 14. 다음 세션 시작 절차

1. 경로, branch, HEAD와 dirty 상태를 확인한다.

```powershell
Set-Location C:\Caplanet\AiWorks\Claude\SwarmTemp_Anal
git branch --show-current
git rev-parse --short HEAD
git status --short
```

2. 다음 파일을 먼저 읽는다.

```text
docs/ADAPTIVE_ALGORITHM_SELECTOR_HANDOFF_2026-08-28.md
swarm-core/src/optimizer/adaptive_selector.rs
swarm-core/src/optimizer/explore_mods.rs
```

3. 코드가 더 변경됐다면 최소 검증부터 다시 실행한다.

```powershell
cargo test -p swarm-core --features "capmods adaptive" adaptive_selector --lib
cargo test -p swarm-core --features "capmods adaptive" optimizer::explore_mods::tests --lib
cargo test -p swarm-core --features "capmods adaptive" --test lns_smoke
```

4. 배포나 성능 결론 전에는 full-feature Rust 테스트, Release 재빌드와 실물 corpus
다중 시드 비교를 추가한다.

## 15. 다음 세션에 전달할 짧은 프롬프트

```text
C:\Caplanet\AiWorks\Claude\SwarmTemp_Anal에서 작업을 이어가 주세요.
먼저 docs/ADAPTIVE_ALGORITHM_SELECTOR_HANDOFF_2026-08-28.md를 읽고,
현재 branch/HEAD/dirty worktree가 문서 snapshot과 달라졌는지 확인하세요.
기존 사용자 변경은 되돌리지 말고, adaptive selector의 Release 다중 시드 A/B
검증과 알고리즘별 telemetry 설계를 우선 검토해 주세요.
```
