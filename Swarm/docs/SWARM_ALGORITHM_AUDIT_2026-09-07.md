# Swarm 알고리즘 목록·정확성 점검·수정 보고서

대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`, 2026-09-07 작업 시작 시점의 소스와 이 보고서에 동반한 변경분. Git 저장소가 아니므로 작업 직전 복사본 `target/algorithm-audit-baseline-2026-09-07`과 파일 해시로 변경 범위를 구분했다. 앞선 탐색 최적화·잠금 Place 정책 작업은 이번 비교의 기준에 포함된다.

이번 작업은 Config 값을 조정하거나 알고리즘을 끄는 방식으로 대체하지 않았다. 재현되는 결함을 수정하고, 결과 검증을 보강하고, 불필요한 할당을 줄였다. 아래의 ‘기본’은 Rust 기본값을 뜻하며, 외부 프로그램이 핸들 생성 시 넘기는 실제 설정값과는 구분한다.

## 1. 실행 경로

`swarm-core/src/optimizer/mod.rs`가 일반 실행의 경로를 선택한다. 초기 시드 뒤 Structure가 활성화되면 P0–P5 경로로 진입한다. 일반 Explore의 우선순위는 NFP ordering exploration → Escape → `adaptive` 기능의 적응형 Explore → 기본 Explore다. 이 네 종류가 모두 동시에 실행되는 구조가 아니다.

Compress는 폭을 줄인 뒤 Separator로 충돌을 해소하고, 실패하면 이전 해로 복원한다. Polish는 LP·slide·hole fill을 반복하고 잔여 예산으로 reclaim을 수행한다. V2 입력의 `src/grouped_engine.rs`는 크기별 lane의 시간 배분, 독립 lane 병렬 실행, 전역 좌표 병합과 콜백을 담당한다. lane 내부의 순차 상태 갱신과 독립 후보·worker 병렬 평가는 구분돼 있다.

현재 네이티브 검증 대상 기능은 `simd capmods adaptive structure-first`다. 기능이 빌드됐다는 사실만으로 선택형 알고리즘이 실행되는 것은 아니다. 기본 설정에서는 Structure, Escape, 별도 NFP seed/통합 옵션, refplace가 꺼져 있고, capmods raster LBF와 Polish의 slide/hole fill/LP/reclaim은 켜져 있다.

## 2. 알고리즘 목록과 점검 내용

단위 테스트는 각 구성 요소의 불변식을 확인하며, 통합 테스트는 제한된 입력·시드·시간 예산에서 전체 경로를 확인한다. ‘통과’는 모든 가능한 도형과 조합의 정확성 증명을 뜻하지 않는다. 정확성 게이트 역시 가져온 충돌 도형과 수치 허용오차를 기준으로 한다.

| 구분 | 알고리즘 / 구현 | 사용 조건·역할 | 점검 및 이번 조치 |
|---|---|---|---|
| 입력 기하 | 도형 정규화·단순화·좁은 오목부 처리·간격 확대 | importer, `util/simplify_mm.rs` 계열 | 유효성·단순화 편차·최소 간격 회귀 검사. 설정 유지 |
| 충돌 broad phase | quadtree CDE, surrogate pole | 모든 기본 배치/분리 | quadtree와 실제 Strip 경계의 역할 구분. Strip 밖 잠금 도형을 담는 CDE 확장 정책 유지 |
| 충돌 비용 | 다각형 충돌/외곽 손실, 증분 `CollisionTracker`, GLS 가중치 | Explore·Compress·Polish 공통 | 전체 재계산과 증분 값·삭제/이동·rollback·frozen 면제 일관성 검사 |
| 충돌 가속 | 원형 surrogate overlap proxy, SoA SIMD 평가, 쌍 행렬 | `quantify/overlap_proxy.rs`, `quantify/simd`, `pair_matrix.rs` | scalar/SIMD 수치 일치·쌍 인덱스·트래커 갱신 회귀. 조각 수에 따른 dense 행렬의 제곱 메모리 비용은 유지 |
| Init 순서 | LargestFirst, SmallestFirst, MiddleOutLargeFirst, MiddleOutSmallFirst | `optimizer/lbf.rs` | 순열·수량 유지, 각 순서의 최종 배치 검증 |
| Init 순서 | ConcavityFirst, SizeTimesConcavity | `optimizer/concavity.rs`, LBF | 방향성 오목도·크기 결합, 정렬 동률과 도형별 점수 검사 |
| Init 후보 | 균등 bbox / 집중 샘플 / 좌측 편향 샘플 | `sample/uniform_sampler.rs` 등 | **정확히 맞는 폭/높이를 빈 구간으로 버리던 결함 수정**. 균등·편향·한 축 고정·불가능 크기 검사 |
| 후보 관리 | 중복 제거·상위 후보 유지·후보별 비용 비교 | `sample/best_samples.rs`, `sample/search.rs` | 탐색 후보의 순서와 상한, 충돌 비용 평가 및 개선 후보 선택 회귀 |
| 국소 최적화 | 좌표 하강, 이동/회전 후보 평가 | LBF와 Separator | 평가 예산·취소·고정 변수, 기존 sample/eval 회귀 검사 |
| Init raster | 점유 grid, 회전 mask, raster NFP 후보, 정확 CDE 승인 | `capmods` LBF | raster 후보를 최종 충돌 판정으로 사용하지 않음. raster/mask/hole 배치 통합 검사 |
| Init tail | 접촉 후보·좌하단 sliding / NFP fallback | LBF의 초기배치 보조 경로 | 후보 실패 시 fallback 및 완전 배치 검사 |
| 기본 Explore | 축소 → 분리 → 교란 → feasible pool | `optimizer/explore.rs` | 기능 없는 빌드 경로와 취소·incumbent 유지 검사 |
| 교란 | 큰 조각 쌍 swap / 간헐적 swap hook | Explore와 Separator | 이동 가능한 조각 부족·전부 잠금·그룹 단위 이동, 실패 rollback 검사 |
| 적응형 Explore | 밀도/실패율 기반 shrink, oscillation, width relief | `optimizer/explore_mods.rs`, `adaptive` | 비유한 계산·실패 상한·폭 복구·이전 feasible 해 유지 검사 |
| 연산자 선택 | 문맥 점수 + ALNS 보상 + UCB 탐색 + hysteresis | `adaptive_selector.rs` | 선택 가능성·초기 탐색·no-op 보상·유한 가중치·결정론 검사 |
| Ruin & Recreate | Rightmost, LowDensity, Fragmented, GapTarget, Random | `optimizer/lns.rs` | 제거 대상·tabu·regret 재삽입·병렬 제안/순차 확정·취소 시 RNG 및 배치 복원 검사 |
| 분리 | 독립 worker의 GLS 국소 탐색과 승자 선택 | `optimizer/separator` | worker/트래커 소유권·승자 교체·충돌 행렬 일관성·cancel 검사 |
| Escape 감지 | stagnation window, diversity, bottleneck 점수 | `escape/stagnation.rs`, `bottleneck.rs` | 선택형. 창 크기·진전 판정·빈 입력/잠금 사례 검사 |
| Escape 조작 | guided weights, level 1–3, conflict burst, strip relax | `escape/guided.rs`, `levels.rs`, `conflict_burst.rs` | 충돌 클러스터·밀기·파괴/복구·incumbent 보호 검사 |
| Escape 다양화 | swarm 계층·독립 후보 다양화·feasible/infeasible pool | `escape/swarm.rs`, `pipeline.rs` | 선택형 pipeline smoke, optional 단계 비활성화 조합 검사 |
| Compress | time/iteration shrink decay, 재분리, rollback | `optimizer/compress.rs` | 폭 감소와 해 유효성·잠금 유지·마감 처리 검사 |
| Polish slide | 좌측/아래 scan + 이분 탐색 | `optimizer/polish/slide.rs` | 이동 가능 조각/그룹만 이동, 정확 후보 확인·폭 비악화 검사 |
| Polish hole fill | raster 빈 영역 탐색·샘플링·국소 개선 | `optimizer/polish/holes.rs` | 선택한 pocket의 실제 충돌 검사, 잠금 장애물·후보 한도·stop 검사 |
| Polish LP 모델 | 접촉 법선/경계 제약·신뢰 영역·정규화·backtrack | `optimizer/polish/lp.rs` | 독립 접촉 제약 수집과 상위 K 제한 유지. **입력 경계·반환 primal 재검증 보강** |
| LP simplex | microlp, 50ms 재개 가능한 solve | `lp_solver/microlp_backend.rs` | 독립 2변수 꼭짓점 oracle. **무관한 자유 변수 때문에 유계 문제를 무계로 오판하는 결함 수정** |
| LP/QP conic | Clarabel | `lp_solver/clarabel_backend.rs` | LP oracle 및 해석적 QP 최솟값 비교. **큰 유한 경계를 무한 경계로 삭제하는 결함 수정** |
| 외부 LP/QP | CLP / Xpress 동적 로딩 및 fallback | `lp_solver/clp.rs`, `xpress.rs` | 공통 입력·primal 검증, loader/fallback 회귀. 벤더 실행은 설치·라이선스 가용성 범위에 한정 |
| Reclaim | CompressReentry / Escape 기반 잔여 시간 재사용 | `optimizer/reclaim.rs` | 폭 개선 때만 채택, 원래 마감 공유, 중단 응답 검사 |
| NFP 기하 커널 | 정수 격자, convex decomposition, Minkowski sum, boolean union | `swarm-nfp/src` | 단위·통합 46개. 접촉 경계·오목 도형·hole·clearance·독립 oracle 검사 |
| Complete NFP | primitive union으로 내부 접촉 seam 보존 | NFP 커널과 `nfp_lbf` | 단순 외곽 contour로 잃는 접촉점, 안/밖/경계 분류 검사 |
| NFP cache | prepared library, 회전 캐시, pair 대칭성, 병렬 prewarm | `nfp_lbf/cache.rs` | 캐시 키·역방향 변환·메모리 예산·결정론 검사 |
| NFP 순서 | AreaDescending, MaxDimensionDescending, ConvexHullAreaDescending, ConcavityDescending, Random, CustomGarment, DifficultyTiered | `nfp_lbf/ordering.rs` | 종류별 순열·중복 수요·동률·난이도 tier 검사 |
| NFP 순서 탐색 | swap/insertion/reversal, simulated annealing, prefix 재사용 | `nfp_lbf/exploration.rs`, `search.rs` | 일반 위치 탐색 대신 선택 가능. NFP seed·순서·캐시 재사용 smoke 검사 |
| NFP 회전 | instance_rotations, 연속 회전의 유한 후보화 | `nfp_strategy` 옵션 ① | 선언된 회전과 샘플 회전 구분. 최종 정확 검증 유지 |
| NFP 쌍 조립 | pairwise_clustering | 옵션 ② | 상대 배치·회전·수량과 cluster 전개 검사 |
| NFP 접촉 점수 | contact_maximization | 옵션 ③ | 길이/접촉/좌하단 우선순위 및 동률의 결정적 비교 검사 |
| NFP pocket | pocket_first | 옵션 ④ | 내부 후보와 frontier fallback, 실제 유효 배치 검사 |
| NFP beam | beam_search | 옵션 ⑤ | 부분 해 복제·폭 제한·후보 선택 검사. best-fit 동시 사용 시 곱셈 비용 존재 |
| NFP 동적 선택 | best_fit_selection | 옵션 ⑥ | 남은 종류 lookahead, 수요 유지와 선택 평가 검사 |
| NFP 압축 | sliding_compaction | 옵션 ⑦ | 개별 조각 제거/재삽입, 좌하단 개선 및 충돌 확인 |
| NFP 누적 배치 | accretion | 옵션 ⑧ | 순서를 다중집합으로 사용. 스레드 수·반복 실행별 결과 지문 검사 |
| NFP 꼬리 회수 | tail_reflow | 옵션 ⑨ | 꼬리 제거·앞쪽 빈 공간 재삽입·폭/완전성 검사 |
| NFP Jostling | 배치 좌표를 다음 순서로 되먹이는 반복 재배치 | `nfp_strategy/jostle.rs` | 공개 모듈이지만 기본 pipeline에 연결되지 않은 실험 코드. 구성 요소 단위 검사에 한정, 과거 주석의 실측 수치를 이번 성능 결과로 사용하지 않음 |
| NFP 통합 보조 | nfp_sampler, exact_separation_vector, nfp_hole_filling, nfp_reinsertion | 일반 optimizer의 선택형 옵션 | 제안/이동 비용의 NFP 경로와 최종 CDE 확인, 캐시/stop 테스트 |
| Structure P0/P1 | motif 생성·선택, profile beam, multistart | `structure/motif.rs`, `profile.rs`, `evaluator.rs` | single/pair/rectangle block/repeated band, 수량·영역 경계·동률·construction timeout 검사 |
| Structure P2–P5 | 압축·slack 탐색·GapTarget RNR·rotation snap·Polish | `structure/pipeline.rs` | 단계별 exact incumbent, 잠금 baseline과 fallback, 폭 비악화 검사 |
| 강체 그룹 | 동일 world rigid delta의 원자적 이동 | `optimizer/place_groups.rs` | **충돌 검증과 별도로 원래 상대 pose 보존 검사 추가**. 그룹 전체 회전·이동은 허용 |
| V2 lane scheduler | 독립 크기 영역 병렬 실행·quantum 배분·전역 병합 | `src/grouped_engine.rs` | **잠금/그룹 pose 검사를 후보·콜백·export에 연결**. rollback·RNG·prefix·region gap 검사 |
| Reference placement | boundary/chain 기반 그룹 배치·beam fallback·finish | `refplace` | 기본 비활성. 경계/chain/score/step 구성 요소와 column draft 통합 검사. 전체 품질 우위는 주장하지 않음 |
| Column draft | 열 단위 초기 골격과 후속 배치 | `refplace/columns.rs` | fresh seed 선택형. 정확 배치·수량·FFI 단계 연결 검사 |
| N06/N02C attach | GRASP 후보, Global/Sorted, area/diameter/concavity 순서, 접촉 slide | `initial/attach.rs`, `initial/nfp.rs` | 실험용 API. A–F 후보/회전/비볼록 단계와 S2C 특수 경로 구성 요소 검사 |
| N06 압축·재시작 | G1 LP, G2 반복, G3 rollback, G4 restart | `initial/compact.rs` | 실험용 경로. 모델·rollback 구성 요소 검사, 장시간 비교 하네스는 별도 |
| N06 점유 평가 | scanline fill, bbox empty, left/bottom shadow | `initial/raster.rs` | **메모리 상한·해상도 자동 보정·행별 scratch 재사용**. 독립 cell-center oracle와 극단 입력 검사 |

## 3. 확인한 결함과 수정

### 정확히 맞는 배치를 후보에서 제외

샘플 구간의 `start == end`는 도형이 정확히 들어맞는 한 좌표를 뜻한다. 기존 half-open `Range::is_empty()` 때문에 회전 후보 자체가 제거됐다. 닫힌 실행 가능 구간으로 유효성을 판단하고 길이가 0이면 그 좌표를 반환하도록 고쳤다. 양의 길이를 가진 일반 구간의 난수 생성 방식은 유지한다.

### LP 입력과 반환 해의 불일치

`[+∞,+∞]`, `[-∞,-∞]` 경계가 입력 검사를 통과했다. 변수와 행 모두 이러한 불가능한 경계를 거부한다. Clarabel에서는 `1e21 <= x <= 2e21`가 내부의 `1e20` infinity 기준 때문에 삭제돼 `x=0, Optimal`을 반환하는 실패를 재현했다. 이제 이 백엔드의 지원 범위를 벗어난 큰 유한 경계는 `Failed`로 명시적으로 거부하며 무경계 문제로 바꾸지 않는다.

네 LP 백엔드는 `Optimal` 결과의 벡터 길이, 유한성, 변수 경계, 원본 행 제약과 목적값의 유한성을 다시 확인한다. primal 검증은 `1e-7 × max(1, |bound|)` 수치 허용오차를 사용한다. 이 검사는 원본 LP의 실행 가능성 확인이며 최적성 증명이 아니다. 실제 도형은 후속 backtrack/정확 충돌 검사로 확인한다.

공통 one-shot 호출부는 solve 전뿐 아니라 반환 뒤 취소/마감도 확인한다. 중단 중 반환된 해를 폐기하지만, 외부 native solve 내부를 강제로 선점하는 기능은 아니다.

### microlp 자유 변수 오판

`min x0`, `x0 >= -3`, 자유 변수 `x0, x1`, 목적·제약에 무관한 `x1`을 주면 microlp가 `Unbounded`를 반환하는 문제를 재현했다. 목적 계수 0이며 유효 제약에도 등장하지 않는 자유 변수만 0으로 대표화한다. 해당 변수를 어떤 값으로 두어도 원래 문제의 실행 가능성과 목적값이 같다는 성질을 이용한다. 비용이 있거나 실제 제약에 등장하는 자유 변수는 수정하지 않는다. `x1` 목적 계수를 -1로 바꾼 진짜 무계 문제는 계속 `Unbounded`다. 신뢰 영역이 유한한 일반 Polish 모델은 추가 열 사용 표를 만들지 않는다.

### 잠금·강체 그룹 후보 인증 누락

V2 candidate의 한 조각을 충돌 없는 곳으로 1mm 옮기면 기존 geometry-only 검사와 내보내기를 통과했다. 잠금 Place를 움직이거나 강체 그룹의 한 구성원만 움직여도 같은 문제가 발생했다. lane 초기 잠금 pose를 보관하고, 그룹은 초기 상대 pose와 현재의 공통 rigid delta를 비교한다. 실패한 후보는 기존 incumbent로 복원하고 인증 내보내기를 거부한다.

잠금 pose는 동일 변환을 요구한다. 그룹 상대 pose에는 부동소수 회전/합성 오차를 위한 작은 허용오차를 둔다. 잠금 Place끼리의 겹침과 Strip 이탈 허용은 유지한다. 이동 가능한 Place와 잠금 Place의 충돌, 이동 가능한 Place의 Strip 이탈은 계속 금지다.

### 점유 래스터의 과도한 메모리 할당

기존 `w*h`는 입력 크기와 셀 간격에 따라 overflow 또는 비현실적인 크기의 할당을 유발할 수 있었다. 유효한 유한 크기를 확인하고 곱셈을 검사하며, 최대 1,048,576개 셀 안에 들어가도록 큰 입력에서만 내부 셀 크기를 보정한다. 극단적 종횡비도 함께 처리한다. 새로운 Config는 없다. 호출부의 면적 점수 환산은 실제 `r.cell()`을 사용하므로 보정된 해상도를 반영한다. 이 grid는 근사 점수용이며 최종 기하 승인 역할을 갖지 않는다.

## 4. 최적화와 병렬화 판단

- LP 접촉 행은 대부분 짧다. 8개 이하 열의 중복 확인에서 clone/sort를 없애고 작은 slice 비교를 사용한다. 긴 행은 기존 정렬 방식을 유지한다. 서로 다른 길이/중복 패턴 648개를 집합 oracle와 비교한다.
- N06 raster scanline의 교차점 배열을 행마다 할당하지 않고 호출 안에서 재사용한다. 채우기에는 연속 slice `fill`을 사용한다. 정상 크기 입력의 요청 셀 간격은 유지한다.
- 그룹 상대 pose 확인은 item ID 표 한 번과 구성원 순회로 `O(N+M)`에 수행한다. 그룹당 회전의 sin/cos를 한 번 계산하고 f64 좌표식으로 비교해 구성원별 변환 행렬 합성을 없앴다. 잠금과 그룹이 모두 없으면 새 pose 검사는 즉시 끝난다.
- 기존의 독립 Separator worker, lane, NFP prewarm/후보 평가, LP 접촉 제약 수집 병렬화를 유지한다. 공유 배치나 충돌 트래커의 mutation, RNR 확정, incumbent 교체를 무리하게 병렬화하지 않았다. 이들은 순서 의존성과 rollback 계약을 가진다.
- 전체 실행시간은 후보 난이도와 시간 예산에 크게 영향을 받는다. 이번 수정의 전체 마커 속도 향상률이나 밀도 향상률은 별도 실측 없이 주장하지 않는다.

### 짧은 LP 행 검증의 제한된 성능 측정

작업 직전/수정 후 `LpRow` 소스를 각각 별도 모듈로 추출한 동일한 독립 Rust `-O` 실행 파일로 측정했다. 행 구성은 루프 밖이며 `black_box`를 사용했다. 역순의 서로 다른 열, 200만 회 × 3회, 중앙값이다. 다른 Cargo 빌드가 없는 시점에 실행했다.

| 열 수 | 기존 ms | 수정 ms | 해당 함수 처리 배율 |
|---|---:|---:|---:|
| 2 | 53.657 | 4.369 | 12.28× |
| 4 | 58.287 | 7.356 | 7.92× |
| 8 | 82.296 | 17.365 | 4.74× |
| 16 | 148.055 | 148.595 | 약 1.00× |

이는 할당 제거의 국소 측정이다. 전체 LP 풀이·Explore 실행시간의 배율로 해석하면 안 된다. 재현 소스 `target/algorithm-audit-row-bench.rs`, 원시 결과 `target/algorithm-audit-row-bench.log`를 보관했다.

## 5. 검증 결과

최종 소스로 아래 검증을 완료했다. 동일 테스트의 재실행 및 서로 다른 기능 조합의 중복은 합산 성과로 세지 않는다.

| 검증 | 결과 | 로그 (`target/` 아래) |
|---|---|---|
| Core 전체, `simd capmods adaptive structure-first`, lib + tests | **821 통과 / 0 실패 / 80 ignored** | `algorithm-audit-core-final.log` |
| Rust FFI 전체, 동일 기능, lib + tests | **142 통과 / 0 실패 / 3 ignored** | `algorithm-audit-ffi-final.log` |
| NFP 커널 전체 | **46 통과 / 0 실패** | `algorithm-audit-nfp-baseline.log` |
| 기능 없는 Core, lib + 새 regression + place_groups | **669 통과 / 0 실패 / 13 ignored** | `algorithm-audit-classic.log` |
| 기능 없는 Explore·Polish·Reclaim stop latency | **3 통과**; 이번 실행 92ms / 656ms / 112ms | `algorithm-audit-classic-stop.log` |
| 156조각 `wsjk156.json`, LBF 6개 모드 × seed 0/1/2 | **18/18 조합** 전량 배치, 손실 0, 외곽/쌍/기타 충돌 0, 최종 solution validator 통과 | `algorithm-audit-lbf-modes.log` |
| Release 라이브러리 빌드 | **성공**, opt-level 3 / fat LTO, 설정 수정 없음 | `algorithm-audit-release-build.log` |
| Release FFI 잠금 회귀 | **4 통과**; all locked, public phases, locked group, Structure | `algorithm-audit-release-frozen.log` |
| 실제 DLL 로딩 및 ABI 함수 | **성공**, null 인자 -1 / 정상 인자 0 | `algorithm-audit-release-abi.json` |
| 수정 파일 rustfmt / 패치 역적용 dry-run | **통과** | 변경 패치 및 해시 manifest |

전체 Core 결과에는 새 `algorithm_audit` 테스트 10개가 포함된다. LP 2변수 문제는 30개 × 두 내장 백엔드로 독립 꼭짓점 열거 결과와 비교했고, Clarabel QP는 해석적 해와 비교했다. 별도로 새 raster 단위 검사 2개와 V2 후보 제약 검사 1개를 추가했다. 잠금 예외·정확 검증·그룹 pose·취소·병렬 결정론 등의 기존 회귀도 함께 실행했다.

실패 재현 로그는 `algorithm-audit-regressions-before.log`, `algorithm-audit-constraints-before.log`, `algorithm-audit-free-column-before.log`다. 의심했던 0계수 행의 infeasible 은폐는 테스트상 재현되지 않았으므로 그 동작을 불필요하게 바꾸지 않았다.

CLP와 Xpress의 실제 `availability()` 호출은 모두 DLL 로딩 실패를 반환했다(`algorithm-audit-vendors.log`). 따라서 이 두 벤더의 native 계산은 검증하지 못했고, 소스 변환·입력/출력 방어·로딩 실패·fallback만 확인했다. 장시간 실측용 `#[ignore]` 전체, WPF 화면·앱 배포, wasm 런타임도 실행하지 않았다. 이번에는 네이티브 소스와 생성 DLL까지 검증했다. 휴리스틱의 전역 최적성이나 모든 데이터셋에서의 품질 우위를 보장하지 않는다.

## 6. 산출물과 변경 경계

- 보고서: `docs/SWARM_ALGORITHM_AUDIT_2026-09-07.md` — 활성·선택형·실험용 알고리즘을 **53개 계열**로 분류했다.
- 소스·테스트 변경 **11개 파일**의 패치: `docs/SWARM_ALGORITHM_AUDIT_2026-09-07.patch`.
- 변경 전/후 SHA-256, 보호 파일, 검증 수, DLL 정보: `docs/SWARM_ALGORITHM_AUDIT_HASHES_2026-09-07.json`.
- 빌드 결과: `target/release/capswarmcore_100.dll`. 기존 앱 폴더로 복사하거나 배포하지 않았다.

`Cargo.toml`/`Cargo.lock`, 각 crate의 Cargo 설정, `swarm-core/src/config.rs`, `consts.rs`, NFP/Structure 설정, `src/ffi.rs`, `include/swarm.h`는 작업 전과 바이트 단위로 동일하다. 이번 작업으로 설정 항목·C ABI 필드를 추가하지 않았다.

DLL 크기: **6,351,872 bytes**. SHA-256: `476b2efe42d46b514e8b7af027b5c37afc651af259be3d143802e39c54e190de`. 실제 ABI: **15**, Config **776 bytes**, Structure **116 bytes**, feature flags **0x0F**.
