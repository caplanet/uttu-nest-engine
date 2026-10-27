**Swarm Rust 탐색·압축·Polish 병렬화 및 알고리즘 검토 — 2026-09-07**

분석 대상은 `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`의 현재 소스다. 이 디렉터리에는 Git 저장소가 없어 커밋으로 식별할 수 없다. 함께 저장한 `SWARM_REVIEW_SOURCE_HASHES_2026-09-07.json`의 SHA-256으로 분석 대상 파일을 식별한다. Rust 솔버 소스는 수정하지 않았으며, 이 보고서와 독립 재현 프로그램만 추가했다.

**판단 요약**

탐색과 압축은 이미 Separator 작업자 단위로 병렬화되어 있다. 추가 성능 개선은 LP 접촉 제약 생성, R&R의 초기 regret 평가, 작업자 상태 복사 감소부터 검토하는 것이 합리적이다. 여러 탐색 상태를 유지하는 island 방식은 품질 향상 후보지만 현재 탐색 알고리즘을 바꾸므로 별도의 비교가 필요하다. 동일 레이아웃의 아이템 이동에 바로 `par_iter_mut()`를 적용하는 방식은 충돌 검증의 전제를 깨뜨린다.

확장에 앞서 고칠 문제도 확인했다. 순수 NFP 순서 탐색은 고정 아이템을 보존하지 않는 경로가 있으며 작은 입력으로 재현했다. 중단 지연 회귀 테스트는 reclaim 경로에서 두 번 실패했다. 실행 중 축소율 설정은 프로세스 전역이어서 여러 솔버의 독립 설정을 보장하지 못한다.

이 보고서에서 **확인**은 현재 소스 또는 실행 결과로 확인한 사실, **추론**은 코드 구조로 예상되는 성능·품질 영향, **제안**은 아직 구현·실측하지 않은 변경을 뜻한다. 속도 향상 배수나 생산 데이터의 밀도 향상률은 측정하지 않았다.

**1. 실제 실행 경로와 이미 존재하는 병렬화**

| 경로 | 현재 동작 | 분석상 주의점 |
|---|---|---|
| 일반 Explore | 폭 축소 → separation → 실패 해 풀 → 교란 | `explore.rs`; 실패 해 풀은 이미 32개 상한 |
| Adaptive Explore | 위 구조 + 상황별 R&R/Swap 선택, EMA 보상, 폭 진동/완화 | `explore_mods.rs`; `adaptive` feature 필요 |
| Escape Explore | 정체 감지와 단계별 탈출 연산 + separation | `escape/pipeline.rs`; 런타임 옵션이 Adaptive보다 우선 |
| 순수 NFP Explore | 초기 순서 평가 → swap/move/block 순서 이웃 → SA 수용 → 주기적 LP | 일반 separation 탐색을 대체; NFP 캐시 prewarm은 병렬이지만 순서 탐색의 주 반복은 직렬 |
| Compress | 최선 해 복원 → 무작위 split에서 축소 → separation → 성공 채택 | Explore와 동일한 병렬 Separator 사용 |
| Polish | LP → slide → hole fill → 검증 → 다음 라운드 | 해당 패스 구현은 직렬; 잔여 시간은 옵션에 따라 reclaim 재진입에 사용 |

직접 API와 step API에서 일반 탐색 선택 우선순위는 `ordering_exploration → escape_enabled → adaptive feature → legacy`다. `structure-first`는 별도의 P0–P5 경로다. CMake는 `simd capmods adaptive structure-first` feature를 지정하지만 Cargo에는 default feature가 없으므로, 단순 `cargo build`와 CMake 빌드는 알고리즘이 다를 수 있다. 이 검토의 테스트는 CMake에 명시된 네 feature를 사용했다. 현재 배포 DLL의 빌드 옵션은 검증하지 않았다.

근거: [진입점과 phase 전환](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/mod.rs:254), [step API 분기](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:2127), [CMake feature](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/CMakeLists.txt:135), [NFP 순서 탐색](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/exploration.rs:312).

`Separator::move_items_multi`는 매 반복마다 다음을 수행한다.

1. master 레이아웃 스냅샷을 저장한다.
2. 모든 작업자에 동일 레이아웃과 CollisionTracker를 복사한다.
3. 작업자마다 독립 RNG로 충돌 아이템 순서를 섞고, 자기 레이아웃에서 순차 이동한다.
4. 전체 작업자가 끝나면 가중 충돌 loss가 가장 낮은 작업자 한 개의 결과를 master에 복원한다.

따라서 현재 구조는 작업자별 독립 후보를 만드는 병렬 탐색이다. 아이템들을 분할해 한 레이아웃을 동시에 갱신하는 구조가 아니다. 각 작업자의 RNG와 NFP 캐시는 독립적이며, 동일 레이아웃을 수정하는 mutex 경합은 없다. 반면 매 반복 barrier와 가장 느린 작업자 대기, 전체 상태 복사, 한 해로 재수렴하는 비용이 있다. 기본 Explore/Compress 작업자는 각각 3개다.

근거: [작업자와 전용 thread pool 생성](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:87), [병렬 실행과 최선 선택](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:386), [작업자 순차 이동](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/worker.rs:175).

**2. 병렬화할 수 있는 부분과 적용 조건**

| 대상 | 가능성·우선순위 | 안전한 분할 단위 | 필요한 조건 및 한계 |
|---|---|---|---|
| LP 접촉 제약 생성 | 높음, 우선 검토 | x-sweep으로 고른 도형 쌍 `(i,j)` | 쌍별 local Vec 생성 후 안정된 순서로 병합; 모델 구성·solve·적용은 분리 |
| R&R 초기 regret 평가 | 높음, 우선 검토 | victim 제거 직후의 고정 레이아웃에 대한 victim별 최선/차선 평가 | evaluator·RNG·NFP 임시 상태를 태스크별 분리; 실제 재삽입은 순차 수행하며 재검증 |
| 한 아이템의 후보/좌표 하강 시작점 | 조건부 | 고정 장애물 스냅샷에서 독립 seed별 정제 | 공유 mutable collector 금지; 현재 상한 기반 조기 탈락이 약해져 평가량이 늘 수 있음 |
| Explore 독립 island | 높음, 알고리즘 변경 | 해·GLS·RNG·tabu·ALNS 상태를 가진 독립 탐색 | 매 반복 동기화 대신 주기적 elite 교환; 다양성 및 동일 CPU 예산 비교 필요 |
| Compress 다중 시도 | 높음, 알고리즘 변경 | 동일 incumbent에서 다른 split 위치/축소율로 시작하는 독립 시도 | 시도별 Separator; 완료 해를 검증 후 폭 기준으로 선택; stale 해 채택 방지 |
| Hole fill 후보 평가 | 조건부 | 고정 레이아웃에 대한 아이템×포켓 또는 아이템×회전 평가 | 후보 두 개가 같은 빈 공간을 차지할 수 있으므로 채택 직전 최신 상태에서 재검증 |
| 점유 격자 생성 | 조건부 | 격자 row/tile 소유권 분할 또는 도형별 임시 bitmap의 OR 병합 | 현재 도형별 `occupied` 갱신을 그대로 병렬화하면 쓰기 충돌; bitmap 복사량을 측정 |
| Slide | 제한적 | 독립 이동 영역이 검증된 아이템 묶음 또는 독립 순서별 전체 패스 | 앞 조각이 비운 자리를 뒤 조각이 사용하는 의존성 존재; 단순 item 병렬화 부적합 |
| LP solve | 백엔드 의존 | 서로 다른 독립 LP 모델 | 호출부에 Rayon을 씌운다고 한 simplex 풀이가 병렬화되지는 않음 |
| GLS weight 갱신 | 조건부, 후순위 | max loss 계산 뒤 각 CTEntry의 독립 갱신 | 작은 N에서는 task 비용이 큼; loss 합산은 부동소수 덧셈 순서 변화에 주의 |
| 순수 NFP 순서 탐색 | 높음, 알고리즘 변경 | 독립 SA chain 또는 초기 ordering 묶음 | 단일 chain의 수용/온도/prefix 상태는 직렬; frozen 계약을 먼저 수정 |

LP는 이미 x축 sweep과 bbox 가지치기를 한다. 병렬화의 정확한 대상은 그 이후 `collect_pair_rows(i,j)`와 `(j,i)`다. R&R은 코드가 명시적으로 “victim 제거 직후”의 레이아웃 하나에서 regret를 평가하므로 병렬 경계가 비교적 분명하다.

근거: [LP 쌍 생성](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:520), [정점×모서리 평가](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:718), [regret 평가](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/lns.rs:333), [slide 의존 순서](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/slide.rs:69), [후보 evaluator의 mutable buffer](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/sep_evaluator.rs:13).

병렬 구현에서는 동일 snapshot에서 평가가 끝났더라도 후보를 동시에 채택하면 안 된다. A/B가 각각 원래 장애물을 피했어도 둘의 새 위치가 겹칠 수 있다. 단일 coordinator가 최신 배치로 재검증 후 commit하거나, 새 위치 간 상호 비충돌까지 검증한 batch만 적용해야 한다. PlaceGroup은 그룹 전체를 하나의 이동 단위로 취급한다. 공유 데이터의 `Send`/`Sync`와 FFI listener 제약은 컴파일 및 호출 경로 검토로 확인해야 하며 `unsafe impl`로 우회할 사안이 아니다.

프로세스 전체 CPU 예산도 필요하다. 솔버 수 S, island 수 I, 각 Separator 작업자 수 W를 동시에 늘리면 대략 S×I×W개의 실행 작업이 경쟁한다. 현재는 Separator마다 전용 pool을 만든다. 외부 병렬 솔버와 NFP prewarm의 Rayon 사용까지 고려해 공통 자원 예산을 정하고, 중첩 수준 중 하나를 주 병렬화 축으로 선택하는 편이 낫다.

**3. 단계별 알고리즘 최적화**

**탐색·압축 공통 — 복사와 loss 집계를 먼저 계측한다.**

CollisionTracker는 `N(N+1)/2`개의 CTEntry를 가진 삼각 행렬이다. CTEntry는 두 f32인 weight/loss이므로 행렬 데이터만 약 `4N(N+1)` bytes다. N=156이면 약 96 KiB, N=1,000이면 약 3.82 MiB, N=5,000이면 약 95.4 MiB이며 작업자 수만큼 복제된다. layout/CDE와 master·rollback snapshot은 이 계산에서 제외했다. 대형 입력에서 O(WN²) 복사·메모리 비용이 커지는 구조는 확인되지만, 현재 입력에서 가장 큰 병목인지는 프로파일링이 필요하다.

`copy_from`은 이미 버퍼를 재사용한다. 따라서 “clone을 copy_from으로 바꾸기”는 새로운 개선안이 아니다. 남은 후보는 단일 작업자 fast path, 변하지 않는 freeze/group 메타데이터의 버전별 동기화, master-작업자 사이 변경분 동기화, phase 사이 준비 geometry 재사용이다. 특히 `change_strip_width`는 이동과 resync를 수행하고 Compress는 곧이어 rollback을 하므로, snapshot을 직접 설치하는 별도 경로가 불필요한 중간 상태와 재구축을 줄일 수 있다. 폭·CDE·키·GLS 복원이 함께 맞아야 한다.

`get_total_loss`와 `get_total_weighted_loss`는 호출마다 행렬을 순회한다. 작업자별 total을 한 번 계산해 선택에 재사용하고, item move 시 total을 delta 갱신하는 방식도 후보이다. delta 누적은 부동소수 오차 때문에 기존 합산과 달라질 수 있으므로 주기적 전수 재계산과 차이 검증이 필요하다. 희소 행렬 전환은 loss=0인 쌍의 과거 GLS weight 감쇠도 보존해야 한다.

근거: [PairMatrix](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/pair_matrix.rs:13), [total 집계](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:269), [GLS 갱신](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:192), [폭 변경과 resync](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:495), [Compress 복원](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/compress.rs:88).

**Explore — 해의 다양성과 시간당 성과를 보존한다.**

현재 모든 작업자가 매 반복 동일 master에서 출발하고 하나만 남는다. 실패 해 풀도 loss 순으로 정렬하고 상위 32개를 유지하므로 서로 다른 배치인지 보장하지 않는다. 구조적으로 유사한 저손실 해에 집중할 가능성이 있다. 상위 일부는 loss로, 나머지는 정규화된 배치 위치·접촉 관계·포켓 구조의 차이로 유지하는 방법과, 작업자 island를 여러 sweep 동안 유지하는 방법을 비교할 수 있다.

Adaptive 보상은 feasible/큰 loss 감소/작은 감소/실패의 등급과 EMA다. 연산 시간은 보상에 포함되지 않는다. 비싼 R&R이 같은 점수를 얻어 더 많은 시간을 소비할 수 있으므로, 기존 보상을 유지한 대조군과 `개선량/경과시간`, 폭 개선 확률, 실패 예산을 반영한 정책을 비교할 가치가 있다. “loss 감소”와 “완전한 배치의 폭 감소”는 별도 지표로 남겨야 한다.

근거: [풀 유지](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore_mods.rs:764), [보상 갱신](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore_mods.rs:682).

**Compress — 무작위 split과 실패 예산을 개선한다.**

현재 split은 strip 전체에서 균일 난수다. 작은 빈틈이 존재하는 band, 우측 끝을 결정하는 패널과 그 충돌 이웃, 밀도가 낮은 seam 등을 후보로 삼고 무작위 후보를 일부 유지하는 포트폴리오가 가능하다. 폭별 성공률과 수리 시간을 기록해 shrink 크기 및 각 시도의 시간을 조절하면, 한 번의 어려운 separation이 남은 압축 시간을 모두 쓰는 문제를 줄일 수 있다.

FailureBased 축소율은 이미 성공 시 한 단계 회복한다. 이를 새 기능처럼 중복 구현할 필요는 없다. 또한 heuristic 수리 실패는 해당 폭의 수학적 불가능성을 증명하지 않으므로, 실패 폭을 엄격한 이분탐색 하한으로 사용하면 안 된다. 고정 아이템이 더 이상의 축소를 막으면 동일 시도 반복 대신 조기 종료할 수 있다.

근거: [축소율·성공 회복](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/compress.rs:30), [split과 frozen 하한](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/compress.rs:92).

**Polish — LP 기하 비용과 실질적인 폭 기여도를 줄인다.**

LP의 근접 쌍마다 정점×모서리 조합을 검사하고 모든 후보를 모아 정렬한 뒤 방향 중복 제거와 개수 제한을 한다. 정점이 많은 의류 패턴에서는 이 비용이 커질 수 있다. edge BVH/격자 색인, 방향별 유효 제약의 조기 축약, 쌍별 임시 버퍼 재사용을 검토하되 현재 정렬·동률·법선 처리 의미를 유지해야 한다. `max_constraints` 검사는 모든 rows 생성 뒤이므로 상한을 넘길 것이 확정된 시점에 중단해 모델을 축소하도록 반환할 수 있다.

Hole fill은 우측 band의 작은 아이템부터 제한 개수만 선택한다. 작은 조각을 옮겨도 큰 패널이 계속 우측 끝을 결정하면 strip 폭은 줄지 않는다. 단순 작은 면적 우선에 더해 “이 조각/연결된 몇 조각을 옮겼을 때 실제 우측 frontier가 줄어드는가”를 평가할 수 있다. 래스터는 셀보다 작은 포켓을 놓칠 수 있고, exact NFP도 후보 cap 및 연속 회전 8방향 이산화가 있어 배치 탐색 전체가 완전하지는 않다.

Polish는 라운드에서 폭이 줄 때만 best solution을 갱신한다. 폭은 같아도 다음 압축을 쉽게 만드는 재배열은 최종 rollback에서 사라질 수 있다. 별도의 동일 폭 다양성 후보를 reclaim에 넘기거나 보조 목적을 두는 방식이 가능하다. 같은 폭 후보를 채택하더라도 frozen·회전·수량·충돌 조건은 그대로 검증해야 한다.

근거: [LP 제약 상한](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:361), [접촉 후보 정렬](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:730), [Hole 후보 선택](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/holes.rs:844), [회전 후보](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/holes.rs:492), [Polish best 갱신](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/mod.rs:206).

**4. 취약점과 정확성·운영 위험**

| 우선순위 | 항목 | 확인 수준 | 영향 |
|---|---|---|---|
| P1 | 순수 NFP 순서 탐색의 frozen 계약 누락 | 소스 및 독립 입력 재현 | 고정 위치 변경 또는 최종 폭 동기화 중 panic |
| P1 | reclaim 중단 지연 | 기존 회귀 테스트 2회 실패 | 정지 요청 후 3–4초 이상 계속 실행 |
| P1 | 프로세스 전역 shrink override | 소스 확인 | 동시 솔버 간 파라미터 간섭, A/B 독립성 훼손 |
| P2 | 작업자 내부 비중단 구간 | 소스 확인 | 고복잡도 도형·큰 샘플 수에서 stop 응답 저하 가능 |
| P2 | proxy loss=0과 전체 계약 검증의 차이 | 소스 확인 | low-level phase 반환/새 병렬 reducer에서 불완전 인증 위험 |
| P2 | 그룹 모드에서 R&R·Polish 생략 | 의도된 동작 확인 | 고정 그룹이 아닌 rigid group에서도 해당 개선 단계의 효과 없음 |
| P2 | Polish NoChanges를 결정론적 고정점으로 해석 | 근거 불충분 확인 | 시간 중단·확률적 정제·동일 폭 상태 변경을 정체로 오분류 가능 |
| P2 | thread pool·cache·dense tracker 자원 증폭 | 구조 확인, 성능 영향 추론 | 대형 입력 및 복수 솔버에서 메모리·CPU 경쟁 |

**P1 — frozen 누락 재현.**

step API는 `sep.freeze_items` 후 `ordering_exploration` 분기로 진입할 수 있다. 하지만 NFP ordering은 그 마스크를 읽지 않고 전체 아이템 순서를 새로 배치한다. 최종 인증 함수는 형상 충돌·경계·회전·수량을 확인하지만 원래 frozen transform과 비교하지 않는다.

4×4 사각형 두 개, strip 높이 12/폭 30, item 0을 `(20,5)`에 고정한 독립 재현 프로그램을 실행했다. 기본 quadtree depth=4에서는 순서 탐색 종료 후 기존 Separator의 폭을 축소하는 과정에서 `jagua-rs 0.8.1 / qt_hazard.rs:113` assertion으로 실패했다. backtrace는 `nfp_ordering_exploration_phase → Separator::change_strip_width → SPProblem::change_strip_width → Layout::swap_container → QTHazard::constrict`였다.

같은 입력에서 quadtree depth만 0으로 설정해 공간 인덱스 assertion을 분리하자 다음이 관측됐다.

```text
locked_before=(20.0, 5.0)
locked_after=(2.001, 2.001)
width=4.00104
exact=true
frozen_preserved=false
```

즉 전체 형상 검증을 통과해도 고정 계약은 깨질 수 있다. 이는 NFP 순서 탐색 옵션의 문제이며 일반 Separator의 모든 이동이 frozen을 무시한다는 뜻은 아니다. 기본 depth panic은 이 재현 입력에서 확인했으며 Release 배포 DLL에서의 재현 여부는 확인하지 않았다.

단기 방안은 frozen 또는 미지원 PlaceGroup이 있으면 이 탐색 경로를 명시적으로 배제하는 것이다. 근본 방안은 frozen을 고정 장애물로 초기화한 뒤 movable만 순서 탐색하고, 최종 결과에 freeze/group transform 보존 인증을 추가하는 것이다. 최선 해 설치 시 기존 배치를 먼저 과도하게 좁히는 대신 target snapshot을 원자적으로 설치하는 경로도 필요하다. depth=0은 원인 분리용이며 운영 수정안으로 권하지 않는다.

근거: [freeze 적용](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:2317), [NFP 전체 재배치](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/exploration.rs:217), [최종 폭 동기화](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/exploration.rs:456), [검증 범위](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/structure/pipeline.rs:1223), [독립 재현 소스](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/swarm_review_frozen_probe.rs).

**P1/P2 — 중단 경로.**

작업자는 아이템 사이에 `term.kill()`을 확인하지만 `search_placement_maybe_nfp`가 호출하는 일반 `search_placement`는 `never_stop`을 전달한다. `refine_coord_desc` 내부에도 중단 인자가 없고 1,000회 평가 제한은 debug assertion일 뿐 Release 상한이 아니다. `CollisionProbe::max_slide` 역시 개별 scan/bisection 중 중단을 확인하지 않는다. LP는 solver 호출 전후만 외부 term을 확인하고, NFP `feasible_region` 한 번의 기하 연산도 외부 stop을 직접 받지 않는다.

실행 결과 Explore stop은 184ms로 통과했으나, reclaim stop은 4,249ms로 실패했고 해당 테스트만 다시 실행해도 3,240ms로 실패했다. 테스트 요구는 1,500ms 미만이다. Polish 자체는 4,596ms로 현재 테스트 기준을 통과했다. 테스트 메시지의 “바깥 플래그 미전달”을 원인으로 단정하지 않는다. 실제 `SliceTerminator`는 외부 `stop_requested()`를 확인하므로, 지연 구간을 계측해 내부 후보 평가·복원·solver 호출 중 어느 부분인지 추가 확인해야 한다.

개선은 control 인자를 작업자→NFP/일반 search→좌표 하강까지 전달하고, 큰 기하 루프를 chunk 단위로 중단 가능하게 만드는 것이다. LP는 작은 solve quantum, backend interruption 또는 필요 시 프로세스 격리를 별도 검토한다. 중단 시 직전 feasible incumbent 및 트래커 일관성을 유지해야 한다.

근거: [작업자 호출](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/worker.rs:52), [never_stop](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/search.rs:34), [좌표 하강](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/coord_descent.rs:89), [LP solve 경계](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:417), [reclaim 테스트 기준](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/tests/stop_latency.rs:222), [SliceTerminator](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/reclaim.rs:108).

**P1 — 전역 설정의 솔버 간 간섭.**

`config::runtime::SHRINK_STEP`은 프로세스 전역 `AtomicU32`다. setter에 handle이 없으며 모든 legacy/adaptive 탐색이 이를 읽는다. 메모리 data race는 방지하지만 솔버별 독립성은 보장하지 않는다. A의 실행 중 설정 변경이 B의 다음 shrink에도 적용된다. 여러 island를 도입하기 전 handle별 `RunControl` 또는 실행별 snapshot/공유 제어 객체로 범위를 좁혀야 한다.

근거: [전역 atomic](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/config.rs:1065), [handle 없는 C API](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:3629).

**P2 — 기존 안전장치와 남은 경계.**

현재 직접 optimizer는 phase 경계마다 `retain_exact_no_worse`를 수행하며, 전체 validator는 공개 transform으로 geometry를 다시 구성하고 수량·허용 회전·경계·도형 쌍을 확인한다. 이 방어는 유지해야 한다. 반면 CollisionTracker는 해결할 수 없는 frozen–frozen/frozen–경계 충돌을 loss에서 제외하므로, `loss == 0`만으로 전체 계약이 충족됐다고 할 수 없다. 또한 여기서 exact는 importer가 구성한 `shape_cd`에 대한 의미다. 전처리 단순화·offset·원본 외곽선 공차까지 자동으로 인증한다는 뜻은 아니다.

PlaceGroup 상태에서는 Adaptive R&R와 현재 Polish가 명시적으로 생략된다. 이것은 안전한 fallback이지만, 그룹 기능 사용 시 해당 단계의 품질 개선을 기대할 수 없다는 기능 한계다. 그룹 전체의 충돌 평가와 공통 이동 변수로 구현을 확장해야 한다.

Polish의 “모든 패스가 결정론적이므로 NoChanges는 진짜 고정점”이라는 주석은 코드와 맞지 않는다. raster hole fill의 좌표 하강은 RNG로 축을 선택하고, 시간 제한도 pass 수행량에 영향을 준다. LP는 폭 개선 없이 위치만 바뀐 경우 `NoImprovement`를 반환할 수 있다. 따라서 변경 수 0을 배치 상태가 완전히 동일하고 재시도 가치도 없다는 증거로 삼으면 안 된다. timeout/stop, 실제 pose 변화, 폭 개선을 구분한 종료 사유가 필요하다.

근거: [최종 인증](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/mod.rs:411), [frozen loss 제외](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:96), [그룹 Polish 생략](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/mod.rs:142), [NoChanges](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/mod.rs:222), [RNG 정제](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/holes.rs:572), [이동했지만 폭 변화 없음](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:479).

NFP 캐시도 이미 병렬 prewarm 및 Arc 기반 값 공유를 지원한다. 다만 `fork()`는 모든 것을 Arc 하나로 공유하는 함수가 아니다. `PreparedLibrary`와 HashMap 구조를 clone하며, Separator의 NFP cache 생성은 pair prewarm 없이 작업자별 fork를 만든다. 콜드 miss가 겹치면 동일 pair를 작업자마다 계산할 수 있다. 준비 library의 Arc화, 제한 시간 내 수요 높은 pair만 prewarm, 불변 공통 cache와 작업자별 miss 영역 분리를 검토할 수 있다. hot path 전체를 단일 mutex cache로 바꾸면 병렬 효과가 줄 수 있다.

근거: [cache fork](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/cache.rs:1238), [기존 병렬 prewarm](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/cache.rs:1267), [Separator cache 준비](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:72).

**5. 권장 실행 순서와 성능 평가 기준**

1. **계약·중단 수정:** frozen NFP 분기, 전역 runtime 범위, reclaim 중단 실패를 먼저 수정한다. quantity/회전/frozen/group/충돌을 함께 검사하는 단일 최종 인증 규약을 정한다.
2. **계측·복사 최적화:** save/restore, CT copy, shape/CDE 갱신, 샘플링, LP rows/solve/backtrack, NFP miss, callback 직렬화 시간을 분리한다. 불필요한 재구축과 total 재계산을 줄인다.
3. **계산 단위 병렬화:** LP pair rows와 R&R 초기 regret 평가부터 적용한다. 태스크마다 고유 인덱스·RNG를 부여하고 결과를 안정된 순서로 병합한다.
4. **알고리즘 병렬화:** 독립 island, Compress 다중 split, NFP 다중 SA를 동일 CPU 예산에서 비교한다. 매 반복 전역 최선 복제 대신 제한적 elite 교환을 사용한다.
5. **Polish 정책 개선:** 실제 우측 frontier 기여도, 동일 폭 구조 개선 후보, LP/slide/hole별 시간당 개선량에 따라 예산을 조절한다.

성능 비교는 고정된 initial layout에서 Explore/Compress/Polish를 따로 측정하고 전체 파이프라인도 측정한다. 소형, 정점 많은 의류 패턴, 반복 형상, 큰 N, frozen, PlaceGroup 입력을 구분한다. 1/2/4/8 작업자 또는 동일 CPU 총량의 island 조합을 비교하고 여러 RNG seed의 중앙값과 분산을 남긴다.

필수 지표는 완전 배치 여부, 원본 계약을 고려한 충돌·간격, 회전·frozen·group 보존, 최종 폭/밀도, time-to-best, 평가 수/초, CPU 시간, peak 메모리, stop latency P50/P95/max다. 시간 제한 실험의 재현성과 연산량 고정 실험의 결정론은 별도로 다룬다. worker 수를 바꾸면 현재 알고리즘의 후보 수 자체가 달라지므로 비트 동일성을 무조건 요구하는 대신, 동일 후보 집합의 병렬 evaluator/reducer에 대해서만 안정된 결과를 요구한다.

**6. 이번에 수행한 검증**

```powershell
cargo test --offline -p swarm-core --features 'simd capmods adaptive structure-first' --lib optimizer:: -- --test-threads=2
cargo test --offline -p swarm-core --features 'simd capmods adaptive structure-first' --test postprocess_phases --test stop_latency -- --test-threads=1 --nocapture
cargo test --offline -p swarm-core --features 'simd capmods adaptive structure-first' --test stop_latency reclaim_stops_promptly -- --test-threads=1 --nocapture
```

| 검증 | 결과 |
|---|---|
| optimizer 단위 테스트 | 99 통과, 0 실패, 2 ignored; 실행 5.18초 |
| postprocess_phases | 7/7 통과; 실행 0.27초 |
| stop_latency | Explore 184ms 통과, Polish 4,596ms 통과, reclaim 4,249ms 실패 |
| reclaim 단독 재실행 | 3,240ms로 다시 실패; 기준 1,500ms 미만 |
| frozen 독립 재현, 기본 depth=4 | quadtree assertion 실패 |
| frozen 독립 재현, depth=0 | exact=true이면서 frozen 좌표 변경 확인 |

테스트는 저장소의 optimized test profile + debug assertions로 실행했다. Release 배포 DLL, C#/WPF, FFI end-to-end, 전체 workspace 회귀, 실제 마커의 병렬 속도·밀도 비교는 수행하지 않았다. 통과한 단위 테스트가 위 두 재현 문제를 부정하지 않는다. 실패한 stop 테스트를 포함하므로 “전체 검증 통과” 상태가 아니다.

독립 재현은 [실행 스크립트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/run_swarm_review_probe.ps1)로 다시 수행할 수 있다. 먼저 위 Cargo 테스트 명령으로 동일 feature의 의존 rlib를 생성한 뒤 workspace 루트에서 실행한다. probe는 frozen 미보존을 확인하면 depth-zero 모드에서 정상 종료하도록 작성된 분석용 프로그램이다.
