# 런타임 비활성 알고리즘의 잔여 비용 검토

2026-09-09, `C:\Caplanet\AiWorks\AdaptiveAlgorithm2` 현재 소스 기준.

후속 작업: R&R 선택 통계 및 빈 Polish 제거를 구현하고 검증했다. 아래는 변경 전 검토 기록이며, [구현·검증 결과](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_DISABLED_WORK_IMPLEMENTATION_2026-09-09.md)를 함께 참고한다.

개선할 부분이 있다. 단순한 `if` 제거보다 **실행되지 않을 기능의 통계 순회, 워커 생성, 상태 복원 및 반복 검사**를 없애는 것이 우선이다. 기존 설정값과 기능 의미는 유지하고 내부 실행 경로를 개선하는 제안이다. 컴파일 기능, ABI 변경, 신규 사용자 설정은 검토 대상에서 제외했다.

이번 작업은 소스 검토와 개선안 작성이다. Rust/WPF 동작 코드는 수정하지 않았고 성능 비교도 실행하지 않았다. 아래 우선순위는 확인된 작업량과 호출 위치에 따른 판단이며, 속도 개선율은 측정 전에는 확정할 수 없다. Uttu.Nest와 fable 소스는 읽지 않았다.

Release 최적화가 일부 분기나 사용되지 않는 계산을 이미 제거할 가능성도 있다. 이 문서는 소스 구조에서 확인한 개선 후보이며, 해당 계산이 기계어에서도 모두 남는다고 단정하지 않는다. 실제 절감량은 호출 계측과 Release 비교로 확인해야 한다.

## 1. 실제 작업이 없는 Polish는 바깥에서 준비를 차단 — 우선순위 높음

확인한 경로:

- WPF `Views/WorkspaceViewModel.cs:1666`: Compress 성공 후 `PolishPhase()` 호출. 이 위치에는 Polish 활성 설정에 따른 생략이 없다.
- `src/ffi_step.rs:3016`: Legacy Polish가 `Separator::new`를 먼저 호출한다.
- `swarm-core/src/optimizer/polish/mod.rs:121`: 그 뒤에야 `!config.enabled`로 즉시 반환한다.
- `src/grouped_engine.rs:2426`의 `polish_slice`도 Separator를 먼저 만든다. 상위 그룹 단계에는 lane transaction/scheduling도 있다.

따라서 Polish가 꺼져도 이 API로 들어오면 입력 검사, solution 복제, collision tracker, 워커별 problem/tracker 복제, 스레드풀 생성 등의 비용이 발생한다. tracker 복제에는 기존 공유 버퍼 최적화가 있으므로 모든 복제를 항상 전체 행렬 복사로 계산해서는 안 된다.

Polish 전체는 켜져 있어도 LP/Slide/Hole Fill이 모두 꺼져 있으면 더 많은 작업이 남는다. `polish/mod.rs:137`부터 복원·원본 보정·입력 인증을 수행하고, `:178`의 반복문에서 snapshot과 pose signature를 만든다. 실행할 패스가 없어도 원본 보정, snapshot, compactness 비교, 무변경 라운드 판정이 이어진다. 그룹 제약으로 LP를 실행할 수 없고 나머지 패스도 꺼진 경우도 같다.

개선안:

1. 단계 진입 시 기존 설정과 그룹 제약으로 실제 실행할 패스를 계산한다. `run_lp = polish.enabled && lp.enabled && !grouped`처럼 실제 적용 가능성을 포함한다.
2. 실행할 패스도 없고 Reclaim 등 후속 작업도 없으면 워커 생성과 패스 반복을 생략한다.
3. 입력 인증, phase 상태 전이, 최종 callback, GetSolution의 결과 유지 계약은 남긴다. 정밀 인증을 통째로 없애는 변경은 아니다.
4. **현재 Reclaim은 Polish 활성 플래그와 독립적으로 호출될 수 있다.** `polish.enabled=false`만 보고 외부 함수를 반환하면 기존 Reclaim까지 꺼지는 의미 변경이 생긴다. 유효 작업 판단에 Reclaim을 반드시 포함한다.
5. Structure P5의 회전 snap/후속 분리도 독립 작업이다. Polish 패스가 없다는 이유로 이것까지 생략하면 안 된다.

안전한 첫 단계는 인증된 입력을 그대로 넘기는 경로에서 워커와 빈 라운드만 제거하는 것이다. public core API에 미인증 입력이 들어올 수 있는 경로까지 인증된 것으로 가정하지 않는다.

## 2. R&R을 사용하지 않는 회차의 선택용 통계 생략 — 우선순위 높음

`swarm-core/src/optimizer/explore_mods.rs:953` 부근에서는 교란할 때마다 다음 정보를 먼저 계산한다.

- `movable_conflict_metrics`: 조각 전체를 순회하며 이동 가능 수와 충돌 비율 계산.
- `collision_hotspot_x`: 조각 전체를 순회하며 충돌 중심 계산.
- `infeasible_pool_loss_spread`: 실패 해 풀 순회.
- 밀도·남은 시간 등을 포함한 `NestingSituation` 구성.

그러나 `swarm-core/src/optimizer/adaptive_selector.rs:136` 부근의 선택기는 R&R을 사용할 수 없거나 이번 회차가 R&R 주기가 아니면 바로 `SwapDisrupt`를 반환한다. 앞서 계산한 상세 통계는 선택에 쓰이지 않는다.

개선안:

1. R&R 활성 여부, `max_items`, 최소 조각 수, R&R 주기부터 확인한다.
2. R&R 불가/비주기 회차는 상세 통계 없이 기존 Swap 교란 경로로 진입한다.
3. R&R 선택이 필요한 회차에서만 상세 통계를 계산한다. 이동 가능 수처럼 설정·잠금·그룹에만 의존하는 값은 단계 동안 재사용할 수 있는지 구분한다.
4. R&R 완전 비활성 단계에서는 선택에 쓰이지 않는 R&R 학습·진단 작업도 분리한다. 단순 배열 초기화보다 반복되는 통계/로그 비용부터 측정한다.

기존 Swap의 RNG 소비, 실제 변화 여부 검사, 후속 separation, 정밀 인증은 유지한다. pose signature는 현재 no-op 검출에 쓰이므로 R&R이 꺼졌다는 이유만으로 함께 제거하지 않는다. 폭 복구 등 다른 사용처의 `collision_hotspot_x`도 유지한다.

## 3. Separator의 샘플링 워커를 실제 separation 시점에 생성 — 효과 후보가 크지만 변경 위험도 있음

`swarm-core/src/optimizer/separator.rs:179`의 생성자는 master tracker 외에 워커, 워커별 상태, 스레드풀을 즉시 만든다. NFP sampler가 켜져 있으면 준비 library도 생성한다.

그런데 현재 LP/Slide/Hole Fill 및 원본 보정 내부에는 `separate()` 호출이 없다. 이들은 master layout/tracker를 사용한다. 일반 Polish만 수행하고 Reclaim이나 Structure 회전 분리를 사용하지 않는 경우, 생성한 샘플링 워커가 실제 탐색에 쓰이지 않을 수 있다. `resync()`는 그 사용되지 않는 워커의 상태와 RNG도 갱신한다.

개선안:

- master layout/tracker와 sampler worker 자원을 분리한다.
- `separate()`/워커 실행의 첫 호출에서 `ensure_workers()`로 워커와 pool을 생성한다.
- 이후 호출은 재사용한다. NFP sampler 준비도 같은 실제 소비 시점으로 이동한다.
- Reclaim, Structure snap, repair 이후 separation이 필요한 경우에는 정상적으로 생성한다.

주의: 생성자와 `resync()`가 현재 `sep.rng`를 소비한다. 지연 생성하면서 이 소비까지 무심코 제거하면 다른 알고리즘의 난수열이 달라진다. 기존 worker seed 순서를 보존하거나 난수 스트림을 명시적으로 분리하고 동등성을 검증해야 한다. 지연 생성 시 최신 frozen mask, 그룹, sample config, tracker도 반영해야 한다. 설정의 `n_workers`를 줄이는 대안은 이번 제안에 포함하지 않는다.

## 4. Conflict Burst를 끄면 전용 NFP 준비도 생략 — 우선순위 중간

`swarm-core/src/escape/pipeline.rs:302`는 `exact_separation_vector`만 확인해 `NfpReinsertionCache::new`를 호출한다. 이 생성자는 `PreparedLibrary::build`를 실행한다(`optimizer/lns.rs:721`).

이 함수에서 해당 캐시를 소비하는 곳은 `escape_config.enable_conflict_burst`로 보호된 `conflict_burst_partial_rebuild_with_nfp` 호출이다(`escape/pipeline.rs:529`, `:554`). 따라서 **Conflict Burst OFF + exact_separation_vector ON** 조합에서는 소비하지 않을 library 준비가 남는다.

개선안은 두 설정의 실제 의존 관계를 반영하고, 가능하면 첫 Conflict Burst 실행 직전까지 준비를 미루는 것이다. 둘 다 켜도 정체가 발생하지 않아 Burst를 실행하지 않는 경우까지 준비 비용을 줄일 수 있다. 단, cache 준비 실패도 기억해 매 회차 재시도하지 않아야 한다.

## 5. Raster OFF 상태의 최소 조각 치수 순회 제거 — 작은 비용, 낮은 변경 위험

`swarm-core/src/optimizer/lbf.rs:1350`의 `init_raster`는 모든 조각의 bbox를 순회해 최소 치수를 구한 다음 `RasterNfp::new`를 호출한다. OFF 여부는 그 안의 `sample/raster_nfp.rs:124`에서 검사한다.

`init_raster` 입구에서 OFF를 확인하고 `self.raster=None`으로 정리한 뒤 반환하면 불필요한 전체 조각 순회를 제거할 수 있다. 원래 raster 격자 생성은 OFF일 때 이미 생략된다. 따라서 이 개선을 큰 격자 메모리 절약으로 과장하면 안 된다. 초기화 경로의 작은 O(n) 작업을 없애는 개선이다.

## 6. 비용이 작거나 이미 방어된 곳 — 낮은 우선순위

| 대상 | 현재 확인된 동작 | 판단 |
|---|---|---|
| Separator Swap OFF | `swap_period > 0`이 첫 조건이어서 뒤의 나머지 연산과 Swap 실행은 생략 | 반복 boolean 검사만 제거하려고 separation 루프 전체를 복제할 이점은 미확인 |
| NFP sampler OFF | `bool::then`으로 무거운 library 준비 생략 | 기본 OFF 경로는 이미 보호됨. 소비하지 않는 phase의 ON 경로는 3번 대상 |
| NFP reinsertion OFF/미호출 | 설정은 pending으로 보관하고 실제 R&R에서 cache 준비 | 이미 지연 초기화가 있음. 불가능한 R&R 후보에 대한 준비 순서는 추가 계측 후보 |
| NFP Hole Fill ON, Hole Fill OFF | 빈 `NfpHoleCache`는 만들 수 있으나 `HashMap::new`들과 상태 없는 engine이 중심 | 형상 준비는 실제 호출 때 수행됨. 조건을 정리할 수 있지만 큰 비용으로 보지 않음 |
| Reclaim OFF | 외부 호출 조건과 함수 입구 반환이 존재 | 실행 경로를 모두 확인하지 않고 준비 비용이 크다고 결론 내릴 대상 아님 |
| NFP sampler 없음 | 조각 탐색에서 `Option` 확인 후 기존 sampler 실행 | per-piece 분기 후보이지만 무거운 준비·순회 제거보다 후순위 |

`Vec::new`, 빈 HashMap, bool/Option 확인을 모두 heap 할당이나 큰 계산 비용으로 취급하지 않는다. 불필요한 동적 trait/함수 포인터 호출로 바꾸면 직접 분기보다 느려질 수도 있다.

## 권장 구조와 적용 순서

기존 설정으로 **단계별 유효 작업 목록을 한 번 계산**하고, 그 목록에 필요한 자원만 준비한다. 내부 자료구조로 처리하므로 Config/JSON/C ABI에 신규 옵션을 추가할 필요가 없다. 설정과 잠금/그룹 상태가 바뀔 수 있는 경계에서는 목록을 다시 만든다.

1. R&R 비활성/비주기 회차의 통계 제거.
2. Legacy FFI·Grouped·direct core의 빈 Polish 준비/반복 제거. Reclaim/P5는 독립적으로 유지.
3. Raster OFF 사전 반환, Conflict Burst 전용 NFP 준비 조건과 시점 정리.
4. Separator worker/pool 지연 생성. RNG와 복원 상태를 검증하며 별도 변경으로 진행.
5. 남는 `if`/Option 확인의 루프 외부 이동은 위 변경 후 프로파일에서 유의미할 때만 검토.

정밀 인증, 잠금·그룹·회전 제약, 원본 겹침 검사 및 중지 시 마지막 인증 결과 보존은 최적화 대상 기능의 ON/OFF와 분리한다. 기존 certificate cache가 이미 있으므로 반복 검증이 전부 고비용 전체 기하 검사라고 단정하지 않는다. 검증 호출 수, cache hit, 실제 기하 계산 시간, snapshot/복원 시간을 나누어 측정한다.

## 구현 후 확인할 기준

- Polish OFF/Reclaim OFF: sampler worker·pool 생성 및 polish 패스 호출 0회, 결과·phase 전이·최종 callback 유지.
- Polish ON/LP·Slide·Hole Fill OFF: 빈 라운드 0회. 그룹으로 LP만 적용 불가인 조합도 포함.
- Polish OFF/Reclaim ON: 기존 Reclaim 실행 의미 유지. Structure P5 snap 경로도 별도 확인.
- R&R OFF 또는 비주기 회차: 선택용 상세 통계 순회 0회, 기존 Swap 및 후속 분리 유지.
- Conflict Burst OFF/exact vector ON: Burst 전용 library build 0회. 켜진 경우 fallback/인증 유지.
- Raster OFF: 최소 치수 순회 및 raster build 0회.
- 워커 지연 생성: 순수 Polish 0개, 첫 separation 때 정상 생성, 후속 재사용, 최신 잠금/그룹 반영.
- 동일 seed와 고정 작업 수에서 RNG 소비·연산자 선택·결과를 비교하고, 실제 고정 시간 실행에서는 사용 길이/수율과 처리 시간을 함께 비교한다. 시간 절약으로 탐색 횟수가 달라지므로 고정 시간 결과의 비트 일치를 요구하지 않는다.
- 확인 대상은 이 저장소의 core, Legacy FFI, Grouped 및 WPF 호출 경로다. 이번 검토만으로 실제 실행 중인 설정 조합이나 절감 시간을 추정하지 않는다.
