# 잠금 Place 예외 정책 구현 (2026-09-07)

작업 경로: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`.
이 문서는 이전 Explore 최적화 작업 이후의 잠금 정책 변경만 설명한다.

## 적용 계약

`Hint=true`로 전달된 `is_freeze=1` Place는 원래 위치와 회전을 유지한다.

| 검사 대상 | 처리 |
| --- | --- |
| 잠금 Place끼리의 겹침 | 허용, 충돌 비용과 상태에서 제외 |
| 잠금 Place의 strip 좌/우/상/하 이탈 | 허용, 충돌 비용과 상태에서 제외 |
| 이동 가능 Place와 잠금 Place의 겹침 | 충돌로 처리 |
| 이동 가능 Place의 strip 이탈 | 충돌로 처리 |
| 유효하지 않은 좌표·회전·수량, hole/quality-zone 충돌 | 기존 검사 유지 |

Legacy C 힌트 배열, V2 `all_items` / `size_regions`, 잠금 PlaceGroup에 적용했다. Config 항목이나 기본값, C ABI 필드 변경은 없다. `src/ffi.rs`, `swarm-core/src/config.rs`, `swarm-core/src/consts.rs`는 작업 직전과 바이트 단위로 동일하다. C API의 기존 잠금 단위인 item_id 의미도 유지한다.

## 구현

- Init 시 부분 배치 검증에 잠금 ID를 전달한다. 허용한 잠금끼리의 겹침 때문에 LBF 재배치나 초기화 오류로 전환하지 않는다.
- 공간 인덱스의 사각형만 배치 좌표까지 확장한다. 실제 strip 경계는 유지하므로 이동 가능 Place에는 원래 경계가 계속 적용된다. strip 밖의 잠금이 quadtree 등록·복구·폭 변경에서 assertion을 일으키는 경로를 보완했다.
- 기존 CollisionTracker의 잠금/잠금·잠금/외부 비용 제외 정책과 결과 검증기를 일치시켰다. Separator 폭 하한과 Polish LP 폭 하한이 잠금의 오른쪽 끝에 묶이지 않도록 했다.
- Explore·Compress·Polish·Reclaim·Structure의 결과 채택과 복구, Stripe 스냅에 같은 정책을 연결했다. N06 후보가 잠금 좌표를 바꾸면 채택·콜백을 거부하고 기존 허용 해를 보존한다.
- 콜백 loss/충돌 상태, JSON 및 C 배열 상태 조회, 최종 JSON 내보내기, handle을 사용하는 경계 충돌 조회에 예외를 반영했다. handle 없는 AABB 조회에는 잠금 정보가 없으므로 기존 기하 판정을 유지한다.
- 모든 Place가 잠겨 있고 전부 x=0 왼쪽에 있으면 V2 내보내기가 원래의 양수 strip 폭을 유지한다. 폭 0을 재구성하다가 FFI 경계에서 프로세스가 종료될 수 있었던 경계 사례를 막았다. 전부 잠겼을 때 탐색·압축은 현재 폭을 유지하며, 0/음수/NaN/무한대 폭 제안은 상태를 바꾸지 않고 거부한다.

Rust에서 잠금 정보를 받지 않는 엄격 검증 API와 Jagua 자체 `Layout::is_feasible()`는 일반 기하 판정이다. 잠금이 있는 공개 실행 경로는 잠금 ID/마스크를 받는 검증 API를 사용한다.

## 검증

새 회귀 테스트는 Legacy·V2 all_items·V2 size_regions를 대상으로 정상 좌표, (-1000,-1000), 상단 밖, 오른쪽 밖, 동일 위치의 잠금끼리 겹침, 잠금 PlaceGroup, 모든 Place가 잠긴 경우를 검사한다. Init 및 Explore/Compress/Polish, Structure, N06의 좌표 보존과 콜백 JSON, 최종 JSON, JSON/C 배열 충돌 조회, 경계 충돌 조회를 확인한다.

코어 테스트는 잠금/잠금·잠금/외부 비용 0, 이동 가능/잠금 및 이동 가능/외부 비용 양수, strip 축소와 rollback, 무한대 변환 거부, 잘못된 폭의 무변경 거부를 검사한다. 기존 warm-start 테스트의 폭 기반 이동 추론은 실제 변환 비교로 교체했다. 잠금의 오른쪽 끝을 벗어나는 정상 결과를 기존 엄격 검증기로 거부하던 테스트는 잠금 마스크를 포함한 검증과 좌표 보존을 함께 확인하도록 변경했다.

중단 지연 검사에서 동시 작업 중 Polish 1516ms가 1500ms 기준을 한 차례 넘었다. 잠금이 없는 refit은 기존 검사 경로를 유지하도록 정리했으며, 빌드·다른 테스트가 끝난 뒤 단독 재검증에서 Explore·Polish·Reclaim 중단 응답 3건이 모두 통과했다. 시간 제한이나 합격 기준을 완화하지 않았다.

| 검증 | 결과 |
| --- | --- |
| Core 단위 및 선택 회귀 | 750 통과, 13 ignored |
| FFI 전체 단위·통합 | 141 통과, 3 ignored |
| Release 잠금 공개 API | 4 통과 (모든 시나리오 포함), 2.93초 |
| C ABI 실측 | ABI 15 / Config 776 bytes / Structure 116 bytes / features 0x0F |

최종 refit 일반 경로 정리 후 Core 전체 선택 범위를 다시 검증했다. FFI 전체 검증 이후 최종 소스의 Release 잠금 공개 API 4건도 통과했다.

```powershell
cargo test -p swarm-core --features 'simd capmods adaptive structure-first' --lib --test frozen_policy --test explore_code_optimization --test parallel_optimization --test place_groups --test postprocess_phases --test stop_latency --test structure_pipeline --offline -- --test-threads=1
cargo test -p swarm --features 'simd capmods adaptive structure-first' --tests --offline -- --test-threads=1
cargo test -p swarm --release --features 'simd capmods adaptive structure-first' --test frozen_policy --offline -- --test-threads=1
```

실행 로그: `target/frozen-policy-core-regression.log`, `target/frozen-policy-root-regression.log`, `target/frozen-policy-release-tests.log`.

전체 WPF 빌드·화면 테스트 및 앱 배포는 수행하지 않았다. C ABI 필드·크기·설정은 유지한다.

## 변경 추적

Git 저장소가 아닌 작업 폴더이므로 작업 시작 시점의 별도 백업과 비교했다. 이전 Explore 최적화 결과는 이번 변경의 기준으로 보존했다.

- 작업 전 백업: `target/frozen-policy-baseline-2026-09-07/`
- 변경 패치: `docs/SWARM_FROZEN_PLACE_POLICY_2026-09-07.patch`
- 소스 전후 해시와 설정 동일성: `docs/SWARM_FROZEN_PLACE_POLICY_HASHES_2026-09-07.json`



## Release 산출물

- DLL: `target/release/capswarmcore_100.dll`
- 크기: 6346240 bytes
- SHA-256: `218c431701a67134b3a410d65c08e20758e5b11034c4ed92550c3993110a261c`
- ABI 함수 반환값 0. ABI 15, Config 776, Structure 116, features 0x0F를 실제 로딩으로 확인했다.
- `target/release/deps/capswarmcore_100.dll`과 최상위 Release DLL의 해시가 동일하다.

변경 파일 rustfmt 검사, 추가 행 공백 검사, 작업 전후 설정 파일 동일성 검사, 소스 해시 검사가 통과했다.
