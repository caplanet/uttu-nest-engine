# R&R · swap · hole fill 개선 및 충돌 안전성 검증

대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`의 네이티브 Rust 소스. 2026-09-08.
이전 분석: `SWARM_RR_HOLE_SWAP_REVIEW_2026-09-08.md`.
Config 기본값과 C ABI를 변경하지 않고 실행 코드와 검증 경계를 수정했다.

## 적용한 변경

| 구분 | 변경 및 수용 조건 |
|---|---|
| 탐색용 교란 swap | 충돌을 허용하는 탐색 연산의 역할을 유지한다. 이후 Separator 결과에 정밀 형상·수량·허용 회전·잠금 좌표·그룹 상대 좌표 검증을 적용한다. |
| Separator swap | 가중 비용 감소, 비가중 국소/전체 비용 비증가 조건을 유지한다. 수치 합산에 대한 기존 상대 허용 오차 1e-5를 적용한다. 그룹 외의 비잠금 조각은 교환 후보로 허용한다. |
| 일반 R&R | 그룹 멤버는 장애물로 유지하고 개별 이동 가능한 조각만 제거·재삽입한다. 임시 충돌이 남을 수 있다는 전제는 유지하며, 실제 proposal 비용 전후와 후속 분리 필요 여부를 로그에 남긴다. 중단 시 기존 트랜잭션 복원 동작을 유지한다. |
| NFP R&R | 기존 nfp_reinsertion 설정을 실제 R&R 래퍼에 연결했다. 첫 호출에서 캐시를 생성하고 같은 phase의 호출에서 재사용한다. 준비 불가능한 회전/형상은 기존 확률 샘플링으로 폴백한다. Native/Structure/그룹 lane/Escape/Reclaim 경로에 연결한다. |
| Profile R&R / DensitySeam | 그룹 멤버를 개별 victim 및 suffix 이동에서 제외한다. suffix 이동의 수용은 작은 충돌 비용만으로 판단하지 않고 partial exact 검증도 통과해야 한다. |
| Hole fill | raster 및 NFP 경로 모두 그룹 밖 비잠금 조각을 이동할 수 있다. 그룹 멤버는 움직이지 않는 장애물로 남는다. |
| Occupancy grid | 실제 ceil 행×열 곱을 검사하고 격자를 확대해 max_cells 이내로 제한한다. checked multiplication 및 fallible reservation을 사용한다. 표시·영역 갱신·flood fill에 중단 확인을 추가했다. |
| Hole fill 비용 | 후속 후보가 필요할 때만 topology를 재계산하고, 곧 폐기할 pocket 정보를 중복 갱신하지 않는다. 앵커 전체 정렬을 partial selection + 상위 K 정렬로 바꿨다. 이동 전 공간을 다음 후보가 재사용하는 동작은 유지한다. |

## 임시 충돌과 공개 결과의 경계

1. 교란 swap/R&R은 내부 탐색 후보를 만든다. 이 상태를 feasible로 인정하지 않는다.
2. Separator가 후보를 분리한다. 실패하면 탐색을 계속하거나 이미 검증된 incumbent로 돌아간다.
3. Explore/Adaptive/Escape의 feasible 목록과 callback은 exact 검증을 통과한 해만 포함한다. 초기 상태가 infeasible이면 초기 스냅샷을 feasible 목록에 넣지 않는다.
4. 비용이 0이어도 정밀 검증에 실패하면 후보를 폐기하고 해당 탐색을 종료한다. stale loss를 반복해서 학습하거나 인증하지 않는다.
5. 종료 시 Separator를 검증된 최선 결과에 맞춰 복원한다. 진입부터 infeasible이고 끝까지 인증 결과가 없으면 core Explore는 빈 목록을 반환한다. 상위 native 경계는 기존 인증 결과로 폴백하거나 오류를 반환한다.
6. Compress 및 Polish의 feasible callback 직전에도 같은 검증을 적용한다. Reclaim은 R&R 이후 재탐색 전에 분리 결과를 인증한다.
7. 기존 native callback/JSON export 검증을 유지한다. 잠금-잠금 겹침과 잠금의 strip 이탈은 허용하지만, 비잠금-잠금 충돌과 비잠금 이탈은 거부한다. 잠금 좌표 자체는 보존한다.

## 성능 효과의 범위

- 폭 1e9, 높이 1, max_cells 400,000인 예에서 기존 면적 기반 하한은 cell=50, 실제 20,000,000셀을 허용했다. 새로운 실제 차원 검사는 cell=2500, 400,000셀로 제한한다.
- 앵커 후보 수 G, 사용 수 K일 때 전체 O(G log G) 정렬을 O(G + K log K)로 줄였다.
- NFP 준비/쌍 캐시는 R&R 호출마다 다시 만들지 않는다. R&R이 호출되지 않으면 재삽입용 준비 비용도 발생하지 않는다.
- 이동 후 connectivity 재계산 자체는 정확성을 위해 남아 있다. 이 변경만으로 전체 탐색 시간이 일정 비율 빨라진다고 주장하지 않는다. 새 certification 비용은 초기/성공 후보 경계에 추가된다.

## 추가 요청: Compress / Polish의 piece_overlap_mm

**두 단계 모두 이미 적용되어 있음을 확인했다.** C ABI의 양수 `piece_overlap_mm`는 `min_item_separation = -piece_overlap_mm`로 변환된다. Init importer가 각 조각을 공차의 절반씩 축소한 `shape_cd`를 만들며 이후 단계는 동일한 instance를 사용한다.

- Compress: Separator와 CollisionTracker가 `shape_cd`로 충돌을 검사한다.
- Polish slide: CollisionProbe가 `item.shape_cd`로 검사한다.
- Polish LP: 모델이 현재 `PlacedItem.shape`(충돌용 도형)를 사용한다.
- Polish raster/NFP hole fill: 후보 생성과 최종 CDE 검사 모두 `shape_cd`를 사용한다.
- 최종 인증: snapshot의 표시 도형만 신뢰하지 않고 같은 instance의 `shape_cd`와 transform으로 재구성한다. 따라서 허용한 겹침을 다시 불법 충돌로 처리하지 않는다.

두 개의 40×40 mm 원본 조각으로 실제 Compress→Polish를 실행했다. 공차의 단순 전달 여부에 더해, 각 단계 출력의 원본 도형을 재구성하여 다음 겹침량을 측정했다. LP 외 slide 및 hole fill도 활성 상태이며, 콜백마다 exact 검증을 수행했다.

| LP | 설정 공차 | Compress 후 원본 겹침 | Polish 후 원본 겹침 |
|---|---:|---:|---:|
| 꺼짐 | 0 mm | 0.0000 mm | 0.0000 mm |
| 꺼짐 | 2 mm | 1.9885 mm | 1.9885 mm |
| 켜짐 | 0 mm | 0.0000 mm | 0.0000 mm |
| 켜짐 | 2 mm | 1.9885 mm | 1.9884 mm |

공차 초과·원단 좌/상/하 경계 이탈·콜백 인증 실패가 없었다. 적용 누락이 없으므로 공차 관련 생산 코드는 변경하지 않았고 `overlap_semantics.rs`에 회귀 테스트를 추가했다. 이 측정은 직사각형 fixture의 값이며 모든 형상의 겹침량이 설정값과 정확히 같아야 한다는 의미는 아니다.

## 검증

- Rust core: **747 통과**, 13개 ignored. lib 708 + 관련 integration 39. `target/rr-implementation-final-validation.log`.
- Native API: **100 통과**. Legacy/V2, 잠금 예외, callback, export, ABI 구조 테스트 포함. `target/rr-implementation-native-tests.log`.
- 추가 overlap phase 테스트: **1 통과**(공차 0/2 mm × LP off/on 4개 조합). `target/rr-overlap-phase-tests.log`.
- 마지막 격자 중단 체크 보정 후 optimizer 재검증: **108 통과**, 3개 ignored. 위 core 수에 중복 포함되어 총계에 더하지 않는다. `target/rr-implementation-optimizer-evidence.log`.
- Explore / Polish / Reclaim의 중단 응답 테스트가 모두 기존 1.5초 한도를 통과했다. Config/테스트 시간 한도를 늘려 우회하지 않았다.
- Config/Cargo/header 파일 5개는 변경 전 baseline과 바이트 단위로 동일하다.

임시 충돌을 실제로 만든 fixture의 결과:

| 연산 | 교란 전 | 교란 후 | 후속 분리 후 | 수용 결과 |
|---|---:|---:|---:|---|
| 탐색용 swap | 0 | 142.70882 | 0 | exact 인증 성공 |
| Random R&R | 356.81232 | 406.43317 | 1.4816208 | 충돌이 남아 인증 거부; 기존 exact incumbent 유지 |

R&R이 항상 제한 시간 안에 분리된다는 뜻은 아니다. 이 회귀는 충돌이 일시 증가하는 실제 경로와, 분리되지 않은 결과를 최종 해로 채택하지 않는 조건을 모두 확인한다.

Polish 중단 경로에서는 중단된 라운드 뒤의 refit/추가 인증을 생략하고, 이미 인증한 incumbent로 복원하도록 개선했다. 인증은 초기 상태와 수용할 후보에서 수행하며 동일 결과를 종료 시 다시 검사하는 중복 작업을 제거했다.

Release 빌드 완료: `simd capmods adaptive structure-first` feature, offline Cargo release profile.

- DLL: `target/release/capswarmcore_100.dll`, 6,410,752 bytes.
- DLL 직접 로드 후 handshake: ABI **15**, CSwarmConfig **776 bytes**, CStructureFirstConfig **116 bytes**, features **0x0F**.
- SHA-256: `22b92a21db15ac67f297d4c92448a66d945674f12b5943119a520f612126d3f8`.
- 변경 전 대비 패치: `docs/SWARM_RR_HOLE_SWAP_IMPLEMENTATION_2026-09-08.patch`.
- 수정 소스·테스트 21개 및 로그 해시: `target/rr-implementation-evidence.json`.
- 추가된 줄의 공백 오류 검사 통과. 기존 변경분은 작업 시작 baseline과 비교하여 이번 패치에서 분리했다.

## 범위 및 제한

- rigid group 전체를 제거·재삽입하는 envelope 기반 R&R은 구현하지 않았다. 이번 변경은 그룹 멤버를 보호하면서 그룹 밖 조각의 기존 연산을 복구한다.
- ProfilePlacementEvaluator 기반 R&R은 해당 프로파일 점수 경로를 유지한다. 일반 R&R의 NFP 재삽입 연결과는 별개다.
- 전체 workspace 검사에서 swarm-wasm의 jagua-rs 0.7.2/0.8.1 타입 불일치가 확인되었다. 이번 네이티브 변경과 별도이며 WASM 빌드 성공으로 보고하지 않는다.
- 임시 충돌을 금지하면 교란 연산의 탐색 기능을 없애게 된다. 이번 수정은 그 후보를 후속 분리와 정밀 검증 없이 공개 결과에 채택하지 않도록 하는 것이다.
