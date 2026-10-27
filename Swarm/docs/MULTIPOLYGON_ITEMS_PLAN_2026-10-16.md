# 다중 폴리곤 아이템(옵션 C) 작업 계획 — 2026-10-16

브랜치 `v1_0_2_1` (기준 `v1_0_1_7` 끝 커밋 `f1c858c`).

사용자 지시: 그룹(place_groups)을 "n개 폴리곤을 상대 좌표로 함께 옮기는" 방식 대신 **엔진이 폴리곤
여러 개를 가진 아이템 하나로 다루는** 방식(옵션 C)으로 바꿀 것. jagua-rs 를 고쳐야 하면 **검증된
최신 버전**으로 할 것. 중간 단계마다 커밋·푸시하고, 판단이 필요하면 묻지 말고 결정한 뒤 내역을 남길 것.

## 0. 버전 결정

- crates.io 최신 릴리스는 **jagua-rs 1.1.0**(태그 `v1.1.0`)이다. 저장소 master 에는 미출시 2.0.0
  작업(쿼드트리 제거)이 올라가 있지만 태그·릴리스가 없으므로 "검증된 최신"은 1.1.0 으로 본다.
- 현재 우리 의존성은 `0.8.1` 요구 → 로컬 해석 0.8.3. 0.8 → 1.x 는 JSON 형식과 Rust API 가 모두
  바뀐 메이저 변경이다(상류 `docs/releases/1.0.0.md`, `1.1.0.md`).
- 상류 커밋: `9a0b28d86ec05ef287c0b641269be83b6f29c230` (태그 `v1.1.0`).
- 포크 위치: `third_party/jagua-rs/` (상류 `jagua-rs/` 서브크레이트 복사, MPL-2.0 라이선스 유지,
  변경 내역은 `third_party/jagua-rs/CAPLANET_CHANGES.md`). 루트·swarm-core·swarm-wasm 의
  `Cargo.toml` 이 경로 의존성으로 가리킨다. 크레이트 이름은 `jagua-rs` 그대로 두어 `use jagua_rs::` 가
  바뀌지 않는다. `Cargo.lock` 은 저장소에 없으므로(.gitignore) 경로 의존성이 재현성에 가장 안전하다.

## 1. 1단계 — 0.8.3 → 1.1.0 마이그레이션 (알고리즘 변경 없음)

### 영향 범위(집계)

jagua-rs 를 쓰는 파일 90개(swarm-core 85, cpswcore 4, wasm 1). 바뀌는 API 사용처 약 3,000곳.
주요 항목과 사용 횟수:

| 변경 | 횟수 | 대응 |
|---|---|---|
| `Instance` 트레이트 삭제, `instance.item(i)` → `&Arc<Item>` | 65 + 59 | 구체 타입 사용 |
| `pi.item_id` → `pi.item().idx()` (우리 `item_id` 필드와 혼재) | 473 | 사용처별 구분 |
| 필드 → 접근자: `.placed_items()`, `.layout()`, `.container()`, `.strip()`, `.layout_snapshot()`, `.shape_cd()`, `.shape_orig()`, `.d_transf()`, `.shape()` | 358/352/53/16/115/156/178/163 | 기계적 치환 |
| `SPolygon` 필드 → 접근자: `.bbox()`, `.area()`, `.diameter()`, `.vertices()` | 319/70/12/70 | 기계적 치환 |
| `SPProblem::new`, `change_strip_width`, `fit_strip` → `Result` | 69/88/58 | `?`/`expect` 로 전환. 실패 시 상태 불변 보장은 유지 |
| `RotationRange` / `allowed_rotation` → `allowed_orientations().rotations()` | 161/51 | 반사(reflection)는 쓰지 않음 |
| `HazardEntity` 열거형 → `BasicHazardEntity` + 트레이트 | 62 | 이름 치환 |
| `haz_key_from_pi_key` → `haz_key(&entity)` | 9 | |
| `collect_*_collisions` 에 필터 인자 추가 | 19 | `NoFilter` |
| `is_feasible` → `is_collision_free` | 26 | |
| `Importer::new` 시그니처(분리 간격은 `with_min_item_separation`) | 27 | |
| `io::parser::Parser` 삭제 | 5 | `serde_json` 직접 사용 |
| `SPPlacement { item_id }` → `item_idx` | 73 | |

### 포크에서 바꾸는 것(우리 쪽 변경을 줄이기 위한 의도적 이탈)

1. **`ExtItem` 역직렬화가 기존 `allowed_orientations` 를 계속 받는다.** 1.x 는 이 필드를 거부하고
   `orientation.rotation` 을 요구한다. 우리 C ABI / C# / Java / WPF 입력(V2 wire 안의 ExtSPInstance)과
   `data/input` 의 고정 입력 25개가 이 형식이다. 포크에서 `allowed_orientations` 를 `orientation` 으로
   변환해 받되(`None`/생략 → continuous, `[]` → `[0]`, 목록 → discrete), 둘 다 있으면 `orientation` 을
   우선한다. 출력(`ExtSPSolution`)은 `reflected` 가 false 면 생략되므로 기존과 같다.
2. **우리 전용 충돌 파이프라인이 쓰는 내부를 다시 공개한다.** `CDEngine::quadtree`, `get_virtual_root`,
   `detect_containment_collision`, `quadtree` 모듈(`QTHazPresence`, `QTNode::collect_collisions`,
   `hazards`)이 1.x 에서 crate-private 가 됐다. `eval/specialized_jaguars_pipeline.rs`(실행 시간의
   90% 이상)와 `eval/lbf_evaluator.rs` 가 쓰므로 `pub` 으로 되돌린다.
3. 그 밖의 캡슐화(필드 → 접근자)는 상류를 따른다. 기계적 치환이라 위험이 낮고, 포크를 상류에 가깝게
   두어야 나중에 2.0 을 받기 쉽다.

### 의미가 바뀌는 지점(주의)

- 1.x 는 아이템 `idx`(내부 밀집 인덱스)와 `external_id` 를 분리한다. 우리 코드는 "id == 인덱스" 를
  전제로 한다. 임포트에서 id 가 0..n 연속임을 계속 강제해 `idx == external_id` 를 보장한다.
- `[0]` 만 허용하는 회전은 `RotationRange::None` 으로 정규화된다(0.8 에서는 `Discrete([0])`).
  회전 후보를 만드는 코드가 `None` 을 "고정 0°" 로 다루는지 확인한다.
- 1.1.0 은 단순화가 원래 경계 상자를 넘지 않게 고쳤고, 대리 도형의 pier 생성 기준점이 바뀌었다
  (가져온 아이템은 중심 정렬이라 영향 없음). 결과가 소폭 달라질 수 있다.
- `SPProblem::restore` 가 폭이 다른 해도 스스로 처리한다(`util/frozen.rs` 의 우회가 불필요해질 수 있음).

### 검증

- 전체 `cargo test`(swarm-core 787 + 통합) 통과.
- 같은 시드의 LBF 초기 해 밀도와 짧은 탐색 결과를 마이그레이션 전후로 비교해 큰 차이가 없음을 확인.
  (이 PC 는 다른 작업 부하가 커서 시간 기준 비교는 신뢰하지 않는다.)

### 커밋 단위

1. 포크 투입 + 포크 변경 1·2 + 의존성 전환 (빌드는 아직 깨진 상태로 두지 않기 위해 2와 묶을 수 있음)
2. swarm-core 마이그레이션
3. cpswcore(ffi) · wasm · 테스트 마이그레이션, 빌드 녹색
4. 문서 갱신

## 2. 2단계 — 다중 폴리곤 아이템

1.1.0 의 JSON 은 이미 `ExtShape::MultiPolygon(Vec<ExtPolygon>)` 을 정의하고 임포트에서
"No support for multipolygons yet" 으로 거부한다. 포크에서 이를 구현한다.

### 엔진(포크)

- `Item`: `shapes_orig: Vec<Arc<OriginalShape>>`, `shapes_cd: Vec<Arc<SPolygon>>` 로 확장.
  기존 `shape_orig()`/`shape_cd()` 는 **첫 부분**을 돌려 단일 폴리곤 아이템의 동작을 그대로 유지한다.
  `shapes_cd()`, `n_parts()`, 그리고 전체를 감싸는 볼록 껍질 `hull_cd()` 를 추가한다(표본 추출·경계
  상자·지름용, 충돌 해저드로는 등록하지 않음). 모든 부분은 **하나의 기준 좌표계**(합집합 중심으로
  정렬)를 공유한다. 면적은 부분 면적의 합.
- `PlacedItem`: 변환된 부분 도형 `shapes: Vec<Arc<SPolygon>>`. 단일 폴리곤이면 기존과 동일.
- `Layout::place_item`: 부분마다 해저드 하나를 등록한다. 엔티티는
  `BasicHazardEntity::PlacedItem { id, dt, pk, part }`. 자기 제외 필터는 `pk` 로 비교한다(키 n개
  대신). `is_collision_free` 는 부분마다 질의한다.
- 임포트: `MultiPolygon` 아이템을 받는다. 구멍은 거부(기존 정책 유지).

### 우리 쪽

- `place_groups` 입력을 **그룹당 아이템 하나(다중 폴리곤)** 로 변환한다. 멤버의 초기 상대 자세로
  부분 도형을 만들고, 출력 시 그룹 자세 ∘ 초기 상대 자세로 멤버별 자세를 복원한다(기존 출력 계약과
  동일). 허용 회전은 멤버들의 교집합(기존과 동일).
- 충돌 손실(`quantify`, `tracker`, `specialized_jaguars_pipeline`): 같은 pk 의 부분들을 합산한다.
  표본 추출은 껍질 경계 상자를 쓴다. NFP 샘플러(기본 꺼짐)는 다중 폴리곤 아이템에서 비활성.
- 기존 `PlaceGroups` 강체 이동 경로는 설정(`place_group_mode: multipolygon | rigid`)으로 남겨 비교·
  회귀 확인에 쓴다. 기본값은 측정 뒤 정한다.

### 검증

- 그룹 단위 테스트·통합 테스트 통과, #912 입력으로 `rigid` 와 `multipolygon` 폭·시간 비교
  (`tests/group_workers_probe.rs`).

## 3. 진행 기록

### 2026-10-16 — 1단계 마이그레이션

- 상류 `v1.1.0`(`9a0b28d8`)의 `jagua-rs/` 서브크레이트를 `third_party/jagua-rs/` 에 넣고 세 크레이트의
  의존성을 경로로 바꿨다. 포크 변경은 `third_party/jagua-rs/CAPLANET_CHANGES.md` 에 번호를 붙여 적었고
  소스에는 `// CAPLANET:` 주석으로 표시했다. 계획의 1·2 외에 다음이 더 필요했다.
  - 3 `SPProblem::layout_mut` — 고정 조각·웜스타트·시험 이동이 레이아웃을 직접 만진다.
  - 4 `Item::with_idx / with_shape_cd / with_shape_orig / with_allowed_orientations` — 임포트 뒤
    후처리(레인 재인덱싱, mm 단순화, 음수 offset 제거)와 검증 테스트가 필드를 바꾸던 자리.
  - 5 `Container::set_base_cde` — 고정 조각이 원단 밖에 있을 때 색인 루트를 넓히는 `util/frozen.rs`.
  - 6 음수 `min_item_separation` 허용 — 겹침 허용(piece_overlap_mm)은 음수 분리 간격으로 표현된다.
    상류는 음수를 거부한다.
  - 7 테스트 전용 스냅샷 변경자(`placed_items_mut`, `set_d_transf`, `set_item`, `strip_mut`) —
    검증 테스트가 일부러 망가진 해를 만드는 데 쓴다(`#[doc(hidden)]`).
- 우리 쪽 변경은 컴파일러 위치 기반 스크립트로 했다(`error[E0616]` 의 파일:줄:열에서 필드명을 접근자로
  치환하는 식). 손으로 고친 곳: `util/io.rs` 임포트(두 단계 `Importer`, `Arc<Item>`, 외부 id = 내부
  인덱스 강제), `util/frozen.rs`(1.x `restore` 가 폭 차이를 스스로 처리하므로 우회 제거),
  `eval/specialized_jaguars_pipeline.rs`(`collect_collisions_until`, 정적 `detect_containment_collision`),
  `eval/sep_evaluator.rs`(버퍼 재사용을 `clone_from` 으로), `grouped_engine.rs` 의 그룹 회전 교집합
  (`ExtOrientation` 기반, `orientation_values`), `original_overlap.rs` 의 정책 적용.
- 의미 변화로 테스트를 고친 곳:
  - `prepared_geometry` 테스트: 1.x 는 회전 목록을 생성 시 정규화·중복 제거하므로 `0.` 대 `-0.` 같은
    비트 수준 차이를 만들 수 없다. 값이 다른 회전으로 바꿨다(캐시가 회전 변경을 감지하는지는 그대로 검증).
  - `util/io.rs` 의 상류 비교 테스트: 1.x `import_instance` 는 분리 간격을 인스턴스 필드에서 읽고 폭 공식이
    다르다. 기하·순서만 상류와 비교하고 폭·offset 은 우리 0.8 방식 값으로 검증한다.
- `cargo check --all-targets` 가 swarm-core·cpswcore·swarm-wasm 모두 녹색. 남은 경고는 마이그레이션 전부터
  있던 것(`updated_gls_weight`, `until` 미사용)뿐이다.
- 테스트에서 드러나 추가한 포크 변경:
  - 8 `SPProblem::change_strip_width_with` — 1.x `change_strip_width` 는 컨테이너를 새로 만들어 조각을 바로
    다시 등록한다. 원단 밖의 고정 조각은 넓히지 않은 색인에 등록되는 순간 쿼드트리 디버그 불변식에 걸린다
    (`frozen_policy` 테스트). 등록 전에 색인 루트를 넓힐 수 있는 변형을 추가했다.
  - 9 `import_instance` 의 분리 간격·초기 폭 — 1.x 는 분리 간격을 인스턴스 JSON 에서 읽고(임포터 설정 무시)
    초기 폭 공식도 다르다. 우리 호출처 28곳(대부분 테스트)은 임포터에 간격을 설정하므로 0 이 아닌 인스턴스
    값이 없으면 임포터 값을 쓰고, 폭은 0.8 공식(면적/높이)을 유지한다. 겹침 허용(음수 간격)이 여기에 걸려
    `OriginalOverlapGuard` 가 꺼지던 것이 원인이었다(`issue10_local_micro_overlap…` 테스트).
  - 레인(size_regions)의 조각은 `external_id` 도 레인 내부 인덱스로 맞춘다(`Item::with_external_id`).
    1.x `export` 가 외부 id 를 쓰므로 그대로 두면 `local_to_global` 색인이 범위를 벗어났다
    (grouped_engine 테스트 8개, 그리고 포이즌된 직렬화 뮤텍스 때문에 `structure_ffi` 6개가 연쇄 실패).
- 결과: swarm-core 단위 787 + 통합 전부, cpswcore 단위 161 + 통합 전부 통과. 바뀐 의미가 테스트에
  드러난 곳은 위 세 가지와 `prepared_geometry`·`util/io.rs` 테스트 조정뿐이다.

### 2026-10-16 — 2단계 (1) 엔진: 다중 폴리곤 아이템 (커밋 `8ccc959`)

- 포크 변경 10. `Item::new_multipart`(부분들의 원본·충돌 외곽, 공유 좌표계, 볼록 껍질 = `shape_cd()`),
  `PlacedItem::shapes()`, 부분마다 해저드 하나(`BasicHazardEntity::PlacedItem { part }`),
  `PlacedItemFilter(pk)`, `Layout::haz_keys(pk)`, `is_collision_free` 부분별 검사, 임포트의
  `ExtShape::MultiPolygon` 지원. 단일 폴리곤 아이템은 부분 하나가 껍질 `Arc` 를 공유해 동작이 그대로다.
- 포크 테스트 3개: 두 정사각형 사이 틈이 비어 있고, 각 부분이 해저드이며, 스냅샷 복원·제거가 부분 전체를
  다룬다.

### 2026-10-16 — 2단계 (2) swarm-core: 부분 단위 충돌 계산

정확해야 하는 경로만 부분 단위로 바꾸고, 나머지 휴리스틱은 껍질(보수적)로 둔다.

- `util/parts.rs`(신규): `PartBuffers`(부분별 변환 버퍼), `item_collides`(대리 도형 → 정확 검사, 부분 전체),
  `placed_collides`, `placed_pair_collide`, `hull_root`.
- 정확 경로: `quantify/tracker.rs`(부분쌍 손실 합산, 패널티는 껍질 면적 기준 유지),
  `eval/specialized_jaguars_pipeline.rs`(해저드의 `part` 로 상대 부분 선택, 자기 부분 전체 제외, 패널티 껍질
  기준), `eval/sep_evaluator.rs`(부분마다 수집·정량화, 남은 상한 전달), `eval/lbf_evaluator.rs`(배치 검사
  부분별, 인증용 `flat_layout_collision_counts` 부분쌍), `structure/evaluator.rs`, `polish/probe.rs`,
  `polish/slide.rs`, `initial/attach.rs` 의 자기 제외 필터(`PlacedItemFilter`), `util/assertions.rs`
  (트래커 검증을 부분 합산으로), `original_overlap.rs`(원본 외곽선을 부분마다 경로로).
- 껍질로 두는 휴리스틱(결과는 보수적, 인증은 정확): `optimizer/explore.rs` 의 "사실상 포함" 판정,
  `initial/compact.rs`, `sample/exact_nfp.rs`·`raster_nfp.rs`·NFP 샘플러(기본 꺼짐), `refplace`, `fabric_edge`,
  구조 우선(motif) 기하. 다중 폴리곤 아이템에서는 이들이 틈을 못 쓰는 쪽으로 틀릴 뿐 잘못된 해를 내지 않는다.
- 테스트 `swarm-core/tests/multipart_items.rs`: 틈에 놓인 조각은 인증 통과, 부분 위 조각은 거부, 트래커 손실
  합산·검증 통과, 분리기가 겹침을 풀고 인증 통과. 기존 단위 787·통합 전부 통과.

### 2026-10-16 — 2단계 (3) cpswcore: place_groups 를 다중 폴리곤 아이템으로

- 설정 `place_group_mode`(`PlaceGroupMode::MultiPolygon` 기본 / `Rigid`). C ABI v20: `CPlaceGroupConfig { mode }` 꼬리
  블록(오프셋 952, 구조체 952 → 960, 정렬 8). C#(`PlaceGroupMode`, WPF 콤보 "그룹 배치 방식"), Java(`placeGroupMode`), JNI.
  0 으로 초기화한 호출자는 다중 폴리곤 방식을 받는다 — 요청하신 방향이 기본값이다.
- `src/group_expansion.rs`(신규): 검증된 V2 입력을 **압축 인스턴스**로 바꾼다. 그룹마다 멤버 원본 외곽선을 힌트
  자세(힌트 계약: 원본 bbox 최소 모서리를 회전해 `tr` 에 둠)로 월드 좌표에 놓은 `multi_polygon` 아이템 하나.
  그룹 힌트는 항등 자세(tr = 합집합 bbox 최소), 고정은 멤버 전부 고정일 때, 허용 회전은 앵커의 제한된 후보에서
  앵커 힌트 각도를 뺀 값, 영역(size_regions)은 멤버를 그룹 아이템으로 치환. 엔진·레인·인증·진단은 압축 id 만
  본다. 공개 경계(`swarm_get_solution_json`, 콜백 JSON, 충돌 상태)에서 `GroupExpansion` 이 멤버 id·자세로 되돌린다
  (멤버 자세 = 그룹 자세 ∘ 힌트 자세, `Transformation` 합성).
- `GroupedStepState`: `expansion` 필드, `export_public_solution`/`export_certified_public_solution`/`expand_flags`.
  내부 호출(`export_solution`, `restored_problem`)은 압축 id 그대로다. 구조 우선 보고서(`global_item_ids`)는
  압축 id 를 보여 준다(진단용, 문서화).
- 테스트: `group_expansion` 단위 2개(힌트 좌표계·회전 합성), `ffi` 의 `place_group_mode_reaches_the_core_config`,
  `multipolygon_place_groups_export_members_with_exact_relative_poses`(두 방식 모두 멤버 id 0·1 과 힌트 상대
  자세를 정확히 유지, 충돌 상태 id 가 공개 id).
- 바꾸지 않은 것: 강체 방식 코드(`PlaceGroups` 전부)는 그대로 남아 `Rigid` 로 선택할 수 있다.
- 테스트에서 드러난 조정:
  - `CSwarmConfig` 는 정렬 8 이라 952 → **960** 이다(956 이 아니다). C# `Marshal.SizeOf` 와 맞는다.
  - 트래커 검증(`util/assertions.rs`)의 기대 손실은 **실제로 부딪힌 부분쌍만** 더한다. 겹침 proxy 는 떨어진 부분에도
    양수(ε²)라 전 부분쌍을 더하면 과대 계산된다 — 부분 고정 그룹 FFI 테스트가 이걸 잡았다.
  - 다중 폴리곤 모드에서는 엔진에 그룹이 없으므로 **구조 우선(structure-first)이 그룹 때문에 꺼지지 않는다**
    (강체 모드는 `user_place_groups_active` 로 P0/P1 을 건너뛰었다). 강체 전용 동작을 보는 테스트 세 개
    (`public_candidates_reject_lock_or_rigid_pose_drift…`, `structure_first_place_group_fallback…`,
    FFI `all_items_place_group_structure_stop_and_wait…`)는 `Rigid` 로 고정했고, 다중 폴리곤 모드에서
    구조 우선이 돌고 멤버 자세가 유지되는 테스트를 추가했다.
  - 그룹 높이 사전 검사(허용 회전 중 어느 것도 원단 높이에 안 맞으면 Init 거부)를 `compact_input` 에 넣었다.
    강체 경로의 `validate_group_geometry` 와 같은 메시지다.

### 2026-10-16 — 측정: #912 그룹 입력, 다중 폴리곤 vs 강체 (커밋 `e816e48`)

`tests/group_workers_probe.rs` 에 `PROBE_GROUP_MODE`(0 = 다중 폴리곤, 1 = 강체), `PROBE_WORKERS`, `PROBE_OVERLAP_MM`
을 추가했다. 탐색 30초, 시드 1·2·3 평균, 단위 mm. 이 PC 는 측정 내내 다른 작업(`nestcli_e4p` 등)이 CPU 의
70% 이상을 쓰고 있어 시간 기준 수치는 참고용이다.

| 방식 | 작업자 | 5초 | 10초 | 20초 | 30초 |
|---|---|---|---|---|---|
| 다중 폴리곤 | 1 | 2357 | 2326 | 2322 | 2317 |
| 다중 폴리곤 | 8 | 2579 | 2381 | 2358 | 2338 |
| 강체 | 1 | 2483 | 2396 | 2379 | 2351 |
| 강체 | 8 | 2706 | 2476 | 2360 | 2351 |

다중 폴리곤 방식이 모든 시점에서 좁다(10초 시점 70~95 mm, 30초 시점 13~34 mm). 이유는 ① 그룹이 조각 하나라
분리기가 부분 손실을 한 번에 평가하고 ② 멤버 사이 틈에 다른 조각이 들어가며 ③ 상대 자세 인증 실패가 구조적으로
없기 때문이다. **기본값을 다중 폴리곤으로 확정한다.** 강체 방식은 설정으로 남긴다.

**초기 폭이 5608 로 보이는 이유(두 방식 공통).** 마이그레이션 전에는 같은 입력의 초기 폭이 2797 이었다. 이 입력의
움직일 수 있는 힌트 23개 중 9개가 원단 아래 경계(y = 0)에 닿아 있는데, jagua-rs 0.8 은 단순화된 충돌 도형이 원본
경계 상자 밖으로 조금 나갈 수 있어 그중 4개가 "원단 밖" 으로 판정됐고, 힌트 배치가 검증에서 떨어져 LBF 로 새로
깔았다(폭 2797). 1.1.0 은 단순화가 원본 경계 상자 안에 머물도록 고쳐져(상류 JeroenGar/sparrow#176) 힌트 배치가
검증을 통과하고, 문서된 계약대로 힌트 위치(폭 5608)를 출발점으로 쓴다. 출발 폭은 넓지만 5초 시점부터 이미 예전
결과보다 좁다(위 표 vs 이전 2382/2587). 즉 회귀가 아니라 워름스타트 계약이 제대로 지켜지는 것이다.

### 2026-10-16 — WPF 샘플 실행에서 드러난 문제: 콜백 충돌 상태가 껍질 기준

사용자가 WPF 샘플로 #912 를 다중 폴리곤 모드로 돌리자 "Polish 인증 snapshot … 충돌=27/35", "인증된 전체 배치 결과
없음" 이 나왔다. WPF 의 충돌 수는 콜백 JSON 의 `collision_status` 를 센 것이고, 그 상태를 만드는 FFI 진단
`compute_callback_diagnostics_with_groups` 가 여전히 `placed.shape()`(그룹의 볼록 껍질)로 쌍 겹침·원단 밖을 판정했다.
그룹 껍질은 틈에 들어간 조각·이웃과 겹치므로 멤버 전부가 충돌로 표시됐다(엔진 인증은 부분 단위라 결과 자체는
정상). 진단을 부분 단위로 고쳤고(`shapes()` 전체 수집, 자기 pk 제외, 부분쌍 `polygons_collide_bidirectionally`,
손실도 부분쌍 합), FFI 테스트에 "그룹 틈에 힌트로 넣은 세 번째 조각" 을 추가해 두 방식 모두 충돌 상태가 전부
false 임을 확인한다. 측정 도구(`group_workers_probe`)도 결과의 충돌 상태 수를 출력한다.

로그의 `[ORIGINAL:VALIDATE] … items 8/14` 는 압축(엔진) id 다 — 진단용 경고라 그대로 둔다.

### 2026-10-16 — 중지 시 최종 밀도 84.93% (같은 폭의 중간 보고는 66.84%)

최종 보고의 밀도는 `GroupedStepState::export_solution` 이 `SPInstance::item_area()` 로 계산하는데, 상류 1.1.0 의
`item_area` 는 `shape_orig` 면적을 더한다. 다중 폴리곤 아이템의 `shape_orig` 는 부분들의 볼록 껍질이라 면적이 부풀었다
(중간 보고는 `LayoutSnapshot::density` → `Item::area()` = 부분 면적 합이라 맞았다). 포크에서 `item_area` 가
`Item::area()` 를 더하도록 바꿨다(포크 변경 10 보강). 포크 테스트: 두 정사각형(면적 2)의 인스턴스 면적이 껍질(4)이 아닌
2 인지 확인.

