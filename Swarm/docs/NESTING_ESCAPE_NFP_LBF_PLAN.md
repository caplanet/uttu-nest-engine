# 의류 네스팅 — 적응형 국소 최적해 탈출 + NFP-LBF 구현 계획

> 원본: `E:\CaPlanit\AiWirks\의류_네스팅_채팅_전체내용.md`
> 대상: `E:\CaPlanit\AiWirks\SwarmParallelLab\AdaptiveAlgorithm`
> NFP 커널 참고: `E:\CaPlanit\AiWirks\SwarmParallelLab\PerfectNFP\swarm-nfp`
> 작성일: 2026-08-28

---

## 0. 요약

원본 MD 문서는 서로 맞물리는 **세 덩어리**를 정의한다. 세 덩어리를 문서에 적힌
수식·가중치·임계값·단계 구성 그대로 `AdaptiveAlgorithm`에 구현한다.

| # | 기법 | 문서 위치 |
|---|---|---|
| A | 정체 감지 기반 적응형 국소 최적해 탈출 (Stagnation-Triggered Adaptive Local-Optimum Escape) | §1 ~ §5 + 구현 예시 |
| B | Conflict Burst + Partial Rebuild (Strip 최소 구간의 광범위 충돌 돌파) | "Strip 최소에 갇혀…" |
| C | NFP Cache → Multiple Ordering → Sequential Placement → Best Initial Nest | "Lbf 배치할때…" ~ 마지막 |

**유사 구현이 아니라 문서 그대로**를 원칙으로 한다. 가중치 `0.35 / 0.30 / 0.20 / 0.15`,
비율 `5~10%`, `15~30%`, `5~15%`, ΔL 구간 `1~2 / 3~5 / 5~10%`, 사다리
`L+6% → L+4% → L+2% → L → L-ε`, 창 `max(100, 3 × 패턴 수)`, 임계 `0.01%p`,
점수식 `Score(p) = w1·L + w2·Waste − w3·Contact + w4·Y`, 우선순위 4단 —
전부 상수 또는 코드로 1:1 대응시킨다.

---

## 1. 문서 → 파일 매핑

### 1-A. 탈출 기법 — `swarm-core/src/escape/`

| 파일 | 문서 절 | 구현 내용 |
|---|---|---|
| `mod.rs` | 전체 | **확정 상수 계약.** 아래 §2의 모든 수치를 `pub const`로 고정 |
| `stagnation.rs` | §1 탐색 정체 감지 | 5개 지표 추적, `max(100, 3×n)` 창, `0.01%p` AND 다양성 판정 |
| `bottleneck.rs` | §2 병목 패턴 선택 | `EscapeScore(i)` 4항 가중합, 최대 공백 탐지, 상위 비율 선택 |
| `levels.rs` | §3 동적 교란 | Level 1 Soft / Level 2 Medium / Level 3 Hard |
| `guided.rs` | §4 Guided Local Search | `Weight(a,b) × CollisionCost(a,b)`, 증가 / 1로 점진 감소 |
| `swarm.rs` | §5 Swarm 추가 처리 | Elite 10~20% 보존, 하위 20~30% 재초기화, 탈출 직후 파라미터 조정 |
| `conflict_burst.rs` | B 전체 | 충돌 그래프, `ConflictEscapeScore(i)`, `EscapeVector(i)`, Adaptive Strip Relaxation, 3 Stage |
| `pipeline.rs` | 구현 예시 (pseudo) | 의사코드 main loop을 그대로 옮긴 탐색 단계 |

### 1-B. NFP-LBF — `swarm-core/src/nfp_lbf/`

| 파일 | 문서 절 | 구현 내용 |
|---|---|---|
| `mod.rs` | 전체 | **확정 상수 계약.** 점수 가중치, 우선순위 표, 예산 목표치 |
| `cache.rs` | NFP Cache | `NfpKey{shape_a_hash, rotation_a, shape_b_hash, rotation_b}`, Geometry Hash, Rayon 병렬 pre-warm, lazy miss |
| `ordering.rs` | Multiple Item Ordering | 6종 ordering + Swap/Move/Block reorder 파생 |
| `placement.rs` | Sequential Placement | NFP 합성, 후보 3종 추출, feasible 판정 |
| `score.rs` | LB + Contact + Void 평가 | `Score(p)` 수식, 4단 우선순위, 근사 Waste/Contact |
| `search.rs` | 전체 파이프라인 | 다중 ordering 평가 + prefix 재사용 + Best Initial Nest |

### 1-C. NFP 기하 커널 — `swarm-nfp/` (PerfectNFP 이식)

`PerfectNFP/swarm-nfp`를 그대로 복사해 workspace member로 추가했다. 이 크레이트는
Clipper2 i64 정수 격자 위에서 동작하는 **완전 다각형 NFP 커널**이다.

```
NfpEngine::prepare / orient / pair_nfp / complete_pair_nfp
         / inner_fit / feasible_region / locate / classify_pair
```

- `pair_nfp(F, M)` = `F + (-M)` — 문서의 "NFP(고정 B, 이동 A) = B ⊕ (−A)"와 동일 규약
- `inner_fit(container, moving)` — 문서의 "Strip boundary까지 고려"에 해당하는 IFP
- 오목 폴리곤·홀·비연결 성분을 모두 정확히 처리한다 (볼록 근사 아님)

`swarm-nfp-ffi`는 이번 범위에 포함하지 않는다 (문서에 FFI 요구 없음, AdaptiveAlgorithm
WPF 샘플도 `capnfp_100.dll`을 참조하지 않음).

---

## 2. 문서 수치의 코드 대응 (확정 계약)

### §1 탐색 정체 감지

```text
최근 max(100, 3 × 패턴 수) 회 동안
효율 개선량 < 0.01%p
    AND
후보 배치 간 위치·순서 다양성이 기준 이하
```

| 상수 | 값 | 근거 |
|---|---|---|
| `STAGNATION_WINDOW_FLOOR` | `100` | `max(100, …)` |
| `STAGNATION_WINDOW_ITEM_FACTOR` | `3` | `3 × 패턴 수` |
| `STAGNATION_EFFICIENCY_EPS` | `0.0001` | `0.01%p` |
| `STAGNATION_DIVERSITY_THRESHOLD` | `0.05` | `기준 이하` |

추적 지표 5종은 전부 구현하되, **탈출 발동 조건은 문서가 명시한 두 조건의 AND**로만
판정한다 (③이동 성공률 ④우측 경계 고정 시간 ⑤반복 충돌 쌍은 `stagnation_intensity`
계산에 반영해 교란 규모를 정한다).

### §2 EscapeScore

```text
EscapeScore(i)
 = 0.35 × RightBoundaryContribution
 + 0.30 × VoidAdjacency
 + 0.20 × BlockingDegree
 + 0.15 × MoveFailureFrequency
```

| 상수 | 값 |
|---|---|
| `W_RIGHT_BOUNDARY_CONTRIBUTION` | `0.35` |
| `W_VOID_ADJACENCY` | `0.30` |
| `W_BLOCKING_DEGREE` | `0.20` |
| `W_MOVE_FAILURE_FREQUENCY` | `0.15` |

네 항 모두 0.0~1.0 정규화 후 가중합. 합 = 1.00.

### §3 Level별 교란 규모

| Level | 문서 | 상수 |
|---|---|---|
| 1 Soft | `대상: 1~2개 패턴` | `L1_TARGET_MIN=1`, `L1_TARGET_MAX=2` |
| 1 Soft | `±1° → ±3° → ±5°` | `L1_TILT_LADDER_DEG = [1.0, 3.0, 5.0]` |
| 2 Medium | `전체 패턴의 약 5~10%` | `L2_RATIO_MIN=0.05`, `L2_RATIO_MAX=0.10` |
| 3 Hard | `오른쪽 15~30% 영역` | `L3_REBUILD_RATIO_MIN=0.15`, `L3_REBUILD_RATIO_MAX=0.30` |
| — | `escape_level = min(escape_level + 1, 2)` | `ESCAPE_LEVEL_MAX = 2` |

Level 1의 회전은 **인스턴스의 허용 회전을 절대 벗어나지 않는다.**
`0°/180°` 이산 집합이면 그 안에서만, 연속 범위를 허용하는 경우에만 Tilt 사다리를 쓴다.

### §4 Guided Local Search

```text
동적 평가값 = Weight(a, b) × CollisionCost(a, b)
반복되면 Weight 증가 / 해결되면 Weight → 1로 점진 감소
```

| 상수 | 값 |
|---|---|
| `GUIDED_WEIGHT_NEUTRAL` | `1.0` |
| `GUIDED_WEIGHT_INCREASE` | `1.30` |
| `GUIDED_WEIGHT_DECAY` | `0.90` |
| `GUIDED_WEIGHT_MAX` | `100.0` |

키는 **item_id 쌍**으로 정규화한다. `PItemKey`는 제거·재삽입 시 바뀌어 학습이 끊긴다.
감쇠는 정확히 `1.0`에서 멈추고 맵에서 제거된다.

### §5 Swarm 다양화

| 상수 | 값 | 문서 |
|---|---|---|
| `SWARM_ELITE_RATIO_MIN/MAX` | `0.10 / 0.20` | `상위 10~20% → Elite로 보존` |
| `SWARM_REINIT_RATIO_MIN/MAX` | `0.20 / 0.30` | `하위 20~30% → 재초기화` |
| `POST_ESCAPE_GLOBAL_BEST_PULL` | `0.5` | `Global Best 추종 강도 ↓` |
| `POST_ESCAPE_EXPLORATION_BOOST` | `2.0` | `랜덤 탐색 및 개별 탐색 강도 ↑` |
| `POST_ESCAPE_BOOST_ITERS` | `20` | `탈출 직후 일정 반복 동안` |

정체가 심할수록 Elite는 20%→10%로 좁아지고 재초기화는 20%→30%로 넓어진다.

### B. Conflict Burst + Partial Rebuild

```text
ConflictEscapeScore(i)
= 0.35 × CollisionDegree
+ 0.25 × StripBoundaryContribution
+ 0.20 × BlockingScore
+ 0.10 × Area
+ 0.10 × Concavity
```

| 상수 | 값 |
|---|---|
| `W_COLLISION_DEGREE` | `0.35` |
| `W_STRIP_BOUNDARY_CONTRIBUTION` | `0.25` |
| `W_BLOCKING_SCORE` | `0.20` |
| `W_CONFLICT_AREA` | `0.10` |
| `W_CONCAVITY` | `0.10` |
| `CONFLICT_REMOVE_RATIO_MIN/MAX` | `0.05 / 0.15` (`상위 5~15% 제거`) |

**Adaptive Strip Relaxation**

```text
collision ratio < 10%   → ΔL = 1~2%
collision ratio 10~30%  → ΔL = 3~5%
collision ratio > 30%   → ΔL = 5~10%
```

| 상수 | 값 |
|---|---|
| `COLLISION_RATIO_LOW / HIGH` | `0.10 / 0.30` |
| `DELTA_L_LOW` | `(0.01, 0.02)` |
| `DELTA_L_MID` | `(0.03, 0.05)` |
| `DELTA_L_HIGH` | `(0.05, 0.10)` |
| `STRIP_RELAXATION_LADDER` | `[0.06, 0.04, 0.02, 0.00]` |
| `STRIP_RELAXATION_EPSILON` | `0.005` (`L - ε`) |

**충돌 방향 기반 강제 이동**

```text
EscapeVector(i) = Σ w_ij × SeparationVector(i,j)

평소 Local Move    1~3 mm
Escape Mode        10~50 mm 또는 bbox 크기의 20~50%
```

| 상수 | 값 |
|---|---|
| `LOCAL_MOVE_MM` | `(1.0, 3.0)` |
| `ESCAPE_MOVE_MM` | `(10.0, 50.0)` |
| `ESCAPE_MOVE_BBOX_RATIO` | `(0.20, 0.50)` |

`w_ij`는 기존 CollisionTracker의 GLS 쌍 가중치를 그대로 쓴다.

**3 Stage 실행 순서**

```text
Stage 1  Collision Push    → 충돌 벡터 기반 강제 분리
Stage 2  Strip Relaxation  → 현재 길이보다 확장
Stage 3  Destroy & Repair  → 충돌 중심 패턴 5~15% 제거 후 재삽입
                           → 이후 Left/Bottom Compaction
                           → 사다리로 base 폭까지 점진 복귀
```

**Particle 계층**

```text
Best 20%   → 현재 배치 유지 + 약한 local search
Middle 50% → Collision Push + 부분 재삽입
Worst 30%  → Strip Relaxation + 큰 Destroy & Repair
```

`PARTICLE_BEST_RATIO=0.20`, `PARTICLE_MIDDLE_RATIO=0.50`, `PARTICLE_WORST_RATIO=0.30`.

### C. NFP-LBF 점수식

```text
Score(p) = w1·L(p) + w2·Waste(p) − w3·Contact(p) + w4·Y(p)
```

| 상수 | 값 | 의미 |
|---|---|---|
| `DEFAULT_SCORE_W1_STRIP_LENGTH` | `1.0` | `L(p)` 배치 후 strip length |
| `DEFAULT_SCORE_W2_WASTE` | `0.35` | `Waste(p)` 새로 생기는 unusable void |
| `DEFAULT_SCORE_W3_CONTACT` | `0.25` | `Contact(p)` 접촉 길이 — **부호 `−`** |
| `DEFAULT_SCORE_W4_Y` | `0.10` | `Y(p)` 세로 위치 |

**Contact만 감점**이다. 같은 strip length라면 주변 형상에 더 밀착되는 위치가 이긴다
(문서의 Candidate A / B 예시를 단위 테스트로 그대로 옮긴다).

**후보 우선순위 (문서 순서 고정)**

```text
1순위 : 배치 후 Strip Length 최소
2순위 : x 최소
3순위 : y 최소
4순위 : 주변 패턴과의 접촉도 최대
```

`CANDIDATE_PRIORITY_ORDER = [StripLength, MinX, MinY, MaxContact]`

**예산 목표치**

| 상수 | 값 | 문서 |
|---|---|---|
| `DEFAULT_N_ORDERINGS` | `50` | `30~50개 LBF ordering 평가` |
| `DEFAULT_CACHE_BUDGET_MS` | `5000` | `Cold Cache < 2 sec 목표 / 최악 < 5 sec` |
| `DEFAULT_TOTAL_BUDGET_MS` | `10000` | `Cold start 10초 이내` |

---

## 3. NFP Cache 설계 (문서 핵심 요구 3가지)

### 3-1. 키는 Geometry Hash — instance 번호 금지

```text
잘못된 방식                      올바른 방식
Pattern_001 × Pattern_020        GeometryHash(A) × GeometryHash(B)
Pattern_001 × Pattern_021
Pattern_001 × Pattern_022
```

```rust
NfpKey {
    shape_a_hash: GeometryHash,
    rotation_a:   i64,
    shape_b_hash: GeometryHash,
    rotation_b:   i64,
}
```

`GeometryHash::of_polygon`은 centroid 상대 좌표 → 0.001mm 정수 격자 양자화 →
사전순 최소 정점 기준 순환 정규화 → 해시. **형상이 같으면 반드시 같은 해시**가 나온다.

문서 예시 그대로: 156 폴리곤 / 고유 35종이면 `156C2 = 12,090` → `35C2 + self ≈ 630`.

정규화 키 `canonical()`은 `NFP(A,B) = −NFP(B,A)`를 이용해 절반만 저장한다.

### 3-2. Rayon 병렬 + thread-local 후 1회 병합

```text
156 polygons → 12,090 unique pairs → Rayon parallel iterator
→ 8 threads 기준 약 1,511 NFP / thread → Cache 저장
```

> 캐시 저장을 연산 중 mutex로 매번 잠그지 않고, thread-local 결과를 모아서 마지막에
> 한 번 합치는 구조라면 더 유리합니다.

→ `par_iter().map(...).collect::<Vec<_>>()` 후 단일 스레드 삽입. 계산 구간에 락 없음.

### 3-3. Lazy miss + 예산 초과 허용

```text
Nesting 시작 → 필요한 NFP 요청 → Cache Hit ─→ 즉시 사용
                                  │
                                Cache Miss → NFP 계산 → Cache 저장
```

pre-warm이 예산을 넘겨 부분 캐시로 끝나도 정상 동작한다. 미계산 pair는 요청 시 계산된다.

---

## 4. Sequential Placement 후보 추출

> 후보 점들은 보통 **NFP vertex**, **NFP edge와 다른 NFP edge의 교점**,
> **strip boundary와 NFP의 교점** 등에서 나옵니다.

```rust
pub enum CandidateSource {
    NfpVertex,                  // 각 NFP(o, item)의 정점
    NfpEdgeIntersection,        // 서로 다른 두 NFP의 edge 교점
    StripBoundaryIntersection,  // IFP 경계 × NFP edge, IFP 자체 정점
}
```

feasible 판정:
- 모든 `NFP(o, item)`에서 `Location::Inside`가 아닐 것 (`Boundary`는 접촉 배치로 허용)
- IFP 안에 있을 것 (`Inside` 또는 `Boundary`)

폭발 방지: NFP bbox 사전 필터 + edge bbox 필터 + 교점 상한 4096 + 평가 후보 상한 256.

### prefix 재사용

```text
Base ordering → NFP-LBF → 좋은 결과 확보
이후 Swap 2 items / Move 1 item / Block reorder
    → 부분적으로 달라진 suffix만 재배치
```

> 가령 156개 중 80번째 이후 순서만 바뀌었다면 앞의 79개 배치를 그대로 재사용할 수 있습니다.

파생 ordering의 변경 지점을 `len/2 ..= len-1`에서 뽑아 재사용 이득을 최대화한다.
직전 ordering의 prefix 구간에 미배치가 있으면 재사용하지 않는다.

---

## 5. MultiPolygon Linked Group 제약

> 하나의 의류 아이템을 구성하는 여러 폴리곤은 각각 독립적으로 교란하면 안 되고
> **동일한 Transform을 공유하는 Linked Group 단위**로 이동·회전·재삽입해야 합니다.

기존 `optimizer/place_groups.rs`의 `PlaceGroups`가 이미 이 제약을 구현하고 있으며,
`Separator::move_item`이 그룹 단위 원자 이동을 보장한다. 새 탈출 모듈은:

- 병목 선택 시 같은 그룹의 멤버는 대표 1개만 후보에 넣는다
- 이동은 전부 `Separator::move_item`을 경유한다 (직접 remove/place 금지)
- `has_place_groups()`가 참이면 그룹을 쪼갤 수 있는 R&R 경로를 건너뛰고 대체 동작으로 전환한다

---

## 6. 통합 지점

### 6-1. `config.rs`

```rust
pub struct SwarmConfig {
    // ... 기존 필드 ...
    pub escape_enabled: bool,                        // 탐색 단계 교체 스위치
    pub escape: crate::escape::AdaptiveEscapeConfig, // 문서 임계값 묶음
    pub nfp_lbf: NfpLbfSeedConfig,                   // NFP-LBF 시드 스위치
}
```

`SwarmConfig`가 `Copy`라서 `NfpLbfSeedConfig`도 `Copy`로 두고, 실행 시점에
`to_config()`로 `Vec`을 가진 `NfpLbfConfig`를 만든다.

기본값은 **둘 다 꺼짐**이다. 기존 동작을 그대로 유지한 채 A/B 비교가 가능해야 한다.

### 6-2. `optimizer/mod.rs`

```text
optimize_configured
   │
   ├─ nfp_lbf.enabled ?
   │     └─ build_best_initial_nest() → SPSolution → warm start
   │          (실패하면 기존 LBFBuilder로 폴백)
   │
   └─ 탐색 단계 선택
         escape_enabled          → escape_driven_exploration_phase   (신규)
         feature="adaptive"      → exploration_phase_adaptive        (기존)
         그 외                   → exploration_phase                 (upstream)
```

NFP-LBF 결과는 `solution_validation_error`로 검증한 뒤에만 seed로 넘긴다.
잘못된 seed가 optimizer에 들어가는 경로를 만들지 않는다.

### 6-3. FFI / WPF

`src/ffi.rs`, `src/ffi_step.rs`의 설정 구조체에 두 스위치를 노출한다.
`csharp/Natives/SwarmNative.cs`에 대응 필드를 추가해 WPF 샘플에서 켤 수 있게 한다.
(런타임 설정이므로 cargo feature 변경 없이 동작한다 — WPF csproj의 빌드 커맨드
`--features "simd,capmods,adaptive,structure-first,jni"`는 그대로 둔다.)

---

## 6-4. 실측 결과 (여성WSjk 156-패널 마커, 16코어)

입력: `E:\CaPlanit\여성WSjk_비교용-G_New_Swarm_신규2.json` (156 폴리곤, strip_height 1524,
원본 정점 median 179 / max 637)

### 형상 등가류 (Geometry Hash)

문서가 요구한 "instance 번호가 아니라 GeometryHash로 캐싱"이 그대로 동작한다.

| | 값 |
|---|---:|
| 배치 폴리곤 | 156 |
| **고유 형상 등가류** | **105** |
| 등가류 없을 때 pair | 12,246 |
| **등가류 적용 후 pair** | **5,565** (−55%) |

### NFP 커널 비용 — 세 차례 개선

초판 커널은 삼각분할(가장 잘게 쪼갠 볼록 분해)을 써서 pair NFP의 원시 조각 수가
`삼각형(A) × 삼각형(B)`로 폭발했다. 한 쌍에 6.6초가 걸리는 경우가 있었다.

| 단계 | 조각 수 median / max | pair(50,51) | 전체 prewarm (tol 0.001) |
|---|---:|---:|---:|
| 초판 (삼각분할) | 32 / 108 | 6,590 ms | (측정 불가, 수십 분) |
| + Hertel–Mehlhorn 볼록 병합 | **13** / 71 | 1,476 ms | 17.8 s |
| + 계층적 chunked union | 13 / 71 | — | **2.6 s** |
| + 오목 단순화 0.02 | — | — | **0.16 s** |

세 개선 모두 **무손실**이다. 볼록 병합은 조각들이 여전히 도형을 정확히 타일링하고,
계층적 union은 같은 집합의 합집합이며, 오목 단순화는 원본의 상위집합을 만든다.

### 오목(concave) 전용 단순화

볼록 분해 조각 수는 **오목 정점 수**를 따라간다. 볼록 정점은 조각 수를 늘리지 않는다.

| tolerance | 평균 정점 | 평균 **오목** 정점 | 평균 조각 | `|조각−(오목+1)|` | `|조각−정점|` |
|---|---:|---:|---:|---:|---:|
| 0.001 | 35.8 | 17.6 | 16.8 | **1.85** | 18.97 |
| 0.01 | 14.3 | 5.4 | 5.5 | **0.96** | 8.81 |

따라서 NFP 비용을 줄이는 레버는 오목 정점뿐이고, 전역 `poly_simpl_tolerance`를 올리면
볼록 정점까지 함께 뭉개져 형상 충실도만 잃는다. 그래서 강도를 분리했다
(`NfpLbfConfig::concave_simplify_tolerance`, NFP 라이브러리 **전용**).

prewarm 벽시계 시간(5,565 pair):

| `poly_simpl_tolerance` | 오목 0 | 0.02 | 0.05 | 0.10 |
|---|---:|---:|---:|---:|
| **0.001** | 2,677 ms | **162 ms** | 100 ms | 71 ms |
| 0.01 | 267 ms | 99 ms | 66 ms | 51 ms |

`0.001` + 오목 `0.02`(162 ms)가 `0.01` + 오목 끔(267 ms)보다 **더 싸면서 볼록 충실도는
0.001을 유지**한다.

밀도 대가 (albano.json, ordering 12개):

| 오목 강도 | strip 길이 | 밀도 | 전체 시간 | 최선 ordering |
|---:|---:|---:|---:|---|
| 0 | 12,140.96 | 71.70% | 1,007 ms | maxdim-desc |
| 0.01 | 12,140.96 | 71.70% | 611 ms | maxdim-desc |
| **0.02** | **12,080.70** | **72.06%** | **453 ms** | **difficulty-tiered** |
| 0.05 | 12,243.10 | 71.11% | 346 ms | concavity-desc |

`0.02`에서는 밀도 손해가 없다(오히려 +0.36%p). `0.05`부터 손해로 돌아선다.

### 안전성 근거

- 오목 정점 제거 → 만입부가 채워져 도형이 **커진다** → 원본의 상위집합
- 상위집합으로 만든 NFP는 실제 금지영역을 **과대평가**한다
- 따라서 NFP가 "배치 가능"이라 한 위치는 jagua-rs CDE 기준으로도 반드시 충돌이 없다
- 반대 방향(볼록 정점을 깎아 도형을 줄이는 것)은 실제 겹침을 놓치므로 하지 않는다

---

## 6-5. 구현 중 발견해 고친 결함

| 결함 | 증상 | 수정 |
|---|---|---|
| IFP 경계 여유 없음 | 커널(f64 격자) → jagua-rs(f32) 변환에서 `y_min = −1e-7`로 컨테이너 이탈, 12개 중 2개 무효 | IFP 컨테이너를 f32 상대오차 이상으로 축소 (`ifp_inset`) |
| 우선순위를 정규화 값에 적용 | `strip_length`·`x`는 `strip_width_cap`, `y`는 `strip_height`로 나눠 축마다 tie 기준이 달라짐 → 한 줄 배치 (strip 126.9) | 4단 우선순위를 **실제 mm** 값으로 비교 (strip 42.0) |
| 접촉 배치가 f32에서 미세 겹침 | `Location::Boundary` 채택 후 5쌍 겹침 | pair NFP를 정사각형 clearance 커널로 0.01 mm 팽창 |

세 번째는 커널이 정수 격자라 내부 오차는 없지만, **f32 경계로 넘길 때** 허용오차가 필요하다는
점을 실측으로 확인한 결과다. 방향은 겹침 허용이 아니라 간격 확보다.

### 6-5-1. "NFP 를 켜도 결과가 LBF 와 똑같다" — 원인 2건

사용자 보고: WPF 에서 NFP 초기배치를 켜도 밀도가 LBF 와 같은 70% 그대로였다.

| 결함 | 증상 | 수정 |
|---|---|---|
| step API 에 배선 누락 | WPF 는 `swarm_init` → `swarm_alg_phase` 경로를 쓰는데 `ffi_step.rs` 에 `nfp_lbf` 참조가 **0건**이었다. 모든 배선이 `optimize_configured`(CLI/wasm 전용)에만 들어갔다 | `LegacyStepState` 에 설정 4종 추가, `swarm_init` 에 시드 블록, `select_exploration_phase()` 추가. 소스 수준 회귀 테스트로 고정 |
| 탐침 예산 1024 | 후보 수는 배치 진행에 따라 폭증(20번째 1,607개 → 155번째 13,514개)하는데 예산이 고정이라 후기 아이템이 후보의 7.5%만 보고 결정. **NFP-LBF 가 LBF 보다 넓은 결과를 내서 시드가 항상 기각**됐다 | `MAX_CANDIDATE_PROBES` 를 `NfpLbfConfig::max_candidate_probes` 로 빼고 기본 **4096** |
| 좌측 순차 패스 공회전 | 예산의 절반을 x 오름차순 앞에서부터 태우는데, 배치가 진행되면 좌측은 이미 꽉 차 있어 **hit 이 0건**. 실효 예산이 절반이었다 | 프론티어 기준으로 구간 분리 — 좌측은 `partition_point` 로 잘라 균등 간격 1/8 예산만, 나머지는 프론티어 이후 순차 |
| cdylib 에 로거 없음 | `log` 는 로거 미등록 시 모든 매크로가 no-op. `init_logger`(fern) 호출자가 CLI 뿐이라 **네이티브 로그가 한 줄도 안 나왔다**. "설정 미전달"과 "전달됐지만 기각"을 구분할 수 없었다 | `src/ffi_log.rs` — `swarm_set_log_callback` FFI + C# `SwarmOptimizer.SetLogSink` + WPF 실행 로그 연결 |

탐침 예산 실측 (여성WSjk 156-패널, ordering 50개, `concave_simplify_tolerance = 0.02`,
LBF 단독 = 7325.05 / 69.45%):

| 예산 | strip 폭 | 밀도 | LBF 대비 | 전체 시간 |
|---:|---:|---:|---:|---:|
| 1024 (구 기본값) | 7479.09 | 68.02% | −1.43%p | 8 s |
| 2048 | 7277.71 | 69.90% | +0.45%p | 10 s |
| **4096 (신 기본값)** | **7082.27** | **71.83%** | **+2.38%p** | 12 s |
| 8192 | 7057.59 | 72.08% | +2.63%p | 14 s |
| 16384 | 7054.85 | 72.11% | +2.66%p | 15 s |

품질은 4096 에서 포화한다. 시간이 예산에 정비례하지 않는 이유는 초과 탐침 대부분이
`MAX_EVALUATED_CANDIDATES` 도달 전에 빠르게 기각되기 때문이다.

> 소스 docstring 에 있던 "4096 = 19.90초" 표는 `NfpMask` / `locate_prepared` 도입 **이전**
> 측정이었다. 그 낡은 수치가 1024 를 고른 근거였다.

### 6-5-2. LBF 단독 vs NFP 단독 vs 좁은 쪽 (탐침 4096, 회전 0/180° 켬)

| 인스턴스 | LBF 단독 | NFP 단독 | 초기 차이 | 탐색 20초 후 (LBF 시드 → NFP 시드) |
|---|---:|---:|---:|---|
| wsjk156 (156) | 69.45% | **71.83%** | +2.38%p | 74.65% → **75.32%** (+0.67%p) |
| albano (24) | 73.57% | **82.14%** | +8.57%p | 86.76% → **87.07%** (+0.31%p) |
| dagli (30) | 70.64% | **74.93%** | +4.29%p | 86.07% → 85.68% (−0.38%p) |
| fu (12) | 70.34% | **74.95%** | +4.61%p | 90.24% → 89.70% (−0.54%p) |

초기 배치에서는 NFP 가 전부 이긴다. 다만 **탐색을 거치면 조각 수가 적은 인스턴스에서는
차이가 씻긴다** — Swarm 이 같은 수렴점으로 가기 때문이다. 조각이 많을수록(wsjk156, albano)
시드 우위가 살아남는다.

`albano` 는 회전을 끄면 −1.51%p 로 **진다**. `allowed_orientations = [0, 180]` 인데
NFP-LBF 가 `allow_180 = false` 기본값 때문에 0도만 쓰기 때문이다. LBF 는 인스턴스가 선언한
회전을 그대로 쓴다. wsjk156 은 `[0]` 뿐이라 이 항목의 영향을 받지 않는다.

결론: **LBF 를 빼지 않는다.** NFP 가 실패하거나(예산 고갈, 라이브러리 생성 실패) 더 나쁠 때
폴백이 필요하고, LBF 는 1초라 비교 비용이 사실상 없다.

### 6-5-3. NFP 초기 배치 전략 7종 — 실측과 감사

측정 조건: `data/input/wsjk156.json` (= 사용자 지정 `여성WSjk_비교용-G_New_Swarm_신규2.json`,
sha256 동일), 156조각, 탐침 4096, **유효 ordering 12개 고정**, seed 0.

> ordering 수를 고정한 이유: 프로덕션에서는 `scale_orderings` 가 전략 비용만큼
> ordering 을 줄여 예산을 지킨다. 그걸 A/B 에 그대로 쓰면 "전략이 나쁜 것" 과
> "ordering 을 덜 돈 것" 이 섞인다 — 첫 측정에서 팔마다 12/8/6/3/1 개로 갈렸다.

기준선 LBF 단독 = 7325.05 / **69.45%**

| 전략 | 폭 | 밀도 | LBF 대비 | 전략 기준선 대비 | 시간 |
|---|---:|---:|---:|---:|---:|
| 기준 (전부 끔) | 7089.96 | 71.75% | +2.30%p | — | 4.4s |
| ① 인스턴스 회전 | 7089.96 | 71.75% | +2.30%p | 0 | 5.2s |
| ② 쌍별 클러스터링 | 7137.92 | 71.27% | +1.82%p | −0.48%p (13쌍) | 6.1s |
| ③ 접촉 최대화 | 7065.60 | 72.00% | +2.55%p | +0.25%p | 5.2s |
| ④ 포켓 우선 | 7080.33 | 71.85% | +2.40%p | +0.10%p | 5.3s |
| ⑤ 빔 서치 (폭 2, 기본) | 7055.72 | 72.10% | +2.65%p | +0.35%p | 9.5s |
| ⑤ 빔 서치 (폭 4) | 7073.72 | 71.91% | +2.47%p | +0.16%p | 18.4s |
| ⑥ best-fit (12) | 7077.75 | 71.87% | +2.43%p | +0.12%p | 27.8s |
| ⑦ 슬라이딩 (2회) | 7054.86 | 72.11% | +2.66%p | +0.36%p | 24.7s |
| **②+③+④+⑦** | **7047.29** | **72.18%** | **+2.74%p** | **+0.43%p** | 26.1s |
| ②+③+④+⑤+⑦ | 7047.66 | 72.18% | +2.73%p | +0.43%p | 30.3s |
| 전부 켬 | 7074.62 | 71.91% | +2.46%p | +0.16%p | 49.3s |

**①이 이 마커에서 효과 0인 것은 정상이다** — 156개 전부 `allowed_orientations = [0]` 이라
회전 선택지가 하나뿐이다. `[0, 180]` 인 albano 에서는 같은 전략이 **+8.57%p** 를 낸다.

**② 단독은 손해지만 조합에서는 이득이다.** 쌍을 붙이면 그 자리가 고정되어 뒤 조각의
자유도가 줄지만, ④가 안쪽을 먼저 채우고 ⑦이 뒤에서 당겨 주면 순이득으로 바뀐다
(③+⑦ 72.00% -> ②+③+④+⑦ 72.18%).

#### 감사에서 고친 결함

이 표는 **감사 후** 수치다. 감사 전에는 세 전략이 망가져 있었다.

| 전략 | 감사 전 | 감사 후 | 원인 |
|---|---:|---:|---|
| ⑥ best-fit | 58.95% (95s) | **71.87% (27.8s)** | "strip 을 가장 적게 늘리는 조각" 만 보면 **작은 조각이 무조건 이긴다**. 큰 조각이 끝까지 밀려 마지막에 몰린다. 면적으로 정규화하고(`growth x strip_height / area`) 큰 조각 우선 티어(`BEST_FIT_AREA_BAND = 0.20`)를 되살렸다 |
| ④ 포켓 우선 | 71.75% (효과 0) | **71.85%** | 좌측 구간 경계에 조각 폭 대신 **IFP 폭**을 빼서 `left_end` 가 늘 0이었다. 두 값은 `IFP폭 = strip폭 − 조각폭` 관계라 의미가 정반대다 |
| ③ 접촉 최대화 | 71.07% (−0.68%p) | **72.00% (+0.25%p)** | 접촉 지표가 `(0.0..=BAND).contains(&gap)` 로 걸러 **bbox 가 겹치는 후보의 접촉을 0으로** 쳤다. 후보는 NFP 로 feasible 판정을 통과했으므로 bbox 겹침은 두 조각이 **맞물렸다**는 뜻이다. 오목한 의류 패널에서 후보의 76.2%가 그 경우였다 |
| ② 클러스터링 | 쌍 0개 | **13쌍 적용** | (a) 이득 정의가 구조적으로 음수였고, (b) 종류 상한 8이 "종류=조각수" 인 의류 마커와 안 맞았고, (c) 파이프라인이 쌍을 **미배치 조각에만** 붙여 전부 배치되는 마커에서는 영영 안 돌았다. 앵커를 놓은 **직후** 붙이도록 바꿨다 |

`DEFAULT_BEAM_WIDTH` 를 4 -> **2** 로 내렸다. 폭을 넓히면 노드는 늘지만 각 노드가 뻗는
가지 수는 그만큼 늘지 않아, 넓은 빔이 서로 비슷한 상태로 채워진다. 폭 2가 품질도
시간도 가장 좋다(72.10% / 9.5s vs 폭 4의 71.91% / 18.4s).

#### 남은 한계

* **③은 탐침 예산에 민감하다.** 접촉 위치를 충분히 열거해야 의미가 생긴다.
  탐침 8192 에서는 이득이 더 크다.
* **접촉 지표는 여전히 bbox 근사다.** 겹침을 최대 접촉으로 세는 것까지는 고쳤지만,
  실제 접촉 길이(폴리곤 경계 교차 길이)는 아니다. 후보를 만든 obstacle 인덱스를
  노출하면 정확히 셀 수 있다 — 다음 개선 후보.
* **⑤/⑥ 경로는 ② 클러스터를 쓰지 않는다.** 조각 선택 순서를 그쪽이 정하므로 쌍의
  전제가 깨진다. 켜면 그 사실을 로그로 알린다.

---

## 7. 진행 상태

| 단계 | 상태 |
|---|---|
| PerfectNFP `swarm-nfp` 이식 + workspace 등록 | 완료 |
| 커널 성능 3단 개선 (볼록 병합 / 계층적 union / 오목 단순화) | 완료 |
| `escape/` 7개 모듈 | 완료 |
| `nfp_lbf/` 6개 모듈 | 완료 |
| `config.rs` 스위치 + const 기본값 | 완료 |
| `optimizer/mod.rs` 배선 (`optimize_with_escape`) | 완료 |
| SIMD (population_diversity / 후보 판정·근사 점수 / 충돌 그래프) | 완료 |
| FFI ABI v6 (escape + nfp_lbf) + C# 대응 | 완료 |
| FFI ABI v7 (원단 겹침 허용) + C# 대응 | 완료 |
| 난이도 사전분류 ordering (`DifficultyTiered`) | 완료 |
| `cargo test -p swarm-core --lib` | 290 통과 / 0 실패 |
| `cargo test -p swarm-nfp` | 43 통과 / 0 실패 |
| `cargo test -p swarm --lib ffi::` | 13 통과 / 0 실패 |
| WPF 샘플 빌드 | 완료 (오류 0, 경고 14 — 기존과 동일) |
| C# 테스트 | 51 통과 / 1 실패 (`LaneCount` 머신 의존, **기존 실패**) |
| 임시 진단 파일 정리 | 완료 |

기존 실패 2건은 이번 작업과 무관함을 변경분 stash 후 재현으로 확인했다.

* `SwarmInputInspectorTests.Parse_V2AllItems` — `LaneCount`가 1 대신 CPU 코어 수(16)를 반환
* `bench_exact_nfp_wsjk` — 데이터 파일 누락으로 `read_spp_input` 실패

## 7-2. NFP-LBF 성능 최적화 이력

문서 목표는 `LBF ordering 1회 100~200 ms`였다. 네 단계로 줄였고 **결과는 매 단계 동일함을
해시로 검증**했다 (`pieces` 전체의 item_id / rotation / translation 비트).

| 단계 | wsjk156 (3 orderings) | 배수 |
|---|---:|---:|
| 초판 (삼각분할 NFP) | 측정 불가 (수십 분) | — |
| + Hertel–Mehlhorn 볼록 병합 | 17.9 s | — |
| + 계층적 chunked union | (같은 단계에 포함) | |
| + 오목 단순화 0.02 | | |
| + 로컬 좌표 질의 / 래스터 마스크 / 반전뷰 캐시 | 4.67 s | 3.8x |
| + `PreparedRegion` (locate 재정규화 제거) | 2.30 s | 2.1x |
| + 반전 뷰 병렬 pre-warm | **1.55 s** | 1.5x |

`locate` 호출당 비용은 **36 µs -> 0.26 µs** (140배)로 떨어졌다. 재정규화를 매 호출 하던 것이
원인이었고, `swarm-nfp`에 `PreparedRegion` + `locate_prepared`를 추가해 해결했다.

되돌린 시도 두 가지 — 측정이 판정했다.

* **프론티어 밴드 제한** — 20.4 s -> 18.6 s 로 거의 안 줄고 strip 이 7112 -> 7568 로 나빠졌다.
  비용의 지배항이 edge x edge 교점이 아니었기 때문이다.
* **마스크 해상도 256셀** — locate 호출이 8% 밖에 안 줄었는데 마스크 생성 비용이 커져
  전체가 7.7 s -> 9.4 s 로 악화됐다. 남은 경계 히트는 해상도 아티팩트가 아니라 **실제로 다른
  NFP 경계 위에 있는 접촉점**이라 해상도로 없앨 수 없다.

작업 중 실제 버그도 하나 잡았다. 래스터 마스크가 `x = max_x` 의 수직 변을 통째로 빠뜨려
경계에 구멍이 뚫렸고 내부가 외부로 오판됐다(albano 에서 CDE 가 겹침 51건으로 검출).
마스크 확정 결과를 `locate` 와 100만 건 규모로 대조하는 크로스체크로 찾아냈다.

---

## 7-3. 탐색 단계 NFP 활용 5종

기존 Swarm 탐색은 "이 조각이 어디로 갈 수 있나"를 전부 **근사**로 답한다 — 난수 샘플링,
bbox MTV, 래스터 격자. 그 근사들을 정확한 NFP 로 바꾸는 선택지를 **항목마다 독립적으로**
추가했다. `NfpIntegrationConfig` 로 켜고 끄며, **전부 기본 꺼짐**이다.

| 항목 | 대상 | 기존 근사 |
|---|---|---|
| 순수 NFP 순서탐색 | 탐색 단계 전체 (`nfp_lbf/exploration.rs`) | Swarm 연속 완화 |
| ① NFP 샘플러 | `sample/nfp_sampler.rs` | 난수 1,000개 + CDE |
| ② 정확한 분리벡터 | `escape/conflict_burst.rs` | bbox MTV |
| ③ NFP hole filling | `optimizer/polish/holes.rs` | 래스터 격자 |
| ④ NFP 재삽입 | `optimizer/lns.rs` | 샘플링 + CDE |

각 항목은 플래그가 꺼지면 기존 경로가 **한 비트도 바뀌지 않는다**. 그게 A/B 측정의 전제다.

**단위 근거** (각 담당이 전용 픽스처로 측정)

* ② 오목 삼각형 NFP 에서 (4,4) 기준 정확 분리벡터 `(1.0, 1.0)`. bbox 근사는 축 정렬만
  가능해 **방향 자체가 틀리고** 크기도 2.8배 과대하다.
* ③ 폭 2.0 틈 / 격자 셀 4.0 mm 픽스처에서 래스터는 `moved=0` 으로 통째로 놓쳤고,
  NFP 는 `moved=1`, x_max 37.9 -> 16.91.

**A/B 측정 (10초 예산)**

| 설정 | fu.json 밀도 | albano 밀도 |
|---|---:|---:|
| 기준 (전부 끔) | 90.18% | 86.84% |
| ② 정확한 분리벡터 | 90.20% | 86.84% |
| ③ NFP hole filling | 90.15% | 86.84% |
| ④ NFP 재삽입 | 90.22% | 86.84% |
| **순수 NFP 순서탐색** | **86.97%** | **85.47%** |

**결론 — 순수 NFP 순서탐색은 Swarm 을 대체하지 못한다.** 두 인스턴스 모두에서 명확히 진다
(-3.2%p / -1.4%p). 순서만 바꿔서는 조각을 0.5 mm 밀어 넣는 미세 조정을 못 하기 때문이다.
LP 압축으로 보완했지만 연속 완화가 하는 일을 메우지 못했다. swarm 논문의 주장이 실측으로
재확인된 셈이다. **NFP 는 초기 배치(NFP-LBF)로는 유효하지만 탐색 단계 대체로는 부적합하다.**

**A/B 재측정 (60초 예산)**

| 설정 | fu.json 폭 / 밀도 | albano 폭 / 밀도 |
|---|---:|---:|
| 기준 (전부 끔) | 31.58 / 90.23% | 9923.55 / 87.73% |
| ② 정확한 분리벡터 | 31.58 / 90.23% | 9923.50 / 87.73% |
| ③ NFP hole filling | 31.59 / 90.22% | 9923.50 / 87.73% |
| ④ NFP 재삽입 | 31.59 / 90.22% | 9923.45 / 87.73% |
| ②+④ 함께 | 31.58 / 90.23% | 9923.50 / 87.73% |
| **순수 NFP 순서탐색** | **31.61 / 90.16%** | **10042.85 / 86.68%** |

**최종 판정**

**①~④ 는 최종 밀도에 기여하지 않는다.** 폭 차이가 0.01~0.10 mm 수준이고 밀도는 소수점
둘째 자리까지 같다. 10초에서 보였던 "②④ 가 미세하게 이긴다"는 **노이즈였다** — 예산이
늘자 Swarm 이 어차피 같은 곳에 도달했다. 근사를 정확하게 바꿔도 **최종 수렴점이 바뀌지
않는다**는 뜻이고, 이는 Swarm 의 근사가 이미 충분히 좋다는 증거다.

**순수 NFP 순서탐색은 60초에서도 진다.** 격차는 줄었다(fu.json -3.2%p -> -0.07%p,
albano -1.4%p -> -1.05%p). 그러나 **어떤 인스턴스에서도 이기지 못한다.**

그래도 ①~④ 코드는 남긴다. ③ 은 전용 픽스처에서 래스터가 통째로 놓치는 틈을 찾았고
(x_max 37.9 -> 16.91), 그런 병리적 형상이 실제 마커에 들어오면 유일한 해법이다.
플래그가 꺼져 있으면 기존 경로가 한 비트도 바뀌지 않으므로 유지 비용도 없다.

**요약 — NFP 의 자리는 초기 배치다.** 탐색 단계는 Swarm 의 연속 완화가 맡는 것이 맞다.


## 8. 검증 계획

### 단위 테스트 (외부 데이터 무의존)

- `EscapeScore` 가중치 합 = 1.00, 각 항 0~1 정규화
- `ConflictEscapeScore` 가중치 합 = 1.00
- `max(100, 3×n)` 창 계산
- `0.01%p` 임계 판정 — 두 조건 AND
- `CollisionRatioBand` 경계값 (`0.10`, `0.30`) 분기
- `StripRelaxationLadder` 출력 순서 `+6% → +4% → +2% → 0% → −ε → None`
- `GUIDED_WEIGHT_DECAY` 반복 시 정확히 `1.0`에서 정지
- `Score(p)`의 Contact 부호가 `−`인지
- 4단 우선순위가 문서 순서대로 동작하는지
- 문서의 Candidate A(공백 큼) vs B(밀착) 예시에서 B가 선택되는지
- `GeometryHash` — 평행이동/정점 순서 회전에 불변, 형상이 다르면 다름
- `NfpKey::canonical()` — 스왑 시 부호 반전 플래그

### 통합 확인

- `cargo build --release --features "simd,capmods,adaptive,structure-first,jni"` 무경고 통과
- 기본 설정(둘 다 꺼짐)에서 기존 동작이 바뀌지 않는지
- `data/input/*.json`으로 NFP 캐시 생성 시간이 문서 목표(2~5초) 범위인지
- WPF 샘플 빌드 및 네이티브 DLL 복사 확인

---

## 9. 명시적 범위 밖

- `swarm-nfp-ffi` (`capnfp_100.dll`) — 문서에 FFI 요구 없음, AdaptiveAlgorithm WPF가 참조하지 않음
- NFP 캐시의 **디스크 직렬화** — 문서는 "Cache serialization < 500 ms"를 목표로만 언급하고
  포맷을 정의하지 않았다. 인메모리 캐시 + 동일 실행 내 pre-warm까지 구현한다.
- 실제 벤치마크 튜닝 — 구현과 별개 작업
