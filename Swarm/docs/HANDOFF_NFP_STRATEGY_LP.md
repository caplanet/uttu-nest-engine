# 인수인계 — NFP 초기배치 전략 7종 + LP·QP 솔버

작성: 2026-08-29 · 갱신: 2026-08-29 · 상태: **배선·감사·측정 완료**

> **갱신 알림** — 아래 §4 "남은 작업" 의 4-1 ~ 4-3 은 **전부 끝났다**(커밋 `0013103`).
> 전략은 WPF 에서 켜면 실제로 동작하고, Phase 감사로 결함 4건을 고쳤으며, 지정
> 데이터로 측정까지 마쳤다. 실측표는 `NESTING_ESCAPE_NFP_LBF_PLAN.md` §6-5-3 에 있다.
> 남은 것은 §4-4(LP 두 경로 통합)와 §8(다음 개선안)이다.

현재 트리는 **빌드 경고 0 / 오류 0**, `swarm-core` 테스트 **478 통과**,
`swarm` FFI 테스트 84 통과, C# 55 통과다. 언제든 이어서 시작할 수 있다.

---

## 1. 이번에 왜 이 작업을 했나

사용자 보고에서 출발했다.

> "초기 배치도 lbf 단독으로 실행했을 때랑 nfp+lbf 로 실행했을 때랑 결과가 완전히 같던데"

원인이 넷이었고 전부 고쳤다(커밋 `9002794`). 그 과정에서 "NFP로 초기배치를 하는
다른 전략은 뭐가 있나" 라는 질문이 나왔고, 그 답으로 일곱 가지를 구현하기로 했다.

**사용자 지시:** *"기능 구현할 때 기존 코드는 참고하지마"*
→ 일곱 전략의 **판단 로직은 전부 알고리즘 정의에서 새로 작성**했다.
기존 `nfp_lbf::placement::best_placement` / `place_sequence` 를 복사·변형하지 않았다.

**다만 후보 열거(NFP 커널 + 캐시)는 포크하지 않았다.** 그 코드는 이 코드베이스에서
성능·정확성이 가장 민감하고 실측으로 여러 번 교정된 곳이라, 두 벌로 갈라 두면
한쪽만 고쳐지는 순간 조용히 어긋난다. 전략 계층은 그것을 **후보 오라클**로만 쓴다.

```
nfp_lbf::SequentialPlacer   후보 오라클 — "여기에 놓을 수 있나" (기존, 포크 안 함)
        |
        v
nfp_strategy                정책       — "무엇을 어디에 놓을까" (전부 신규)
```

---

## 2. 완료된 것

### 2-1. 코어 — 신규 파일

| 파일 | 내용 |
|---|---|
| `swarm-core/src/nfp_strategy/config.rs` | `NfpStrategyConfig` — 7전략 on/off + 노브, `normalized()`, `estimated_cost_factor()`, `enabled_labels()` |
| `swarm-core/src/nfp_strategy/mod.rs` | `StrategyContext` / `AnchorOption` / `AnchorMetrics` + 접촉·낭비 평가 |
| `swarm-core/src/nfp_strategy/rotations.rs` | ① `instance_rotation_set`, `item_rotation_table` |
| `swarm-core/src/nfp_strategy/cluster.rs` | ② `Cluster`, `build_clusters`, `expand_cluster` |
| `swarm-core/src/nfp_strategy/select.rs` | ③ `SelectionRule`, `pick_best`, `pick_top_k` / ⑥ `pick_next_piece` |
| `swarm-core/src/nfp_strategy/beam.rs` | ⑤ `place_sequence_beam`, `BeamPlacement` |
| `swarm-core/src/nfp_strategy/sliding.rs` | ⑦ `compact`, `SlidingOutcome` |
| `swarm-core/src/lp_solver/mod.rs` | `LpProblem` / `LpRow` / `LpSolution` / `LpBackend` / `backend_for` |
| `swarm-core/src/lp_solver/microlp_backend.rs` | 단체법 (LP) |
| `swarm-core/src/lp_solver/clarabel_backend.rs` | 내점법 barrier (**LP + QP**) |
| `swarm-core/src/lp_solver/clp.rs` | COIN-OR Clp — 런타임 DLL 로딩 |
| `swarm-core/src/lp_solver/xpress.rs` | FICO Xpress — 런타임 DLL 로딩 |

④ **포켓 우선**만 별도 파일이 없다. 후보 **탐침 순서**를 바꾸는 전략이라
오라클 쪽(`placement.rs`의 `set_pocket_first`)에 구현했다.

### 2-2. 코어 — 수정한 파일

* `nfp_lbf/placement.rs` — 전략 계층용 공개 접근자 추가
  (`shape_bounds_area`, `allowed_rotations`, `world_bounds`, `set_probe_frontier`,
  `set_pocket_first`, `strip_height`, `Aabb` 공개) + ④ 포켓 우선 탐침 모드
* `config.rs` — `SwarmConfig`에 `nfp_strategy`, `lp_solver` 추가
* `optimizer/mod.rs` — `optimize_with_escape`에 `lp_solver` 파라미터 추가
* `optimizer/polish/mod.rs` — `polish_phase_with_nfp`에 `lp_solver` 추가
* `optimizer/polish/lp.rs` — 솔버 백엔드 분기 + `build_generic_model`
* `lib.rs` — `pub mod lp_solver; pub mod nfp_strategy;`
* `Cargo.toml` — `clarabel = "0.11"`, `libloading = "0.8"`

### 2-3. FFI — ABI v10

```
offset 616  nfp_strategy   CNfpStrategyConfig   12 x 4 = 48 B
offset 664  lp_solver      CLpSolverConfig       4 x 4 = 16 B
size   680  (v9 는 616)
```

기존 블록 오프셋은 전부 보존했다. 새 노브는 **항상 구조체 끝에 블록으로 덧붙인다** —
중간 블록을 늘리면 뒤따르는 오프셋이 전부 밀려 기존 호스트가 깨진다.

### 2-4. C# / WPF

* `SwarmNative.cs` — `CNfpStrategyConfig`, `CLpSolverConfig`, `ExpectedAbiVersion = 10`
* `SwarmOptimizer.cs` — `NfpStrategySettings`, `LpSolverSettings`, `LpSolverBackend` enum
* `NestingConfigViewModel.cs` / `NestingConfigView.xaml` — 컨트롤 16개
  (전략 12 + 솔버 4), 툴팁에 근거 수치 포함
* `CallbackOptionsConfigTests.cs` — ABI v10 레이아웃 + 전 플래그 전달 테스트

---

## 3. 설계 결정 세 가지 (되돌리기 전에 읽을 것)

### 3-1. Clp / Xpress 를 빌드에 링크하지 않는다

둘 다 빌드 시점에 벤더 SDK 를 요구한다. Xpress 는 라이선스도 필요하다.
`Cargo.toml` 에 박으면 **SDK 없는 모든 환경에서 빌드가 깨진다.**

그래서 C ABI 를 `libloading` 으로 **런타임 동적 로딩**한다. 빌드는 어디서나 되고,
DLL 을 놓아 두면 그때부터 동작한다.

```
Clp     Windows: clp.dll     Linux: libClp.so     환경변수 SWARM_CLP_LIB
Xpress  Windows: xprs.dll    Linux: libxprs.so    환경변수 SWARM_XPRESS_LIB
```

없으면 `fallback` 설정에 따라 microlp 로 물러나거나 압축을 건너뛴다.

**QP 는 라이선스 없이도 실제로 풀린다** — `clarabel`(순수 Rust 내점법)이 있다.

### 3-2. LP 기본 경로는 손대지 않았다

`lp_compaction_pass_with_solver` 는 두 경로로 갈린다.

```
backend == Microlp && !uses_quadratic()  ->  기존 microlp 코드 그대로 (한 줄도 안 바뀜)
그 외                                    ->  LpProblem 으로 옮겨 담아 backend_for()
```

압축은 백트래킹·신뢰영역·마진이 전부 실측으로 맞춰진 코드다. 모델 표현을
일반화하면서 같은 날 함께 갈아엎으면, 회귀가 났을 때 원인이 "새 백엔드"인지
"새 모델 표현"인지 구분할 수 없다. **새 경로가 검증되면 합치는 것이 맞다** (§5-4).

### 3-3. 2차 항이 폭 변수 W 에도 걸린다

`LpProblem::quadratic` 은 모든 변수에 같은 계수를 거는 대각 형태다. 정규화의 본래
목적은 "같은 폭이면 덜 움직이는 해" 라 `dx, dy` 에만 걸고 싶지만, W 에 걸리는
`(mu/2)W²` 은 양의 정의역에서 **단조 증가**라 폭을 줄이는 방향을 강화할 뿐 뒤집지
않는다. 1차항이 이미 W 를 최소화하므로 방향은 안전하다. 다만 **상대 가중치는
바뀌므로 계수를 키울 때 A/B 로 확인할 것.**

---

## 4. 남은 작업 — 이것부터 하면 된다

### 4-1. 【필수】 전략 분기를 `build_best_initial_nest` 에 배선

**이게 유일하게 남은 배선이다.** 지금은 모듈과 설정이 전부 있는데 `nfp_lbf/search.rs`
가 그것들을 호출하지 않는다. 즉 **WPF 에서 체크박스를 켜도 아직 아무 일도 일어나지
않는다.** (이번에 고쳤던 것과 정확히 같은 종류의 결함이니, 배선 뒤 회귀 테스트를 꼭 넣을 것.)

대상: `swarm-core/src/nfp_lbf/search.rs` 의 `build_best_initial_nest`

```rust
// 1) 회전 집합 — 라이브러리를 만들기 "전" 에 정해야 한다
let rotations = if strategy.instance_rotations {
    nfp_strategy::rotations::instance_rotation_set(instance, strategy.continuous_rotation_step_deg)
} else {
    cfg.rotations.clone()
};
let library = PreparedLibrary::build(instance, &rotations, cfg.concave_simplify_tolerance)?;

// 2) 아이템별 회전표
let item_rotations = if strategy.instance_rotations {
    nfp_strategy::rotations::item_rotation_table(instance, &rotations)
} else {
    item_allowed_rotations(instance, &cfg.rotations)   // 기존 규칙
};

// 3) 전략이 하나라도 켜져 있으면 전략 경로, 아니면 기존 경로
if !strategy.any_enabled() {
    ... 기존 ordering 루프 그대로 ...
} else {
    let mut ctx = StrategyContext::new(&mut placer, strategy, item_rotations, Some(deadline));

    // ② 클러스터링 (전처리)
    let clusters = strategy.pairwise_clustering
        .then(|| cluster::build_clusters(&mut ctx, instance, Some(deadline)))
        .unwrap_or_default();

    // ⑥ best-fit 이면 ordering 을 쓰지 않고 매 단계 조각을 고른다
    // ⑤ 빔이면 place_sequence_beam, 아니면 그리디 1패스
    // ③ 은 SelectionRule::MaxContact / LeftBottom 선택으로 전달
    // ④ 는 ctx 가 options_for 안에서 알아서 반영한다 (set_pocket_first)

    // ⑦ 슬라이딩 (후처리)
    if strategy.sliding_compaction {
        pieces = sliding::compact(&mut ctx, &pieces, strategy.sliding_rounds).pieces;
    }
}
```

**주의점**

* `estimated_cost_factor()` 로 **ordering 수를 줄여** 전체 예산을 맞춰야 한다.
  빔 폭 4 + best-fit 12 면 배치 1회가 48배다. 50 ordering 을 그대로 돌리면 예산이 폭발한다.
* ⑤와 ⑥을 함께 켜면 비용이 곱해진다. 허용하되 로그로 알릴 것.
* 결과는 반드시 기존과 같은 `SequentialPlacement` 로 만들어 `materialize` 에 넘긴다
  (검증 경로를 우회하면 안 된다).

### 4-2. 【필수】 배선 회귀 테스트

`ffi_step.rs` 에 넣어 둔 `step_api_config_carries_nfp_and_escape_flags` 와 같은 방식으로,
`search.rs` 소스에 각 전략 모듈 진입점이 실제로 등장하는지 단정하는 테스트를 넣어라.
**배선이 빠지면 즉시 깨지도록.** 이번 세션의 최초 버그가 정확히 그 배선 누락이었다.

### 4-3. 【필수】 A/B 측정

`data/input/` 의 wsjk156 / albano / dagli / fu 로 전략별 밀도·시간을 재라.
기준선(전부 끔)은 이미 측정되어 있다 — `NESTING_ESCAPE_NFP_LBF_PLAN.md` §6-5-2.

| 인스턴스 | LBF 단독 | NFP 단독(현재) |
|---|---:|---:|
| wsjk156 (156조각) | 69.45% | 71.83% |
| albano (24) | 73.57% | 82.14% (회전 켬) |
| dagli (30) | 70.64% | 74.93% |
| fu (12) | 70.34% | 74.95% |

**측정 전 예측:** ①은 albano/dagli/fu 에서 크게(회전 선언이 `[0,180]` 이상),
wsjk156 에서는 0 (선언이 `[0]` 뿐). ②는 의류 데이터에서 가장 클 것으로 본다.
③은 탐침 예산이 높을 때만. ⑤⑥은 시간 대비 효과를 반드시 볼 것.

측정 결과는 `NESTING_ESCAPE_NFP_LBF_PLAN.md` 에 표로 남길 것.

### 4-4. 【권장】 LP 두 경로 통합

§3-2 의 이중 경로를 하나로 합친다. 조건: 일반 경로가 기본 경로와 **같은 결과**를
낸다는 것을 실제 인스턴스에서 확인한 뒤. `both_paths_agree_on_the_same_model`
테스트를 먼저 쓰고, 통과하면 기존 inline microlp 코드를 지운다.

---

## 4-완료. 4-1 ~ 4-3 결과 요약

| 항목 | 상태 | 결과 |
|---|---|---|
| 4-1 전략 분기 배선 | **완료** | `build_best_initial_nest` 에 ①~⑦. `to_config` 가 전략을 인자로 받아 빠뜨리면 컴파일이 안 된다 |
| 4-2 배선 회귀 테스트 | **완료** | `search.rs` + `ffi_step.rs` 두 곳. 배선이 끊기면 즉시 깨진다 |
| 4-3 A/B 측정 | **완료** | 최고 조합 **②+③+④+⑦ = 72.18%** (LBF 69.45% 대비 +2.74%p) |
| 4-4 LP 두 경로 통합 | 미착수 | 새 경로는 `lp_backends.rs` 5종으로 검증됨 — 이제 합칠 수 있다 |

Phase 감사로 고친 결함 4건(⑥ 58.95%→71.87%, ④ 효과0→71.85%, ③ −0.68%p→+0.25%p,
② 쌍0→13쌍)의 원인과 근거는 `NESTING_ESCAPE_NFP_LBF_PLAN.md` §6-5-3 참조.

---

## 5. 서브에이전트가 남긴 경고 (읽고 판단할 것)

전문은 `C:\Users\INSIGH~1\AppData\Local\Temp\claude\...\tasks\wu610ibm4.output`.
휘발되므로 **중요한 것만 여기 옮긴다.**

### 5-1. microlp 0.6 의 오판 (측정으로 확인됨)

"자유 변수 + 어떤 행에도 등장하지 않는 또 다른 자유 변수" 조합에서 **유계인 LP 를
`Err(Unbounded)` 로 잘못 판정**한다. 압축 모델의 `dx/dy` 는 신뢰 영역으로 양쪽이
잘려 있어 지금은 이 조합이 생기지 않는다. `microlp_backend.rs` 모듈 문서에 재현표가 있다.

무한대를 유한 상수로 클램프해 우회하지 **않았다** — 클램프하면 진짜 무계인 모델이
"아주 큰 해"로 조용히 성공해 모델 오류를 잡을 길이 사라진다.

### 5-2. microlp 시간 제한 판정이 정답을 버릴 수 있다

microlp 순수 LP 경로에서 `Solution` 이 돌아왔다면 **언제나 증명된 최적해**다
(데드라인에 걸리면 값 없이 `Interrupted` 로 온다). 따라서 사후
`elapsed > time_limit` 판정이 실제로 걸러 내는 것은 "틀렸을지 모르는 해"가 아니라
**예산보다 조금 늦게 도착한 정답**뿐이다. `Limit` 은 `is_usable()` 이 거절하므로,
예산이 짧으면(수십 ms) 정답을 쥐고도 압축 스텝을 건너뛰는 일이 반복될 수 있다.

**압축 성공률이 낮게 나오면 여기가 첫 번째 의심 지점이다.**

### 5-3. clarabel `AlmostSolved` 를 `Failed` 로 매핑했다

정확도가 떨어졌을 뿐 쓸 만한 해일 수 있다. 압축 성공률이 낮으면 `Limit` 으로
완화하는 것을 고려할 만하다.

### 5-4. Xpress 상수는 미검증

`LPSTATUS=190`, `LPOBJVAL=2001`, `OUTPUTLOG=8`, `MAXTIME=1001` 등은 문서 기억에
의존해 상수로 박아 두었고 코드에 "검증 필요" 주석이 있다. **실제 라이브러리로
한 번 확인해야 한다.**

### 5-5. wasm32 미검증

`clarabel` 을 wasm32 로 빌드해 본 적이 없다. `swarm-wasm` 빌드 시 확인 필요.

---

## 6. 빌드 · 테스트 명령

```bash
cargo build --release --features "simd,capmods,adaptive,structure-first,jni"
cargo test  -p swarm-core --release --features "simd,capmods,adaptive,structure-first" --lib
cargo test  --release --features "simd,capmods,adaptive,structure-first,jni" --lib
```

```bash
dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj -c Release
dotnet test  csharp/CAPlanetSwarmSample.WPF.Tests/CAPlanetSwarmSample.WPF.Tests.csproj -c Release
```

> WPF 프로젝트는 `target/release/capswarmcore_100.dll` 을 자동으로 복사한다.
> **Rust 를 먼저 빌드할 것.**

---

## 7. WPF 에서 확인하는 법

설정 패널 → `NFP 초기배치 전략` 절에 체크박스 7개와 노브 5개,
그 아래 `LP·QP 솔버` 절에 4개.

실행 로그에 네이티브 로그가 `[native:INFO]` 로 찍힌다 (이번에 추가한 로그 콜백).
전략을 켜면 다음이 보여야 한다 — **안 보이면 §4-1 배선이 아직 안 된 것이다.**

```
[native:INFO]   [NFP-LBF] 탐침 예산 4096/아이템, ordering 50개, 예산 10000 ms
[native:INFO]   [INIT] NFP-LBF 시드 채택: 폭 7325.050 -> 7082.270 (밀도 71.83%, ...)
```
