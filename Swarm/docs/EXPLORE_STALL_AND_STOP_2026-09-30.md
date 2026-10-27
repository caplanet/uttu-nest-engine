# Explore 정체 구간 · Stop 응답 — 실측과 수정 (2026-09-30)

마커: `여성WSjk_비교용-G_New_Swarm (2).json` (156조각 / 1524mm, 회전 `[0]`,
sha256 `42417e04…`). 이전 문서들의 `…SwarmV1.json` 과는 **다른 파일**이다.

조건: 겹침 0.8mm, Explore 240s + Compress 30s + Polish 30s, 시드 1~3, 3코어
(`run.ps1` affinity 224 / 28, `RAYON_NUM_THREADS=3`). 결과 파일은 세션
스크래치(`…/scratchpad/runs/<arm>/`)에 있고 저장소에는 요약만 남긴다.

## 1. 사용자가 본 «몇 분씩 개선이 없는 구간» 의 정체

시드 3 로그(`--log`, Explore 240s):

```
6460.594 feasible (78.740%)                       ← 90초 시점
[SEP] … total 37.s   → unable to reach feasibility (width 6448.639, min loss 2.92K)
[EXPL] disrupting by swapping two large items (id: 110 <-> 109)
[SEP] … total 89.s   → feasible solution found! (width 6448.639)   ← 216초 시점
```

**126초 동안 화면은 6460mm 에서 멈춰 있었다.** 내용은 «실패한 분리 37초 + 무작위
큰 조각 교환 1회 + 같은 폭 재분리 89초» 다. 시드 1도 벽 근처(6391~6438mm)에서
분리 한 번이 17~58초다. 240초 안에 분리 실패는 시드당 1~2회, R&R·폭 완화·진동은
**0회** 다 — 즉 4분 예산에서 정체 구간은 «분리 하나가 오래 걸리는 것» 이지
교란 루프가 도는 것이 아니다.

## 2. 시도한 것과 결과 — 전부 기각, 노브는 기본 꺼짐

| 팔 | 내용 | s1 최종 | s2 최종 | s3 최종 | 평균 |
|---|---|---:|---:|---:|---:|
| `base18` | 새 DLL, 노브 전부 0 (= 종전 동작) | 79.628% | 78.447% | 79.483% | **79.186%** |
| `restart60` | 분리 안 정체 재출발 창 60라운드 | 78.182% | 77.718% | 78.620% | 78.173% |
| `port4` | 실패 후 교란 후보 4개 × 2초 탐침 | 79.683% | 78.337% | 78.947% | 78.989% |
| `cap20_port4` | 분리 1회 20초 상한(막힌 폭 건너뛰기) + 위 | 79.193% | 77.988% | 78.949% | 78.710% |

시드 잡음(base18 s1~s3 표준편차 ≈ 0.6%p)을 감안해도:

* **재출발 창은 명백히 나쁘다** (−1.0%p, 3/3 시드). 로그에서 «개선이 strike 롤백
  직후에만 몰린다» 는 패턴으로 세운 가설이었는데, 짧은 창마다 인컴번트로 되돌리면
  GLS 가중치 학습이 끊겨 초반부터 느려진다 (30초 시점에 이미 40mm 뒤진다).
* **막힌 폭 건너뛰기(시간 상한)는 손해다** (−0.5%p). 벽 근처의 긴 분리는 대부분
  결국 성공하는 분리다. 20초에 끊고 다른 후보로 넘어가면 그 성공을 버리는 셈이고,
  대안 후보가 더 빨리 푸는 경우는 관측되지 않았다.
* **교란 포트폴리오는 잡음 안** (−0.2%p). 240초 안에 실패가 1~2회뿐이라 발동
  기회 자체가 적다.

건너뛰기 질문에 대한 답: **가능하지만 이 마커·예산에서는 이득이 없다.** 정체
구간에서 실제로 일어나는 일은 «풀리는 중인 분리» 이고, 그것을 끊으면 폭이 남는다.
탐색 튜닝으로는 4분 안의 결과를 못 올린다는 9월 11일 문서의 결론이 이 마커에서도
반복된다.

## 3. 그래도 남긴 것 (ABI v18 `search` 꼬리 블록, 전부 0 = v17 동작)

| 필드 | 뜻 |
|---|---|
| `sep_restart_window` | Explore 분리 안 정체 재출발 창(라운드) |
| `sep_iter_floor` | capmods `iter_no_imprv_limit` 하한(아이템 수 × 2.5) 덮어쓰기 |
| `cmpr_restart_window` | Compress 분리 재출발 창 |
| `rnr_original_candidate` | R&R 재삽입에서 원위치를 후보로 비교 (§4) |
| `escape_window_absolute` | 탈출 파이프라인 정체 창을 `window_floor` 절대값으로 |
| `sep_time_cap_ms` | Explore 분리 1회 벽시계 상한 |
| `portfolio_k` / `probe_budget_ms` | 실패 후 교란 후보 K개 × 탐침 예산 |

`CSwarmConfig` 는 916+32 = **952바이트**, `SWARM_ABI_VERSION = 18`. C#
바인딩(`SwarmNative.cs`)과 FullBench 는 함께 갱신했다. 기존 `ffi.rs` 의 레이아웃
테스트는 9월 14일에 `CBandRearrangeConfig` 가 10필드로 늘어난 뒤 갱신되지 않아
**이미 깨져 있었다**(20 vs 40바이트) — 실제 오프셋(812/852/856/872/916)으로 고쳤다.

### 탈출 파이프라인이 한 번도 발동하지 않았던 이유

`escape/pipeline.rs` 의 정체 창은 `max(100, 3 × 조각 수)` **분리 호출** 이고
156조각이면 468회다. 벽에서 분리 한 번이 수십 초이므로 예산 안에 도달할 수 없다.
9월 11일의 «escape 전면 교체 −0.258%p» 는 사실 발동 없는 기준선을 잰 것이다.
`escape_window_absolute=1` + `escape.window_floor=N` 으로 창을 절대값으로 줄일 수
있게 했다. 다만 §1 의 이유로 이 예산에서는 측정하지 않았다.

## 4. R&R 재삽입 «원위치 후보» (`rnr_original_candidate`)

9월 14일 문서는 «착석률 60% 이상인데 loss 가 3~8배» 를 «재배열 자체» 탓으로 봤다.
코드를 보면 다른 설명이 선다: 자리를 못 찾은 조각은 regret `-inf` 로 **마지막에**
원위치로 돌아가는데, 그 원위치는 먼저 앉은 조각이 이미 차지했을 수 있다. 나머지
40% 가 그렇게 깊게 겹친다.

켜면 (1) 자리를 못 찾은 조각을 **먼저** 원위치에 앉히고, (2) 찾은 자리와 원위치를
같은 LBF 점수로 비교해 낮은 쪽을 쓴다. 단위 테스트
`original_candidate_reinsertion_never_worsens_more_than_the_legacy_path` (fu.json,
시드 3개): 종전 경로가 loss 를 357→406 으로 늘리는 시드에서 원위치 후보는 357 을
유지한다. 240초 예산에서는 R&R 이 0회라 수율 측정은 하지 않았다 — 480초 이상에서
`arms/rnr_orig.json` 으로 잴 것.

## 5. Stop 응답 (`tests/stop_latency_marker.rs`, `SWARM_MARKER` 로 마커 지정)

| 단계 | 수정 전 | 수정 후 | 원인 / 수정 |
|---|---:|---:|---|
| Init | **중단 불가** (구성 끝까지 대기, 6~20초) | 0.8~5.8초 | 핸들이 생기기 전이라 `swarm_request_stop` 이 닿지 않았다. 전역 취소 `swarm_cancel_init()` 추가; LBF 구성 루프·NFP-LBF 시드·prewarm·Init 단계 경계에서 확인. 남는 지연은 jagua `import_instance`(단순화, 2코어 5초 / 8코어 2.3초)로 협조 취소가 불가능한 구간이다 |
| Explore | 0.3초 | 0.3초 | 문제 없음 |
| Compress | 0.01초 | 0.01초 | 문제 없음 |
| Polish | **3.3~3.5초** | 0.03~0.1초 | LP 압축의 microlp 가 데드라인을 **1000 pivot 마다** 검사한다. 이 마커의 모델(수천 개 접촉 행)은 pivot 하나가 ms 단위라 50ms 조각이 3~7초로 늘어났다. `third_party/microlp` 로 벤더링해 검사 간격을 32 로 낮췄다(`[patch.crates-io]`). 진행 로그는 종전 1000 간격 유지 |

Polish 의 `[LP] slow stage: … took N ms` 로그(500ms 초과 시)를 남겨 두었다 — 현장에서
Stop 지연의 원인을 바로 볼 수 있다.

WPF: `StopAsync` 가 Init 중이면 `CancelInit()` 을 부르고, `RunAsync` 는 오류 코드
2002 를 «중단됨» 으로 표시한다.

## 6. 재현

```powershell
# 기준선 / 팔 (DLL 은 target/release 또는 복사본)
$env:PYTHONIOENCODING='utf-8'
python scripts/full_bench/sweep.py --arm base18 --seeds 1 2 3 --out <out> --marker <marker> --explore 240 --compress 30 --polish 30
python scripts/full_bench/sweep.py --arm port4 --overrides scripts/full_bench/arms/port4.json --seeds 1 2 3 --out <out> --marker <marker> --explore 240 --compress 30 --polish 30

# Stop 응답
$env:SWARM_MARKER='<marker>'
cargo test --release --features "simd,capmods,adaptive,structure-first,jni" --test stop_latency_marker -- --ignored --nocapture --test-threads=1
```

`sweep.py` 는 Windows 콘솔(cp949)에서 대체 문자를 만나도 죽지 않게 `errors="replace"`
를 켰다 (이전에는 시드 3에서 `UnicodeEncodeError` 로 중단됐다).

## 7. 네이티브 라이브러리 이름 변경 (2026-09-30)

Rust cdylib 출력 이름을 `capswarmcore_100` → **`cpswcore`** 로 바꿨다 (`Cargo.toml` `[lib] name`).
크레이트 식별자도 함께 바뀌므로 통합 테스트의 `use capswarmcore_100::…` 는 `use cpswcore::…` 다.
C# `SwarmNative.DllName`, WPF csproj 의 복사 경로, FullBench/PGO/벤치 스크립트, CMake 의
파일명을 모두 바꿨다. 이전 문서들에 남은 `capswarmcore_100.dll` 은 역사적 기록이다.
