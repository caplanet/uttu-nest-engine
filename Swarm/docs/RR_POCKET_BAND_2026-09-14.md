# R&R 빈틈 순위 · 격자 해상도 — 실측 결과 (2026-09-14)

Swarm(`swarm-core`) 전용. AutoNest 원천은 대상이 아니다.

## 측정 조건

| 항목 | 값 |
|---|---|
| 마커 | `C:/Caplanet/AiWorks/InputData/여성WSjk_비교용-G_New_SwarmV1.json` (156조각 / 1524mm) |
| 단계 예산 | Explore 480s + Compress 60s + Polish 60s (**회차 606초**) |
| 겹침 | `piece_overlap_mm = 0.8` |
| 시드 | 1, 2, 3 |
| CPU | affinity 3코어(마스크 224) + `RAYON_NUM_THREADS=3`, **순차 1개씩** |
| DLL | `target/release/capswarmcore_100.dll` (2026-09-14 19:39 빌드) |
| 하네스 | `scripts/full_bench/run.ps1` + `arms/*.json` |
| 결과 | `C:/Users/mwlee/tmp/rr_runs/<arm>_s<seed>.summary.json` |
| 총 소요 | 15회 × 606초 + 초기화 ≈ **2시간 35분** (19:47~22:21) |

## 결과

| 시드 | 기준 | ② pocket | ②+④ fine | ④ gridalone | ③ nfp_reins |
|---|---|---|---|---|---|
| 1 | 80.357% | 80.102% | 79.670% | 80.066% | 79.831% |
| 2 | 78.816% | 79.850% | 79.911% | 79.964% | 79.925% |
| 3 | 79.635% | 80.045% | 79.637% | 79.627% | 79.794% |
| **평균** | **79.603%** | **79.999%** | **79.739%** | **79.886%** | **79.850%** |
| **Δpp** | — | **+0.396** | **+0.137** | **+0.283** | **+0.248** |
| **승** | — | **2/3** | **2/3** | 1/3 | **2/3** |

**기준선 시드 잡음: 1.541%p** (80.357 / 78.816 / 79.635). 모든 팔의 개선폭이 이보다 작다.

## ★ 결정적 관찰 — 네 팔이 «공통 설정» 을 공유하므로 팔 간 분리가 안 된다

네 팔 모두 `post_shrink=1, full_band=1` 을 공통으로 켠다. 그래서 **팔 간 Explore
결과가 소수점까지 동일**하다:

| 팔 | s1 explore | s2 explore | s3 explore |
|---|---|---|---|
| **기준선** | **6345.37** | **6480.05** | **6401.41** |
| ② pocket | 6414.86 | 6420.39 | 6413.13 |
| ②+④ fine | 6414.86 | 6420.39 | 6413.13 |
| ④ gridalone | 6414.86 | 6420.39 | 6413.13 |
| ③ nfp_reins | 6414.86 | 6420.39 | 6413.13 |

**기준선과는 다르고, 팔끼리는 완전히 같다.** 기준선 대비 Explore 차이:

| 시드 | 기준 explore | 팔 explore | 차이 (mm) |
|---|---|---|---|
| 1 | 6345.37 | 6414.86 | **+69.49 (기준선이 더 좋음)** |
| 2 | 6480.05 | 6420.39 | −59.66 (팔이 더 좋음) |
| 3 | 6401.41 | 6413.13 | +11.72 |

**이것이 의미하는 바:**

1. **팔 간 차이(②·④·③ 각각의 효과)는 Explore 에서 측정되지 않는다.** 네 팔의
   Explore 가 완전히 같으므로, Explore 단계에서 ②④③ 노브는 **아무 영향이 없다.**
2. 관측된 Δpp(0.137~0.396)는 전부 **Compress/Polish 단계**에서 나온 것이고,
   그 단계는 Explore 와 달리 **시드 의존 잡음이 큰** 구간이다.
3. `post_shrink=1, full_band=1` 공통 설정 자체의 효과(기준선 대비)는 **시드마다
   부호가 뒤집힌다**(+69.49 / −59.66 / +11.72). 즉 **이 설정도 잡음 안에 있다.**

**결론: 네 팔 전부 «효과 없음» 으로 판정한다.** 개선폭이 기준선 잡음(1.541pp)보다
작을 뿐 아니라, **Explore 단계에서 노브가 아예 작동하지 않는다는 것이 직접 확인**됐다.
Compress/Polish 에서의 Δpp 는 그 단계의 시드 잡음으로 설명된다.

## ★★ 확정 — R&R 제안이 100% 분리 루프로 되돌아간다

`--log` 검증(Explore 90초, seed 1, `rr_nfp_reins`, `C:/Users/mwlee/tmp/rr_verify/`)에서
R&R 제안을 **전량 집계**했다:

```
총 R&R 호출               : 49
requires_separation=true  : 49   (100%)
requires_separation=false :  0   (  0%)
strategy=Rightmost        : 49   (100%)
```

**R&R 이 제안한 배치가 단 한 번도 그대로 채택되지 않았다.** 49회 전부 겹침이 남아
분리 루프로 되돌아갔다. 그리고 매번 **loss 가 3~8배로 폭증**한다:

```
loss=21395.0723 -> 144498.6406   (6.75배)
loss=17720.5781 -> 143119.7031   (8.08배)
loss=32144.7910 -> 187065.9844   (5.82배)
loss=17541.7734 -> 184441.4688   (10.51배)
loss=23411.0508 -> 107229.5312   (4.58배)
```

**이것이 ②④ 가 Explore 결과에 전혀 안 닿는 이유의 직접 증거다.**

구조적으로 설명하면:
1. R&R 이 밴드에서 조각을 꺼내 **재삽입**한다 (`reinserted=52`)
2. 재삽입이 `requires_separation=true` → **겹침이 남은 상태**로 끝난다
3. 제안이 채택되지 않고 **분리 루프가 배치를 다시 만든다**
4. → 최종 배치를 결정하는 것은 **분리 루프**이고, R&R 의 밴드 선택이 아니다

즉 **`full_band=1` 로 52개를 부수고 재삽입한 결과가 «겹침 덩어리» 일 뿐**이고,
그걸 분리 루프가 처음부터 다시 푼다. **R&R 은 손해만 보고 이득은 없다** —
최소한 이 설정 조합(`full_band=1`, `min_clear_reinsert_ratio=0`)에서는.

### 왜 이 조합이 이렇게 되는가

`full_band=1` 은 `rnr_max_items` 상한(기본 40)과 무작위 truncate 를 건너뛰고
**밴드 안 이동 가능 조각을 전부** 파괴한다 → 52개. 그런데 재삽입은
`RECREATE_SAMPLE_CONFIG`(컨테이너 샘플 300개)로 자리를 찾으므로, **52개를 한 번에
넣을 만큼의 탐색을 하지 않는다.** 못 넣은 조각은 원위치 폴백 → **없던 겹침 생성**.

`min_clear_reinsert_ratio` 주석이 정확히 이걸 경고한다:
> 조밀해지면 대부분 폴백으로 떨어져 «재배열은 없고 겹침만 늘어난» 상태가 된다

**주의 — 실측은 이 주석의 진단과 다르다.** 가드를 켜도 폭증이 안 막혔다.
아래 «가드를 켜도 loss 폭증이 안 막힌다» 절 참조. 폴백이 아니라 **정상 착석인데도**
loss 가 폭증한다.

### 가드를 켜도 loss 폭증이 안 막힌다 — 가드가 «다른 것»을 재고 있다

`min_clear_reinsert_ratio=0.6` 으로 측정했으나 **loss 가 가드 없을 때와 소수점까지
동일**했다:

```
가드 없음: loss=21395.0723 -> 144498.6406
가드 0.6 : loss=21395.0723 -> 144498.6406   ← 완전히 같음
```

로그에 `[LNS] rearrange rejected` 가 **0건**이다. 가드가 한 번도 발동하지 않았다.
코드는 정확히 구현돼 있다(`lns.rs:614~634`):

```rust
let min_ratio = cfg.min_clear_reinsert_ratio;
if min_ratio > 0.0 && min_ratio.is_finite()
    && (n_clear_seats as f32) < min_ratio * n_ruined as f32
{
    info!("[LNS] rearrange rejected: ... rolling back");
    rollback_legacy_rnr(...);
    return 0;
}
```

**원인 — 가드가 재는 것과 loss 가 재는 것이 다르다.**

`n_clear_seats` 는 «충돌 없는 자리를 찾았는가» 만 센다(`lns.rs:602`):
```rust
let seat = reserved.or(nfp_searched).or(searched);
n_clear_seats += usize::from(seat.is_some());   // ← «자리 찾음» 여부만
let new_dt = seat.unwrap_or(cand.placement.d_transf);  // 못 찾으면 «원위치»
```

착석률은 60%를 넘으므로 가드가 통과된다. **그런데 loss 는 8배가 된다.**

**즉 폭증의 원인은 «폴백»이 아니라 «재배열 자체» 다.** 52개를 꺼내 각각 충돌 없이
다시 놓아도, **전체 배치의 국소 가중 loss(어려운 쌍의 충돌 근접도)는 크게 나빠질 수
있다.** 각 조각은 «합법»이지만 그 조합이 «나쁜» 것이다.

`min_clear_reinsert_ratio` 주석은 «대부분 폴백으로 떨어져» 를 원인으로 들었지만,
실측은 **폴백이 아니라 정상 착석인데도 loss 가 폭증**함을 보인다. 주석의 진단이
틀렸거나 불완전하다.

**따라서 가드는 loss 폭증을 막지 못한다.** 필요한 것은 «착석률» 이 아니라
**«재배열 후 loss 를 재는 판정»** 이다 — `post_shrink_rearrange` 경로
(`explore_mods.rs:876`)은 재배열 후 width 만 보고 loss 회복은 분리 루프에 맡긴다.
여기에 loss 가드를 추가하는 것이 다음 착수다.

### ★★ 근본 진단 — «재배열 자체» 가 항상 loss 를 폭증시킨다

파괴 규모를 바꿔가며 측정했다(Explore 60초, seed 1, `--log`):

| 설정 | 재배열 시도 | loss 가드 거부 | 파괴 조각수 | loss 폭증 |
|---|---|---|---|---|
| `full_band=1` + `max_loss_growth=2.0` | 43 | **43 (100%)** | 49~52 | x3.67 ~ x7.32 |
| `full_band=0` (상한 40) | 41 | 0 (가드 없음) | **40 (고정)** | **x6.41, x9.07** |

**`full_band` 를 꺼도 loss 폭증이 그대로다** (21395→137189, 24489→222065).
파괴 규모를 52 → 40 으로 줄여도 마찬가지다.

**즉 원인은 «대량 파괴» 가 아니라 «우측 밴드 재배열» 그 자체다.**

`Rightmost` 전략은 **우측 끝 밴드 = 현재 packing frontier** 를 통째로 재배열한다.
이 구간은 **가장 최근에 놓인, 즉 가장 «잘 맞춰진» 조각들**이다. 그걸 꺼내서
재삽입하면:

1. 좌측 우선 규칙(`RECREATE_X_MULT=10.0`)으로 **다시 왼쪽부터 채운다**
2. 그런데 좌측은 이미 조밀하다 → 조각들이 **원래 있던 자리보다 나쁜 곳**에 앉는다
3. 각 조각은 «충돌 없이» 앉으므로 `min_clear_reinsert_ratio` 는 통과
4. 결과적으로 **어려운 쌍들이 서로 가까워져** `local_weighted_loss` 가 폭증

`loss` 가드(2.0)를 켜면 **43회 전부 거부**되므로, `full_band` 계열 설정은
**loss 가드 없이는 순손해, 가드가 있으면 항상 no-op** 이다. 어느 쪽이든 이득이 없다.

### 그래서 «Rightmost + 재삽입» 조합 자체가 문제다

`post_shrink_rearrange` 는 `RuinStrategy::Rightmost` 를 **하드코딩**한다
(`explore_mods.rs:881`). 축소 직후 우측이 밀려났으니 그 구간을 다시 놓는다는
발상인데, 실측은 **그 재배열이 항상 손해**임을 보인다.

**다음 착수 후보:**

1. **`Rightmost` 대신 다른 전략을 쓴다.** `loss` 가드는 재배열 후 판정이므로,
   전략을 바꿔 «가드를 통과하는 재배열» 이 나오는지 본다. `LowDensity`·`Fragmented`
   는 «비어 있는 곳» 을 고르므로 frontier 를 건드리지 않는다.
2. **`recreate_bias` 를 바꾼다.** 기본 `(10, 1)` 은 좌측 우선이라 위 2번 문제가 생긴다.
   `(1, 10)`(아래 우선) 등으로 바꿔 «원래 자리 근처» 로 되돌아가는지 본다.
3. **재배열을 우측 밴드가 아니라 «밀려난 조각만» 으로 좁힌다.** 지금은 밴드에
   걸친 모든 이동 가능 조각(40~52개)을 꺼낸다. 실제로 밀려난 것은 소수일 수 있다.
4. **`max_loss_growth` 가드를 기본으로 켠다** — 최소한 손해를 막는다. 다만
   실측상 100% 거부이므로 **no-op 과 같다** → 3번 전까지는 켜도 의미가 없다.

### ★★★ 최종 진단 — 네 축 전부 검증 완료. 재배열 자체가 성립하지 않는다

네 가지 축을 모두 직접 측정했다(Explore 60초, seed 1, `--log`, `max_loss_growth=2.0`):

| 축 | 설정 | 재배열 시도 | 거부 | 통과율 | loss 배율 |
|---|---|---|---|---|---|
| **전략** | `Rightmost` (0) | 43 | **43** | **0%** | x3.7 ~ 7.3 |
| **전략** | `LowDensity` (1) | 44 | **44** | **0%** | x3.7 ~ 6.4 |
| **전략** | `Fragmented` (2) | 48 | **48** | **0%** | x3.4 ~ **15.8** |
| **편향** | `recreate_bias=(1,10)` 아래우선 | 48 | **48** | **0%** | x2.8 ~ 3.9 |
| **규모** | `full_band` 해제(상한 40) | 41 | (가드 없음) | — | x6.4, x9.1 |
| **착석률** | `min_clear_reinsert_ratio=0.6` | 45 | **0** | — | 가드 무력(발동 안 함) |

**네 축 전부 «재배열 제안을 살릴 수 없다» 로 수렴한다.** 편향을 아래 우선으로
바꾸면 배율이 **x6.75 → x3.90** 으로 개선되지만(그 방향이 맞다는 증거), 여전히
2배 기준을 못 넘는다.

**결론: `post_shrink_rearrange` 는 현재 구현으로는 순손해이거나 no-op 이다.**

- `arms/band_post.json` 기본(`min_clear_reinsert_ratio=0`, `max_loss_growth=0`) →
  **순손해**. 40~52개를 꺼내 다시 놓으면 loss 가 최소 2.8배, 최대 15.8배 폭증한다.
- 가드를 켜면 **100% 거부 = no-op**. 손해는 막히지만 이득도 없다.

이번 측정의 네 팔(②④③)이 전부 `post_shrink=1` 을 공통으로 켰으므로, **기준선 대비
Explore 차이(+69.49 / −59.66 / +11.72)의 상당 부분은 이 기능의 순손해**일 가능성이
있다. 즉 ②④③ 의 «효과 없음» 판정에는 **공통 설정의 손해가 섞여 있다.**

### 왜 이렇게 되는가 — 구조적 이유

`RECREATE_X_MULT=10.0` / `RECREATE_Y_MULT=1.0` 은 **좌측 우선** 재삽입이다
(`lns.rs:226`). 밴드에서 조각을 꺼내면 그 조각들은 **좌측의 빈 곳**부터 다시 앉는다.
그런데:

1. 좌측은 이미 조밀하다 — 빈 곳이 거의 없다
2. 꺼낸 조각은 «원래 있던 자리» 보다 나쁜 자리로 밀려난다
3. **어려운 쌍들이 서로 가까워진다** → `local_weighted_loss` 폭증
4. 각 조각은 «충돌 없이» 앉으므로 `min_clear_reinsert_ratio` 는 통과

편향을 아래 우선(`(1,10)`)으로 바꾸면 배율이 2배 가까이 개선된다 — **«밀려난 방향과
반대로 재삽입한다» 가 손해의 주 원인**임을 시사한다.

### 진짜 다음 착수

1. **(최우선) «원위치 복귀» 를 재탐색 후보에 넣는다.** 지금은 무조건 좌측부터
   재탐색한다(`cand.placement.d_transf` = 원위치는 **폴백으로만** 쓰인다). 원위치를
   후보에 포함하면 **최소 «제자리» 는 보장**되고, loss 악화가 원리적으로 0 이 된다.
   그러면 재배열이 «개선될 때만» 배치를 바꾸게 된다.
2. **편향을 더 조정한다.** `(1,10)` 이 x6.75→x3.90 을 만들었으므로, 밴드 밖 방향으로
   더 밀거나 중립(`(1,1)`)으로 두는 조합을 스윕한다. 2배 기준을 통과하는 조합이
   나오는지 확인.
3. **`max_loss_growth` 를 기본값으로 켠다.** 실측상 100% 거부 = no-op 이지만,
   최소한 손해는 막는다. 1·2번이 성공하면 그때 실제 이득이 난다.
4. ②④(후보 집합·격자)는 **채택 경로가 생긴 뒤에** 다시 본다. 지금은 R&R 제안이
   100% 되돌려지므로 그 뒤에 있는 노브가 관측될 수 없다.

### 실측 팔 정의 (검증용, Explore 60초 seed 1)

| 파일 | 설정 | 결과 |
|---|---|---|
| `rr_lossguard2.json` | `post_shrink+full_band+max_loss_growth=2.0` | 43 시도 / 43 거부 |
| `rr_nofullband.json` | `post_shrink` (full_band 해제) | 41 시도 / 40개 파괴 / 폭증 유지 |
| `rr_guard60.json` | `post_shrink+full_band+min_clear_reinsert_ratio=0.6` | 45 R&R / 0 rejected (가드 무력) |
| `rr_lossguard15.json` | `post_shrink+full_band+max_loss_growth=1.5` | (미실행 — 2.0 에서 이미 100% 거부) |

### `requires_separation=false` 비율이 핵심 지표다

앞으로 R&R 개선은 **이 비율을 먼저 올려야** 한다. 0% 인 한 어떤 밴드 선택·격자
개선도 Explore 에 도달하지 못한다. 로그 한 줄로 측정 가능하므로 다음 시도의
**1차 지표**로 삼는다. 실측값(Explore 90초, seed 1):

| 설정 | R&R 호출 | `req_sep=true` | `req_sep=false` | 채택률 |
|---|---|---|---|---|
| `post_shrink+full_band+nfp_reinsertion` | 49 | 49 | 0 | **0%** |
| `post_shrink+full_band+guard0.6` | 45 | 45 | 0 | **0%** |

## ③ nfp_reinsertion — 배선 검증 완료 (실제로 돈다)

`--log` 를 켠 별도 검증(Explore 90초, seed 1, `C:/Users/mwlee/tmp/rr_verify/`):

```
[LNS] NFP reinsertion cache available: true
[LNS] ruin&recreate: 52 items in Rightmost band [5832.8, 7113.2] (strip w: 7113.2)
[LNS] proposal: strategy=Rightmost, reinserted=52, loss=21395.0723->144498.6406, elapsed_ms=134, requires_separation=true
```

전 경로가 실제로 실행된다:
```
optimizer/mod.rs:291  expl_separator.configure_reinsertion(nfp_integration)
        ↓  nfp_reinsertion=true → reinsertion_pending = Some(config)
explore_mods.rs:878   lns::ruin_and_recreate(sep, ...)
        ↓
lns.rs:160~161        reinsertion_pending.take() → NfpReinsertionCache 생성
        ↓
lns.rs:169~171        cache = reinsertion_nfp.take() → with_nfp(...) 전달
```

**문서 `SWARM_RR_HOLE_SWAP_REVIEW_2026-09-08.md:41~45` 의 «연결 누락» 지적은 낡았다.**
문서 작성 후 고쳐진 상태다(현재 `configure_reinsertion` 은 14곳에서 호출된다).

## ★ 부수 발견 — `full_band=1` 이 loss 를 7배로 폭증시킨다

```
loss=21395.0723 -> 144498.6406   (6.75배)
loss=17720.5781 -> 143119.7031   (8.08배)
loss=19101.2207 -> 82973.4844    (4.34배)
```

`band_rearrange.full_band=1` 이 **한 밴드에서 52개 조각을 파괴**한다(`rnr_max_items`
상한과 무작위 truncate 를 건너뛰므로). 이는 `min_clear_reinsert_ratio` 주석이
경고한 바로 그 현상이다:

> 조밀해지면 대부분 폴백으로 떨어져 «재배열은 없고 겹침만 늘어난» 상태가 된다 —
> 실측에서 축소 직후 총 loss 가 **4배**로 뛰었다.

기록된 4배보다 **더 크다(최대 8배)**. `full_band=1` + `min_clear_reinsert_ratio=0`
조합은 **착석률 가드가 꺼진 상태로 대량 파괴**를 하는 것이므로, 이 조합 자체를
재검토할 가치가 있다. 기존 `arms/band_guard60.json`·`band_guard85.json` 이
가드를 켠 팔인데, **그 결과가 저장소에 없다.**

## 판정

| # | 항목 | 판정 |
|---|---|---|
| ① | `post_shrink_rearrange` | **이미 구현·배선 완료.** 팔도 존재했고 결과만 유실. 이번 측정의 네 팔이 모두 이 설정을 켠 상태 → **기준선 대비 효과는 잡음 안**(부호가 시드마다 뒤집힘) |
| ② | 빈틈 순위 → 밴드 선택 | **구현·측정 완료. 효과 없음** — Explore 결과가 ② 를 안 켠 팔과 완전히 동일. Δpp +0.396 은 Compress/Polish 잡음 |
| ③ | `nfp_reinsertion` | **배선 정상 확인**(로그로 검증). Δpp +0.248 도 잡음 이하 — 캐시가 돌아도 채택 경로가 없어 결과에 안 닿음 |
| ④ | 격자 해상도 | **구현·측정 완료. 효과 없음** — Δpp +0.283(④ 단독), +0.137(②+④). 잡음 이하 |

**네 팔 전부 «효과 없음».** 노브는 **기본 꺼짐 상태로 남긴다** — 배포 경로
(`--best`) 무손상이고, 더 긴 예산이나 다른 마커에서 재측정할 여지를 남긴다.

## 다음 착수 — «채택 경로» 부터

이번 측정이 남긴 진짜 단서는 **Δpp 가 아니라 `requires_separation=true`** 다.

```
[LNS] proposal: strategy=Rightmost, reinserted=52, loss=...->..., requires_separation=true
```

R&R 이 52개를 부수고 재삽입해도 **항상 겹침이 남아** 분리 루프로 되돌아간다. 그래서
R&R 이 만든 배치가 최종 배치가 되지 못하고, **뒤따르는 분리가 최종 배치를 결정**한다.
밴드 선택 규칙(②)도 격자 해상도(④)도 그 뒤에서 안 보인다.

**순서를 바꿔야 한다:**

1. **`requires_separation=false` 로 끝나는 R&R 제안의 비율을 측정한다.** 이 비율이
   낮으면 어떤 R&R 개선도 Explore 에 도달할 수 없다. 먼저 이 수치를 뽑아야 한다.
2. **`full_band=1` + `min_clear_reinsert_ratio=0` 조합을 재검토한다.** 로그에서
   loss 가 **최대 8.08배**(21395 → 144498)로 폭증한다. 기록된 4배보다 크다.
   `band_guard60.json`/`band_guard85.json` 이 가드를 켠 팔인데 **결과가 유실**됐다 —
   가드 60/85 를 먼저 측정해 «대량 파괴가 실제로 이득인가» 를 확정해야 한다.
3. **`post_shrink` 트리거의 실행 횟수를 로그로 센다.** 주석은 «1~6회 → 13~14회» 로
   는다고 했는데, 이번 로그에서 실제로 몇 회 돌았는지 세어 확인한다.
4. 그 다음에야 ②④(후보 집합·격자)를 다시 본다. **채택 경로가 없으면 의미가 없다.**

## 유실된 기존 실측 결과 (재측정 필요)

저장소에 `sweep.json`/`summary.json` 이 남아 있지 않다. 다음 팔들은 **arms 정의만
있고 결과가 없다**:

`band_post.json`, `band_full.json`, `band_full_rnr1.json`, `band_raster.json`,
`band_raster_nfp.json`, `band_gls.json`, `band_guard60.json`, `band_guard85.json`,
`band_wide.json`, `band_ydown.json`, `band_edge01.json`

특히 **`band_guard60`/`band_guard85`**(착석률 가드)와 **`band_full_rnr1`**(period=1)
은 위 «다음 착수» 1·2번과 직결되므로 우선 재측정 대상이다.

## 구현 요약 (부록)

### 추가한 노브 3개 (전부 기본 꺼짐)

`RuinRecreateConfig` (`swarm-core/src/config.rs`) + `CBandRearrangeConfig` (ABI v18 tail):

| 노브 | 기본값 | 의미 |
|---|---|---|
| `pocket_band_candidates: bool` / `pocket_band: i32` | `false` / `0` | 밴드 후보에 실제 포켓 좌측 경계 추가 |
| `pocket_band_max: usize` / `i32` | `24` / `0`→24 | 추가할 포켓 개수 상한 |
| `pocket_grid_cell_scale: f32` | `1.0` / `0.0`→1.0 | 빈틈 격자 셀 크기 배율 |

C# 바인딩(`csharp/Natives/SwarmNative.cs`)에도 동일 3필드 — `FullBench` 의
`SetPath` 가 리플렉션으로 `CSwarmConfig` 필드를 찾으므로 C# 쪽 추가가 필수다.

### 구현한 헬퍼 (`swarm-core/src/optimizer/lns.rs`)

```rust
fn band_candidates(sep, cfg, band_w, term) -> Vec<f32>
fn pocket_grid_cell(layout, cell_scale) -> f32
fn pocket_inventory(sep, cell_scale, term) -> Option<Vec<Pocket>>
```

두 후보 생성부(231행 `with_nfp` 경로, 1202행 `profile` 경로)를 헬퍼 호출로 교체.

### 근본 원인 (`N_DENSITY_BANDS = 8`)

```rust
let step = span / (N_DENSITY_BANDS - 1) as f32;   // span / 7
```
1524mm 원단에서 간격 약 **190mm** → 조각 하나 크기 빈틈은 두 후보 사이에 빠진다.

### 부수적으로 고친 기존 오류 2건 (이전부터 존재, `cargo test` 컴파일 불가 상태였음)

1. `swarm-core/src/nfp_lbf/score.rs:485` — `PlacementScoreWeights` 에
   `score_first`·`length_tie_mm` 누락
2. `swarm-core/tests/parallel_optimization.rs:215` — `RuinRecreateConfig` 에
   `..DEFAULT_SWARM_CONFIG.expl_cfg.rnr` 누락

## 검증

- `cargo build --release --features "simd capmods adaptive structure-first jni"` → 성공
- `cargo test --release --features "simd capmods adaptive structure-first" -p swarm-core`
  → **780 passed / 0 failed** + 통합 테스트 전부 통과
- 스모크: `overrides` 에 `band_rearrange.pocket_band=1` 전달 확인
- ③: `[LNS] NFP reinsertion cache available: true` 확인
