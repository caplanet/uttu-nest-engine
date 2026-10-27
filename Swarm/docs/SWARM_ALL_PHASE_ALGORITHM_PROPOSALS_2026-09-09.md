# Swarm 전체 단계의 추가 알고리즘·개선 방법 검토

2026-09-09 · 검토 대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`의 현재 작업 소스

**먼저 권하는 방향은 기존 계산의 재사용, Init의 제약 보존과 후보 선택 개선, Polish의 소수 조각 공동 최적화다. 이진 탐색은 목표 제안·국소 접촉 정밀화에 한정하고, Beam은 기존 구현을 활용한 짧은 구간 탐색부터 시험하는 것이 적절하다.**

이 문서의 본문은 최초 소스 검토와 제안 기록이다. 해당 검토 때는 Rust·WPF·설정·DLL을 변경하거나 새로운 성능 실험을 하지 않았다. 이후 요청한 우선순위 1·2·3·4·5·9·10의 구현·검증은 [별도 구현 보고서](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_PRIORITIES_IMPLEMENTATION_2026-09-09.md)에 기록한다. Uttu.Nest와 fable 소스는 읽지 않았다. 아래의 예상 이득은 가설이며, 속도나 수율 개선을 보장하는 수치가 아니다. Config를 끄거나 설정값만 바꾸는 것으로 개선안을 대신하지 않는다.

최초 검토 기준에는 **노력·시간 기반 shrink, 실패 목표 backoff, 조기 인증 콜백**이 함께 적용돼 있었다. 이후 실제 사용에서 손해가 크다는 사용자 요청에 따라 **노력·시간 기반 shrink와 쉬운 목표 재시도는 롤백**했고, 조기 인증 콜백은 유지했다. 본문에서 이를 기준선으로 삼은 실험 제안은 당시 기록으로 읽어야 한다.

## 1. 먼저 판단해야 할 것

이득은 세 가지를 구분해서 판단해야 한다.

1. **첫 결과까지 시간:** Init과 전처리가 빠르고 완전한 배치를 빨리 확보하는가.
2. **같은 시간의 수율:** 전체 실행 시간이 같을 때 마지막 인증 원단 길이가 더 짧은가.
3. **결과 전달 지연:** 이미 찾은 좋은 배치를 사용자가 더 빨리 볼 수 있는가.

후보 평가 수가 늘거나 Init 결과가 좋아져도 최종 수율이 좋아진다는 보장은 없다. 조기 콜백은 주로 3번의 개선이며, 자체적으로 분리 라운드의 승자 결정이나 다음 축소를 앞당기지는 않는다.

직전의 두 기능 결합 비교는 WSjk·OPS2 각각 20초, seed 2개였다. WSjk 평균 길이는 **26.977mm 증가**, OPS2는 **9.282mm 감소**했다. 따라서 현재 shrink 정책을 다른 단계에 그대로 확대하거나, 더 복잡한 탐색을 항상 유리하다고 가정하지 않는다. 이 결과는 이번에 재측정한 값이 아니며 shrink와 콜백의 기여를 분리한 결과도 아니다. [직전 구현·비교 보고서](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_EXPLORE_OPTIMIZATION_REVIEW_2026-09-09.md:469)

## 2. 현재 단계와 이미 있는 알고리즘

코드가 존재하는 것과 현재 실행에서 선택되는 것은 다르다. 사용자가 지금 실행 중인 설정을 조사한 것은 아니므로 아래 표의 선택 경로가 모두 동시에 실행된다는 뜻은 아니다.

| 단계·경로 | 현재 있는 방법 | 추가 제안에서 구분할 점 |
|---|---|---|
| 입력·형상 준비 | 간격/겹침 오프셋, 면적 비율 단순화 또는 mm 최대 이탈 단순화, surrogate 생성 | mm 모드는 면적 비율 단순화를 대체한다. 두 번 연속 단순화하는 구조가 아님 |
| Legacy Init | LBF, Raster 후보, 선택적 Column Draft/refplace/NFP-LBF | 여러 엔진이 이미 있음. 엔진 신설보다 공통 incumbent·예산·제약 연결이 중요 |
| V2 Init | lane별 LBF/Raster, hint/frozen, rigid PlaceGroups | Legacy의 선택적 Init 경로가 그대로 모두 연결된 것은 아님 |
| 별도 Initial phase | N02C/Sorted 구성, GRASP, escape, shrink, compact | Legacy 전용 API. 기존 Init과 이름이 비슷하지만 별도 실행 단계 |
| Structure P0/P1 | profile·motif 생성과 선택, 여러 구성 시작, item/motif 후보 경쟁 | 일반 LBF와 별도 파이프라인. item beam은 여러 세대 배치 Beam과 다름 |
| Explore | 적응형 shrink, 실패 풀, R&R/Swap, ALNS 선택, 진동·완화 복구 | 현재 기본 검색 외에 NFP ordering·Escape·Structure 경로가 따로 있음 |
| Separator | GLS, 샘플링·좌표 하강, surrogate/SIMD 충돌 비용, 동기 워커, 원본 보정 | 조기 인증 콜백은 adaptive Explore 연결. 모든 대체 Explore가 동일한 정책을 쓰지는 않음 |
| Compress | 시간/실패 기반 축소, 무작위 seam와 우측 형상 경계 후보, 분리·인증 | 우측 경계 후보는 이미 추가됨 |
| Polish | LP, Slide, Hole Fill, pass 사이 인증 checkpoint | LP 접촉 병렬화·방향별 제약 중복 제거·변 묶음 제외도 이미 있음 |
| Reclaim | Compress 재진입, R&R+Explore, Escape | 일반 optimizer·Legacy FFI 연결. Grouped Polish와 Structure P5에 똑같이 붙지는 않음 |
| 최종 인증·내보내기 | 공개 pose로 CDE 재구축, 원본 겹침 검사, 인증/JSON 캐시, 최선 보존 | 이미 캐시·SIMD·조건부 병렬화가 있음. 이를 새로 추가한다고 제안하면 중복 |

주요 진입 근거: [입력 준비](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/util/io.rs:181), [Legacy Init](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:1925), [V2 lane 생성](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/grouped_engine.rs:2653), [별도 Initial](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:2216), [Structure 단계](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/structure/pipeline.rs:41), [Explore 선택](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:2403).

Swarm의 원 논문도 원단 길이 최적화를 반복적인 배치 가능성 문제로 바꾸어 푸는 구조를 설명한다. 다만 현재 Swarm에는 추가된 원본 겹침·잠금·그룹 계약이 있으므로 논문의 무충돌 조건을 그대로 대입하면 안 된다. [Swarm 원 논문, 2026-02 개정판](https://arxiv.org/html/2509.13329v3)

## 3. 권장 우선순위

‘위험’은 주로 같은 시간의 수율 악화, 메모리 증가, 구현 난도를 의미한다. 정확성 인증은 모든 제안에서 유지한다.

| 순서 | 제안 | 주 대상 | 기대하는 이득 | 비용·위험 | 권장 판단 |
|---:|---|---|---|---|---|
| 1 | 실제 실행 분기와 비용 추정 일치 | 선택적 NFP Init | 불필요한 ordering 제한 제거 | 낮음 | 확인된 불일치부터 작은 수정 후보 |
| 2 | 회전 bbox·고정 형상 계수 재사용 | Init·Explore·Separator | 검색 의미를 유지하며 준비 비용 감소 | 낮음~중간 | 범용 첫 실험 |
| 3 | 원본 검사에 첫 위반 반환 경로 | 모든 인증 경계 | 실패 후보의 남은 정밀 검사 절약 | 낮음~중간 | 위반 후보 비중이 높을 때 유망 |
| 4 | full instance/lane 형상 준비 공유 | Init·그룹 배치 | 반복 import·단순화 비용 감소 | 중간 | 반복 형상·V2에서 먼저 측정 |
| 5 | 기존 인증해를 보존하는 Init 후보 경쟁·부분 개선 | Init·Initial | 완료율·초기 길이 개선 | 중간~높음 | warm start·짧은 예산부터 |
| 6 | 대안이 적은 조각 우선 + 짧은 Beam | 초기배치 | 나중에 막히는 배치 선택 감소 | 중간~높음 | 깊이·폭을 제한한 실험 |
| 7 | 진행 중인 실패의 연장/복구 판단 | Explore | 유망한 분리 진행을 너무 일찍 버리는 문제 완화 | 중간 | 현재 shrink와 독립 비교 |
| 8 | 다양한 소수 배치 보존·확장 | Explore | 비슷한 실패 반복 감소 | 중간~높음 | 풀 다양성부터, 이후 작은 Beam |
| 9 | 강체 unit 단위 LP | Polish | 그룹이 있어도 동시 압축 수행 | 중간~높음 | 그룹 사용이 많으면 우선순위 상승 |
| 10 | 우측 소수 조각의 공동 pocket 배정 | Polish | 단일 donor 탐욕 배치의 한계 보완 | 높음 | 수율 개선용 유력 실험 |
| 11 | LP 접촉·모델 증분 재사용 | Polish | 반복 모델 구성·풀이 비용 감소 | 높음 | LP 시간 비중이 높을 때 |
| 12 | Compress/Reclaim의 비용·성과 기반 선택 | Compress·마무리 | 잔여시간의 낭비 감소 | 중간~높음 | 앞선 변경 이후 분리 시험 |

첫 실험 묶음도 한꺼번에 합치지 않는다. **속도 목적은 1~4, 새 수율 기회는 5~6·9~10**이 중심이다. 7~8·12는 탐색 정책을 바꾸므로 특히 보수적으로 평가한다.

## 4. 공통 계산과 Init 전처리

### 4.1 회전별 경계와 고정 계수를 재사용

현재 `UniformRectSampler`는 생성할 때 도형을 복제하고 허용 회전별 bbox를 계산한다. 한 검색에서도 focused/container sampler를 각각 만든다. 이웃이 바뀌더라도 같은 도형·같은 회전의 원점 bbox는 변하지 않는다. 또한 형상 penalty의 convex-hull 면적 제곱근은 배치 위치와 무관하다. [sampler](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/uniform_sampler.rs:41), [한 검색의 두 sampler](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/search.rs:341), [형상 penalty](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/mod.rs:28)

제안은 도형별 허용 회전 bbox와 `sqrt(hull_area)`를 불변 준비 자료로 저장하는 것이다. 매번 달라지는 탐색 영역과의 교차, 위치, GLS 가중치는 기존대로 계산한다. 모든 조각 쌍의 거대한 표부터 만들 필요는 없다. 연속 회전은 이산 회전과 같은 유한 테이블로 처리할 수 없으므로 기존 계산 또는 정확한 각도 키의 작은 캐시가 필요하다.

계산 순서·반올림·상한 판정이 바뀌지 않게 하고, 고정 반복에서 pose·loss·RNG가 같은지 먼저 확인해야 한다. 형상이 교체되거나 단순화·허용 회전이 바뀌면 자료도 교체한다. **SIMD 폭을 늘리기 전에 계산 호출 자체를 줄이는 개선**이다.

### 4.2 Init 형상 준비 공유와 병렬화

V2는 full instance를 import한 뒤 lane별로 외부 item을 복제해 다시 import한다. mm 단순화도 현재 item loop 안에서 순차 수행한다. 따라서 동일한 형상의 offset·단순화·surrogate 결과를 같은 실행 안에서 공유할 여지가 있다. [full import](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/grouped_engine.rs:191), [lane 재import](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/grouped_engine.rs:2669), [mm 처리 loop](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/util/io.rs:207)

먼저 full instance의 불변 형상 자료를 lane의 local ID에 연결하는 방식을 검토한다. ID·OrgID·수량·회전 허용 조건·그룹은 개별 배치의 정보이므로 합치지 않는다. strip 크기와 local/global ID 매핑도 lane별로 유지한다. 단순히 instance 전체를 얕게 복제하는 작업으로 끝나지 않는다.

독립 item의 형상 준비는 병렬화 후보지만 작은 형상에서는 스케줄링 비용이 더 클 수 있다. item index 순으로 결과를 합치고 실패 처리·중지를 동일하게 유지해야 한다. mm 경로 최적화는 mm 단순화를 사용한 실행에만 직접 이득이 있으며, 사용자의 `PolySimpl=0.01` 실행에서 같은 효과가 난다고 단정하지 않는다.

NFP 내부에는 이미 형상 중복 제거, 회전별 준비, Arc 공유, 병렬 prewarm, miss 시 계산이 있다. 새 공통 캐시는 그것과 중복으로 같은 자료를 쌓지 않아야 한다. 현재 NFP 준비는 양자화된 형상 해시를 사용하므로 이를 **원본 정밀 인증의 동일 형상 증명으로 그대로 재사용하면 안 된다.** [PreparedLibrary](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/cache.rs:981), [NFP prewarm](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/cache.rs:1258)

### 4.3 실제 사용 전략 기준의 비용 산정

확인된 작은 불일치가 있다. best-fit과 Beam이 함께 지정되면 배치는 best-fit 분기 하나를 실행하지만, 비용 추정은 두 배율을 곱한다. 그 추정값은 ordering 수를 줄이는 데 쓰인다. 현재 설명의 ‘둘을 함께 실행하니 비용이 곱해진다’는 전제가 실제 분기와 맞지 않는다. [실행 분기](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_strategy/pipeline.rs:236), [비용 추정](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_strategy/config.rs:327), [ordering 축소](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/search.rs:321)

실제로 선택된 전략 조합 기준으로 비용을 산정하도록 정리할 가치가 있다. 사용자가 지정한 ordering 수와 총시간 상한은 유지한다. 선택적 NFP 경로의 문제이며, NFP를 사용하지 않는 실행을 빠르게 하는 변경은 아니다. 이번에는 수정하지 않았다.

## 5. 초기배치 품질을 개선할 방법

### 5.1 기존 인증해를 출발점으로 하는 제한된 후보 경쟁

현재 Legacy Init은 LBF 이후 선택적 생성기를 실행한다. NFP-LBF는 더 짧을 때 채택하지만 실험용 refplace에는 기존 해와 비교하지 않고 대체하는 경로가 있다. 각 생성기가 자신의 준비와 시간 예산을 가지므로 단순히 엔진을 추가하면 Init 지연이 커진다. [Legacy 생성·교체](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:1938), [refplace 대체](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:2004), [NFP 예산](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/search.rs:281)

제안 구조는 다음과 같다.

1. 완전한 LBF 또는 warm-start 인증해를 먼저 확보하고 보존한다.
2. 허용된 Init 시간 안에서 저비용의 다른 ordering/기존 생성기 후보부터 시험한다.
3. 준비·구성·보정·인증 비용을 모두 그 시간에 포함한다.
4. 원본 정책을 통과한 완전한 후보만 비교한다. 시간 만료의 부분 배치는 공개 incumbent를 대신하지 못한다.

부분 배치 후보의 길이를 완전한 배치와 직접 비교해서는 안 된다. Init 자체 시간과 전체 Init→최종 시간이 모두 중요하다. 단계별 API에 주어진 Explore/Compress 시간을 임의로 빼앗거나 옮기자는 제안도 아니다.

특히 별도 N06 Initial은 기존 해·frozen·그룹을 인자로 받지 않고 새 GRASP 배치를 만든다. FFI가 잠금 위치 변경을 걸러 내더라도, 애초에 채택할 수 없는 후보를 만드는 비용이 발생할 수 있다. **기존 인증해를 받고 movable 일부나 미배치 조각만 개선하는 생성기**로 확장하는 편이 warm start에 적합하다. [Initial FFI](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:2305), [N06 생성](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/initial/attach.rs:4156), [LBF 기존 배치 재사용 기반](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/lbf.rs:387)

### 5.2 Regret: 나중에 놓을 곳이 없어지는 조각을 먼저 배치

두 조각이 같은 좋은 자리를 원한다고 하자. A는 다른 곳에도 거의 비슷하게 들어가지만, B는 그 자리를 놓치면 원단 길이가 크게 늘어난다. 현재의 최선 위치만 보는 선택보다 **차선 위치에서 얼마나 나빠지는지**를 함께 보면 B를 먼저 놓을 수 있다.

현재 NFP best-fit은 면적 티어 안에서 각 조각의 최선 위치와 현재 비용으로 선택한다. R&R의 profile 재삽입에는 이미 최선·차선 차이를 사용하는 regret 계산이 있으므로, Init에 제한적으로 재사용하는 것이 적절하다. [Init best-fit](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_strategy/select.rs:795), [기존 regret 재삽입](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/lns.rs:1273)

모든 조각을 매번 다시 평가하지 말고 기존 면적 티어·후보 창을 유지한다. 사실상 같은 위치 두 개를 최선·차선으로 세지 않고 서로 다른 pocket/배치 대안으로 구분해야 한다. 그룹은 멤버별이 아니라 이동 단위별로 평가한다. Regret 원리는 ALNS 원 연구에도 있으나, 운송 문제의 성능 결과를 Swarm 수율 향상 근거로 가져오지는 않는다. [Ropke·Pisinger 연구](https://di.ku.dk/forskning/Publikationer/tekniske_rapporter/tekniske-rapporter-2004/04-13.pdf)

### 5.3 실제 윤곽 맞물림으로 상위 후보만 재평가

현재 NFP 전략의 contact/waste에는 bbox 기반 점수가 쓰인다. 오목한 의류 형상의 맞물림은 사각형 경계만으로 표현되지 않는다. Structure의 경계 profile 접촉 점수를 재사용해 **길이가 비슷한 상위 소수 후보만** 다시 평가할 여지가 있다. [NFP 점수](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_strategy/mod.rs:209), [profile 접촉](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/structure/profile.rs:395)

접촉 길이가 늘었다고 최종 원단 길이가 줄어드는 것은 아니다. pocket 입구를 막을 수도 있으므로 다음 몇 조각의 배치 가능성과 함께 봐야 한다. 모든 후보에 원본 Clipper 연산을 추가하는 방식은 권하지 않는다.

## 6. 이진 탐색: 어느 문제에 유리한가

### 6.1 결론표

| 적용 대상 | 판단 | 이유 |
|---|---|---|
| 고정 회전·고정 이웃에서 국소 접촉 좌표 정밀화 | 적합, 이미 여러 곳에 있음 | 안전한 bracket 안에서 경계 정밀화 비용을 줄일 수 있음 |
| 조각의 전체 X/Y 위치·회전·순서 선택 | 일반 이진 탐색 부적합 | 충돌 여부와 배치 점수가 단조적이지 않음 |
| Init을 길이마다 처음부터 다시 실행하는 전역 탐색 | 우선순위 낮음 | 생성·준비를 반복하고 실패 판정이 불완전함 |
| Explore에서 성공 길이와 도전 길이의 중간 목표 제안 | 제한적 실험 가치 있음 | 과한 축소 뒤 더 쉬운 목표를 빨리 찾을 가능성 |
| Separator 실패를 불가능 하한으로 확정하는 이진 탐색 | 비추천 | 시간제한 실패는 배치 불가능 증명이 아님 |

### 6.2 ‘해가 존재하는가’와 ‘이번에 찾았는가’는 다르다

다른 제약을 고정하고 직사각 strip 길이만 늘리면, 6,500mm에서 유효한 배치를 더 긴 strip에도 놓을 수 있다. 이런 조건에서 **해의 존재는 길이에 대해 단조적**이다.

하지만 Separator는 제한시간 안에 해를 찾는 휴리스틱이다. 6,480mm에서 실패해도 다른 조합이나 더 긴 시간에서는 성공할 수 있다. 심지어 더 긴 목표에서 출발 배치와 난수 때문에 실패할 수도 있다. 따라서 ‘이번에 실패했으니 6,480mm 이하는 전부 버린다’는 이진 탐색은 잘못된 가지치기를 할 수 있다.

안전하게 실험할 수 있는 것은 다음 방식이다.

```text
6,500mm: 인증된 해 있음
6,480mm: 이번 시간 안에는 못 찾음 — 불가능 판정 아님
6,490mm: 다음 목표 후보로 시험
실패한 6,480mm도 다른 배치·여유 시간에서 재도전 가능
```

검토 당시의 backoff는 `best − (best − failed) × decay`였으며 기본 0.6에서 위 상황에 6,488mm를 제안했다. **이 노력·시간 기반 shrink와 쉬운 목표 재시도는 2026-09-09 사용자의 실제 사용 판단에 따라 롤백했다.** 현재 실행 경로에는 적용되지 않는다. 중간 목표 제안의 장단점은 별도 실험 주제로 남으며, 작은 성공이 늘어도 총 축소량이 줄 수 있다. [롤백과 현재 동작](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_EXPLORE_OPTIMIZATION_REVIEW_2026-09-09.md)

N06 Initial도 이미 축소 실패 때 step을 절반으로 줄이고 일부 재확장하는 로직을 갖고 있다. 이를 일반적인 ‘실패 하한 확정 이진 탐색’과 혼동하면 안 된다. [N06 shrink](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/initial/attach.rs:4253)

면적 하한을 쓸 때도 주의해야 한다. `piece_overlap_mm > 0`이고 잠금 예외가 있으므로 **원본 면적 합÷원단 폭을 무조건 배치 불가능 하한으로 사용하면 안 된다.** 현재 허용 계약에서도 성립하는 하한인지 별도 증명이 필요하다.

### 6.3 위치의 이진 탐색은 이미 있지만 전역 최적을 보장하지 않는다

Init의 `slide_axis`, Structure의 `refine_axis`, exact-NFP refinement와 Polish Slide에는 접촉을 좁혀 가는 경로가 이미 있다. [Init slide](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/initial/attach.rs:1279), [Structure refinement](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/structure/profile.rs:790), [exact refinement](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/exact_nfp.rs:118)

이동축 위에서는 `안전 → 충돌 → 안전` 구간이 나올 수 있다. 단순 이진 탐색은 떨어져 있는 좋은 빈자리를 모두 찾지 못한다. 기존 slide도 최종 안전한 위치를 보존하는 도구이며, 전체 구간의 최대 이동량이나 첫 접촉점을 항상 구한다는 증명은 아니다. 네스팅은 보통 이동 경로가 아닌 최종 위치를 평가하므로 멀리 떨어진 안전 위치로 옮겼다는 사실 자체가 잘못은 아니다.

추가할 만한 방법은 **이미 준비된 NFP에서 축 방향의 허용 구간을 구해 좋은 구간을 선택하고, 그 경계를 정밀화**하는 것이다. NFP가 없는 일반 검색마다 새로 NFP를 생성하면 준비 비용이 더 클 수 있으므로 기존 NFP 경로에서 먼저 시험한다.

## 7. Beam Search: 이미 있는 부분과 새로 이익이 날 부분

### 7.1 세 가지를 구분해야 한다

| 현재 구조 | 실제 의미 | 여러 세대 배치 Beam인가 |
|---|---|---|
| NFP `place_sequence_beam` | 복수 부분배치를 남기고 다음 조각 배치로 확장 | 예 |
| Structure `item_beam` | 다음 item/motif 후보 여럿을 평가하고 하나 확정 | 아니오, 후보 경쟁의 폭 |
| 동기 Separator 워커 | 같은 기준 배치에서 각각 이동한 뒤 승자 하나로 합류 | 아니오 |
| Explore 실패 풀 | 여러 실패 배치를 보관하고 하나를 선택해 교란 | archive이며 정규 Beam 확장은 아님 |

근거: [NFP Beam](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_strategy/beam.rs:483), [Structure item 후보](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/lbf.rs:677), [동기 워커](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:751), [실패 풀](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore_mods.rs:914)

### 7.2 초기배치에는 짧은 구간 Beam이 더 유망하다

예를 들어 앞쪽 큰 조각 두 개의 선택으로 다음 조각이 들어갈 긴 빈자리가 막힐 수 있다. 즉시 좋아 보이는 배치 하나만 확정하지 않고, 그 주변에서 몇 단계 더 놓아 본 뒤 비교하면 이런 실수를 줄일 가능성이 있다.

권하는 실험은 **위험한 선택 지점에서만 깊이 2~4·폭 2~4의 부분배치**를 유지하는 것이다. 수치는 최적값이 아니라 작은 실험 시작점이다. 대상은 큰 조각의 초기 조합, pocket 입구, 경쟁하는 motif 등으로 제한한다. 나머지는 기존 greedy/profile 경로를 사용한다.

분기 후 제한된 후속 배치를 해 보면서 현재 길이, 남은 조각의 삽입 여유, 막힌 빈공간을 비교한다. 거의 같은 좌표의 후보를 많이 남기기보다 다른 회전·접촉 이웃·pocket 사용·motif를 보존하는 편이 중요하다. 정확한 중복 제거는 이미 있어도 구조적 다양성까지 보장하는 것은 아니다.

기존 NFP Beam에는 greedy 경로 보존과 중복 제거가 있다. 그러나 시간 만료 시 부분배치가 남을 수 있으므로 **실시간 전체 실행에서 greedy보다 절대 나쁘지 않다는 보장은 아니다.** Beam 폭이 커지면 ordering 완료 수나 후속 Explore 시간이 줄 수 있다. 현재 주석의 과거 수율·시간 표는 이번 실행 조건의 새 실측 결과로 사용하지 않았다. [Beam 종료·보존](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_strategy/beam.rs:473)

### 7.3 Explore Beam은 실패 풀의 다양성 개선 다음에

현재 실패 풀은 loss 순위 상위 32개를 남기고 하나를 뽑는다. 처음부터 전체 Explore를 여러 개 복제하기보다, **같은 목표 길이에서 낮은 loss와 서로 다른 배치 구조를 가진 2~4개**를 남겨 제한된 분리 작업 후 다시 선별하는 작은 Beam이 자연스러운 실험이다.

인증된 최선 해는 이 실패 Beam 밖에서 항상 보존한다. 서로 다른 길이의 raw/weighted loss를 그대로 비교하지 않는다. GLS 가중치가 다르면 같은 길이에서도 weighted loss의 직접 비교가 부당할 수 있으므로 공통 평가 기준을 사용해야 한다.

각 가지에는 배치·strip·tracker·GLS 가중치·RNG의 일관된 상태가 필요하다. 현재 copy-on-write는 읽기 공유를 돕지만, 모든 가지가 쓰기 시작하면 복사 비용이 다시 발생한다. 가지마다 Separator 워커 풀을 추가하면 CPU 예산도 늘어난다. 먼저 기존 총 워커 수 안에서 가지별 시간 배분을 시험해야 한다.

**초기배치의 짧은 Beam을 Explore 전역 Beam보다 먼저 권한다.** 초기 선택을 조금 더 보아야 하는 문제와 수천 번의 분리 상태를 여러 갈래 유지하는 문제는 비용 규모가 다르다. Explore에서는 정확한 중복 제거·다양성 슬롯만 추가한 대조군도 필요하다.

## 8. Explore·Separator의 추가 개선

### 8.1 진행을 보이는 실패에만 제한적인 연장 기회

현재 shrink 제어는 성공 여부·소요시간을 보고, 시도 만료 뒤 인증 배치에서 더 쉬운 길이로 돌아간다. 분리 loss가 방금까지 빠르게 줄고 있었는지에 대한 입력은 없다. [시도 예산](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore_mods.rs:779), [관측 상태](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore_mods/shrink_control.rs:64), [복구 분기](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore_mods.rs:940)

Separator가 시작 loss·최저 loss·마지막 감소 시점·실제 종료 원인을 작은 결과 구조로 반환하게 하고, 계속 개선 중인 시도에만 한정된 연장 기회를 주는 방안이다. 정체한 시도는 backoff한다. 어려운 실패 배치를 조금 넓혀 이어가는 후보도 비교할 수 있지만 매번 모든 복구를 병렬 실행하지는 않는다.

`min_l: 83.1 K` 같은 절대값은 입력 크기에 따라 의미가 달라 단독 기준으로 부적절하다. 같은 목표 안의 상대 감소, 소비한 시간·평가량, 최근 변화가 더 적합하다. 원본 인증 성공 여부가 최종 판단이다. 이 부분이 WSjk 회귀의 원인이라는 뜻은 아니며, 원인 후보로 분리 실험해야 한다.

### 8.2 같은 pose 평가 결과의 작은 캐시

현재 `BestSamples`는 평가 뒤 후보를 보관하면서 중복을 처리한다. 좌표 하강의 clamp 등에서 정확히 같은 pose가 반복되면 한 검색 안에서 결과를 재사용할 여지가 있다. [검색](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/search.rs:355), [후보 보관](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/best_samples.rs:26)

다만 bounded evaluator의 `Invalid`는 충돌 비용의 정확한 값이 아니라 **이번 상한을 넘었으므로 계산을 중단했다**는 뜻일 수 있다. 이를 다른 상한에서 무조건 재사용하면 선택이 바뀐다. 완전 평가 결과부터 캐시하거나, 동일/더 엄격한 상한에서만 제한 재사용해야 한다. [평가 상한](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/sep_evaluator.rs:156)

캐시 수명은 layout·GLS가 고정된 evaluator 안으로 제한한다. 검색을 넘겨 공유하려면 pose뿐 아니라 layout·strip·가중치·그룹·형상 정책의 변경을 모두 알아야 한다. 논리 후보 횟수와 실제 계산 횟수를 구분해 기존 평가 한도가 무심코 늘어나지 않게 한다. 균일 실수 후보의 중복이 낮으면 캐시 조회가 더 비쌀 수 있으므로 적중률부터 측정한다.

### 8.3 동기 워커별 비용 편차부터 계측

현재 동기식을 유지한다. 워커별 실제 계산시간·평가 수·이동 unit 수·zero-loss 시각을 라운드 경계에서 모으면 무엇 때문에 마지막 워커가 오래 걸리는지 구분할 수 있다. 합산 worker load 시간과 라운드 wall-time은 의미가 다르므로 둘의 차이를 대기시간이라고 계산하면 안 된다. [현재 계측](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:759)

편차가 확인된 뒤에만 일부 워커의 후보 생성 역할을 다르게 하는 동기식 조합을 시험한다. 이는 이미 평가한 후보의 단순 병렬 가속이 아니라 탐색 알고리즘 변경이다. 후보마다 clock·atomic을 추가하거나 별도 워커를 무조건 늘리는 방식은 피한다.

## 9. Compress·Polish·Reclaim

### 9.1 강체 그룹 단위 LP

현재는 PlaceGroups가 있으면 개별 조각 LP를 건너뛴다. 안전한 처리지만 주변의 독립 조각도 LP 동시 압축 기회를 잃는다. [LP 제외](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:230)

그룹 대표 하나에 `dx,dy` 변수 하나씩을 두고 모든 멤버를 같은 이동 변수에 연결하면 그룹 형태를 유지하면서 압축할 수 있다. 첫 구현은 회전을 고정한 평행이동으로 한정한다. 외부 조각과의 멤버별 접촉·원단 경계는 검사하고, 내부 상대위치는 고정한다. 잠금 멤버가 있으면 전체 unit을 고정하는 현재 규칙을 유지한다.

그룹 없는 데이터에는 이 기능 자체의 추가 이득이 없다. 그룹 LP를 지원한다고 기존 개별 LP의 분기를 그냥 켜서는 안 된다. 같은 변수로 모이는 제약 계수를 합치는 처리와 그룹 내부 제약 제외도 필요하다.

### 9.2 우측 소수 donor의 공동 pocket 배정

현재 Hole Fill은 우측 donor부터 여러 위치를 비교하지만 한 조각씩 확정한다. A가 유일한 큰 pocket을 쓰면 더 큰 B가 이동하지 못해 전체 길이가 그대로일 수 있다. [현재 donor 선택·확정](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/holes.rs:484)

우측 길이를 결정하는 2~4개 movable unit과 소수 pocket 후보에 한정해 작은 배정 문제를 푸는 방법을 제안한다. 후보끼리의 충돌도 포함한 branch-and-bound 또는 짧은 Beam을 사용할 수 있다. 단순 이분 매칭만으로는 두 후보의 실제 형상 충돌까지 보장하지 못한다.

1차 목적은 전체 `max(x_max)` 감소, 동점에서는 다음으로 큰 우측 끝들의 감소다. 단일 조각의 이동량이나 pocket 면적만으로 개선을 인정하지 않는다. 기존 R&R의 밴드 제거·재삽입과 달리 **서로 경쟁하는 몇 개의 자리 배정을 함께 결정**한다는 차이가 있다. 후보 수·깊이·시간을 제한하고 최종 공동 이동을 하나의 인증된 transaction으로 채택한다.

Raster/scanline·다중 해상도는 이런 후보를 싸게 생성하는 연구 기반이 있다. Swarm에는 이미 Raster가 있으므로 새 격자 엔진을 통째로 추가하기보다 후보 배정에 재사용한다. 격자 결과는 원본 허용 판정을 대신하지 않는다. [Raster 좌표 하강 연구](https://www.sciencedirect.com/science/article/pii/S0377221722002582), [다중 해상도 연구](https://www.sciencedirect.com/science/article/pii/S2405896315003675)

### 9.3 LP 접촉과 모델을 증분 갱신

현재 LP는 반복마다 item 목록·접촉 행·모델·backend를 구성한다. 접촉 생성 병렬화와 변 묶음 제외는 이미 있지만 반복 사이의 solver 상태 재사용 API는 없다. [반복 모델](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:319), [backend 풀이](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:395), [backend 인터페이스](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/lp_solver/mod.rs:407)

먼저 stable item/edge ID와 pose revision으로 이웃·접촉 후보를 재사용한다. 그 다음에 backend가 지원하는 모델 갱신·basis/초기점 재사용을 별도 구현한다. 현재 API에 단순 옵션 하나를 더하면 모든 backend가 warm start를 지원하는 것은 아니다.

지난 접촉만 남기면 안 된다. trust 영역이 커지거나 형상이 움직이면 **새로 충돌할 수 있는 쌍**도 찾아야 한다. 이동 가능 범위만큼 확장한 bbox로 후보 집합을 갱신하고, 변경이 크면 전체 재구축으로 돌아간다. 마지막 실제 CDE·원본 인증과 backtracking은 유지한다. 작은 LP에서는 관리 비용이 더 클 수 있다.

### 9.4 Compress와 Reclaim의 시간 사용 개선

Compress에는 이미 무작위 seam와 우측 경계 후보가 있다. 추가할 것은 seam 종류·실제 이동 unit 수·초기 충돌량·분리 비용으로 **축소 크기와 분리 예산을 함께 선택**하는 방법이다. 현재 Explore shrink의 효과가 엇갈렸으므로 같은 제어기를 바로 복사하지 않는다. [Compress 축소](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/compress.rs:45), [기존 seam](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/compress.rs:118)

Reclaim은 선택한 mode와 남은 시간의 일정 비율을 사용한다. 기존 Compress/R&R/Escape 중 직전 성과·남은 시간에 맞는 작업을 고르는 작은 선택기를 검토할 수 있다. 보상은 실행 횟수가 아닌 비용 대비 인증된 길이 감소로 한다. 후속 처리와 최종 인증 시간을 먼저 남겨야 한다. [Reclaim](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/reclaim.rs:168)

Compress 재진입은 첫 무개선 cycle 후 끝나지만 이것은 보수적 정책이지 수렴 증명이 아니다. 분할 위치가 확률적이라는 현재 코드와 달리 Config 주석 일부는 결정론적이라고 설명한다. 주석과 실제 동작을 정리할 필요가 있다. [정책](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/reclaim.rs:168), [Config 주석](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/config.rs:663)

## 10. 원본 인증·최종 결과에서의 개선

### 10.1 첫 위반만 필요한 검사와 전체 목록 검사를 분리

현재 원본 guard는 pose·쌍 revision 캐시, X 정렬 sweep, bbox SIMD, 조건부 쌍 병렬화를 갖고 있다. 그런데 성공 여부만 필요한 `validate()`도 전체 위반 목록을 만든 다음 첫 위반만 사용한다. [전체 inspect](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap.rs:127), [검증 호출](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap.rs:575)

`첫 위반 발견 시 반환` 경로를 추가하면 탈락이 확정된 후보의 나머지 Clipper 검사를 줄일 수 있다. 보정·진단에는 기존 전체 목록 경로를 유지한다. **성공으로 인정할 때는 필요한 모든 쌍을 검사**하므로 검증 강도를 낮추는 방식이 아니다.

현재 병렬 precompute는 최대 128쌍의 일부를 미리 계산한다. 첫 위반을 빨리 찾는 경로에서 이 작업을 모두 기다리면 이득이 줄 수 있으므로, 기존 캐시·저렴한 위반 후보부터 확인한 뒤 병렬 묶음을 시작하는 방식도 비교해야 한다. 캐시 통계·오류 전달 순서와 취소 처리는 유지한다. [현재 병렬 예열](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap.rs:222)

### 10.2 원본 sweep의 반복 준비 절감

pose와 쌍 계산 결과는 이미 재사용하지만 매 검사마다 active key 집합과 X 정렬 목록을 다시 만든다. 바뀐 조각이 적은 보정·Polish에서는 정렬된 목록을 유지하고 변경 항목만 갱신할 여지가 있다. 대다수 조각이 바뀌면 기존 전체 정렬이 더 단순하고 빠를 수 있으므로 전환 기준을 측정해야 한다.

이는 bounding volume으로 불가능한 충돌 쌍을 먼저 제외하는 broad-phase 방법의 시간적 재사용이다. 일반 격자는 크기가 매우 다른 의류 조각에서 후보가 많이 생길 수 있어, 현재 sweep을 무조건 격자로 교체하는 것은 권하지 않는다. [현재 순서 생성](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap.rs:161), [NVIDIA broad-phase 설명](https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-32-broad-phase-collision-detection-cuda)

### 10.3 더 큰 변경인 증분 인증은 후순위

현재 최종 인증은 공개할 `d_transf`에서 도형·CDE를 다시 만들어 내부 중복 상태가 오래됐어도 잘못 통과하지 않게 한다. 성공 인증 캐시는 입력·pose·정책 전체를 비교한다. 이 안전성을 단순 dirty flag 하나로 대체하면 안 된다. [재구축 이유](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/structure/pipeline.rs:1348), [성공 인증 cache](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/structure/certification.rs:19)

장기적으로는 인증된 불변 snapshot과 변경 unit 목록을 함께 관리해 변경 쌍만 검사할 수 있다. 하지만 모든 mutation·rollback·group·strip·형상 변경을 추적할 내부 계약이 선행되어야 한다. 외부 JSON/hint 유입과 불명확한 변경은 전체 재구축 경로로 검증한다. 먼저 10.1~10.2의 작은 개선을 시험하는 편이 낫다.

‘원본 겹침 깊이’의 판정도 그대로 유지한다. 현재는 교집합에 허용량 절반에 따른 침식을 적용하는 정책이다. 이를 bbox 겹침, 최소 이동 거리, 면적 감소로 바꿔 놓고 같은 mm 허용이라고 주장하면 안 된다. [원본 판정](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap.rs:61)

## 11. SIMD·병렬화의 적합한 위치

| 작업 | 적합성 | 적용 조건 |
|---|---|---|
| 독립 형상 단순화·surrogate 준비 | 병렬화 후보 | 크기 충분할 때, 입력 순서대로 병합, 동일 결과·중지 계약 |
| 회전 bbox·고정 계수 | 재사용 우선 | 작은 계산을 매번 병렬화하지 않음 |
| 같은 snapshot의 Beam 자식 후보 생성 | 병렬화 후보 | 기존 총 워커 예산 안, 가지별 상태 독립 |
| pocket 후보끼리 bbox 충돌 | SIMD·병렬화 적합 | 제안 필터이며 공동 최종 인증 유지 |
| LP 변경 쌍 접촉 생성 | 기존 병렬화 재사용 | 행 순서·방향·실제 제약 동등성 확인 |
| 원본 좌표 변환·bbox 필터 | 이미 SIMD | 더 넓은 벡터보다 반복 변환·후보 수 감소부터 |
| 원본 Clipper 쌍 | 이미 조건부 병렬 | 작은 쌍은 직렬, 소유자 캐시는 join 후 갱신 |
| 개별 후보 내부 중첩 병렬화 | 우선순위 낮음 | 이미 worker/lane 병렬이며 오버헤드·과다 스레드 위험 |
| 콜백·최선 결과 채택 | 직렬 소유 유지 | 워커에서 UI callback 직접 병렬 호출 금지 |

V2 SizeRegions에는 이미 바깥 lane 병렬 실행과 적응형 slice가 있다. 여기에 Beam·Separator·원본 검증용 풀을 각각 무제한 추가해서는 안 된다. **새 병렬 단계도 전체 CPU 예산을 공유**해야 한다. [Grouped scheduler](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/grouped_engine.rs:1242), [기존 lane slice 조절](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/grouped_engine.rs:1286), [원본 SIMD](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap/kernels.rs:7)

## 12. 지금 우선하지 않을 방법

- **전면 GPU 전환:** CPU의 CDE·Clipper·snapshot과 왕복하고 후보가 분기하는 현재 구조에서는 구현·전송 비용이 크다. 반복 대량의 독립 후보 병목이 측정되기 전에는 권하지 않는다.
- **전체 마커 MILP/전수 탐색:** 많은 복잡한 비볼록 조각의 모든 조합을 직접 풀기보다, 9.2 같은 작은 공동 배정 문제에 한정하는 편이 현실적이다.
- **새 유전 알고리즘·대형 신경망 선택기:** 이미 GRASP·ALNS·Escape·Swarm 계열이 있어 추가 구조보다 기존 후보의 완료율·비용·다양성 문제를 먼저 확인해야 한다.
- **큰 Beam·많은 NFP probes·모든 엔진 동시 실행:** 작업량 증가는 개선 증거가 아니다. 준비와 인증을 포함한 총시간으로 비교해야 한다.
- **첫 성공 워커가 다른 워커를 취소하는 구조:** 현재 동기식 유지 방향과 다르고 winner·RNG·탐색 기회를 바꾼다. 기존 조기 콜백과도 다른 변경이다.
- **격자·단순화·Clipper 정밀도를 낮춰 통과시키기:** 현재 허용 계약을 변경한다. 계산 절약과 정확성 완화를 혼동하지 않는다.
- **직선·유지 정점 보호 정책 재도입:** 사용자 요청으로 롤백한 고비용 정책이다. 이번 후보 제안은 그 정책을 되살리는 것이 아니다.

## 13. 검증·실험 설계

### 13.1 정확성 기준

모든 채택 결과는 다음을 유지해야 한다.

- 완전한 조각 수, ID·OrgID 대응, 유한 좌표, 허용 회전.
- 잠금 pose 보존. 잠금끼리의 겹침·잠금 원단 이탈 허용 예외 유지.
- movable과 잠금 사이, movable끼리의 충돌은 현재 정책대로 검사.
- rigid 그룹의 상대좌표·회전, size-region 경계·간격 유지.
- `piece_overlap_mm`와 원본 인증 정책 유지. raw loss=0만으로 공개하지 않음.
- Stop·오류·시간 만료 시 마지막 인증 incumbent 보존. Final/GetSolution/내보내기 역행 방지.

검색 의미를 보존하는 캐시·준비 최적화는 고정 반복에서 pose·loss·RNG·논리 평가 횟수까지 대조한다. 정책을 바꾸는 Beam·regret·예산 제어는 같은 pose가 나올 필요가 없지만 인증 기준은 같아야 한다.

### 13.2 이진 탐색·Beam의 분리 비교

| 시험 | 변경하는 부분 | 대조 기준 |
|---|---|---|
| 목표 중간값 | 현재 0.6 backoff의 목표 제안만 중간값 방식으로 변경 | 나머지 shrink·콜백·총예산 동일 |
| Init 제한 Beam | 선택이 어려운 구간만 짧은 Beam | 기존 greedy/profile, 기존 전체 NFP Beam을 별도 대조 |
| Explore 다양성 | 실패 풀 일부에 다른 pose 구조 보존 | 현재 loss 순위 풀 |
| Explore 작은 Beam | 같은 길이의 소수 후보를 제한적으로 확장 | 현재 정책, Beam=1 wrapper, Beam=2/4 |
| 결합 | 단독 변경에서 의미 있는 이득이 확인된 것만 결합 | 각 단독 변형과 비교 |

현재 적용 버전을 기준선으로 고정한다. shrink 적용 전 버전도 보조 대조군으로 둘 수 있지만, 콜백을 같이 제거해 놓고 shrink 하나의 효과라고 해석하면 안 된다. Beam 폭마다 전체 예산을 새로 주는 비교도 하지 않는다.

### 13.3 시간과 데이터

Init 시험은 import·cache 준비·초기 구성·보정·인증을 포함한 첫 결과 시간과 **전체 단계의 최종 길이**를 모두 잰다. Explore만 비교할 때는 동일한 초기 인증해를 사용한다. Polish는 동일한 입력 snapshot에서 시작한다.

WSjk·OPS2는 기존 자료와 비교할 출발점으로 사용하되 두 마커만으로 일반화하지 않는다. 반복 형상, 매우 다른 조각 크기, 많은 회전, 잠금, rigid 그룹, SizeRegions, 오목 pocket, 높은 정점 수 데이터를 추가한다. 선택한 변경별로 짧은/긴 예산과 복수 seed를 시험하며, 후보 변경이 유망하면 예를 들어 seed 10개로 확장한다. 이는 제안된 시험 규모이며 이번에 수행한 횟수가 아니다.

필수 지표는 첫 인증해 시간, 초기 배치 완료율, 최종 인증 길이, 목표 수율 도달시간, CPU 시간, 최대 메모리, 인증/복원 비용, Stop 응답시간이다. Beam은 고유 가지 수·완료된 ordering 수도 기록한다. 목표 미도달은 제한시간으로 바꿔 평균내지 않고 미도달률을 따로 보고한다. 서로 다른 마커의 mm 변화는 합산해 평균내지 않는다.

**제안의 최종 순서는 ‘반복 계산 절약 → 제약을 보존한 Init 개선과 짧은 Beam → 그룹 LP·소수 donor 공동 배정 → Explore의 제한된 목표/후보 정책 실험’이다.** 이진 탐색이나 Beam이라는 이름만 추가하기보다, 현재 코드에서 빠진 판단을 작은 범위에 보완하는 것이 위험 대비 이득을 확인하기 쉽다.

## 14. 검토 범위와 근거 관리

직접 확인한 범위는 `src/ffi_step.rs`, `src/grouped_engine.rs`, `swarm-core/src/{initial,nfp_lbf,nfp_strategy,structure,optimizer,sample,eval,quantify,original_overlap,lp_solver,util}`의 관련 구현이다. 모든 함수에 대한 형식적 정확성 증명이나 전체 버그 감사는 아니다. 기존 실험 기록은 현재 코드와 대조해 이미 적용된 제안을 구분하는 데 사용했다.

검토 전후 관련 소스·Cargo 파일 130개의 SHA256이 동일함을 확인했다. 문서의 로컬 링크 68개는 대상 파일 존재와 지정 줄 범위를 확인했다. 이는 문서·변경 범위 확인이며, 제안의 성능이나 정확성을 실행 시험으로 검증했다는 뜻은 아니다.

추가 참고: [기존 전체 단계 검토](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_PHASE_DEEP_REVIEW_2026-09-08.md), [9/9 적용 내역](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_PHASE_IMPROVEMENTS_2026-09-09.md), [동기/대안 분리 실험](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_SEPARATOR_VARIANTS_TRIAL_RESULTS_2026-09-09.md). 이 문서들의 오래된 DLL 해시·빌드 수치를 현재 산출물의 증거로 사용하지 않았다.

## 15. 후속 구현 상태

사용자가 선택한 **1·2·3·4·5·9·10**을 적용했다. 실제 선택 전략 비용, 형상 준비 재사용, 첫 원본 위반 반환, lane 형상 공유, 인증 Init 경쟁·부분 개선, 강체 그룹 LP, 제한된 공동 pocket 배정이다. 나머지 제안을 자동으로 확대 적용하지 않았다.

후속 요청에 따라 **노력·남은 시간 기반 shrinkStep과 쉬운 목표 재시도는 롤백**했다. 기존 밀도/Feedback 동작과 조기 인증 콜백은 유지한다. 최종 core 816개, native 125개 테스트 통과와 실제 lane 준비 측정, DLL fat LTO·WPF 빌드의 상세 결과는 [구현 보고서](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_PRIORITIES_IMPLEMENTATION_2026-09-09.md)에 있다.
