# Explore·Separator 개선안별 분리 실험

2026-09-09 / `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`

후속: 사용자가 D만 승인하여 [변 검사 순서 재사용을 운영 코어에 적용](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_EDGE_ORDER_APPLICATION_2026-09-09.md)했다. 아래의 ‘운영 미반영’ 및 소스 해시 동일 기록은 실험 완료 당시 상태다.

**36회 실행과 정밀 인증을 완료했다. 운영 코어·설정·DLL은 그대로 유지했다.** 변 검사 순서 재사용(D)이 가장 유망했지만 이득은 작았다. 준비 목록 공유(B)는 최종 길이를 그대로 유지했다. 동일 이동 생략(A)과 공통 배치 인계(C)는 품질 악화 사례가 있어 현 상태로 채택하지 않는 것이 좋다.

## 시험한 변경

운영 코어의 현재 상태를 `target/separator-variants-trial/source`에 복사한 뒤, 그 복사본에만 아래 변경을 넣었다. 사용자 Config 및 컴파일 기능으로 알고리즘을 꺼서 얻는 성능 개선은 시험하지 않았다. 각 시험은 독립 적용이며, A+B+D를 합친 효과는 이번 결과에 포함하지 않는다.

| 구분 | 실제 변경 | 그대로 유지한 부분 |
|---|---|---|
| 기존 동기식 | 기존 Explore 1개, 내부 워커 3개 | 매 반복 전체 완료 후 승자 배치 공유 |
| A | 선택한 이동·회전이 기존 값과 정확히 같으면 재배치 생략. 진입 loss가 0이면 빈 분리 반복 생략 | 원본 보정과 Explore의 정밀 인증, 중지 확인 |
| B | 각 라운드의 잠금 제외·양의 loss인 후보 목록을 한 번 수집해 워커에 전달 | 워커별 shuffle과 이동 직전 최신 loss 검사. 그룹은 기존 경로 사용 |
| C | 독립 Explore 3개가 각자 내부 워커 1개 사용. 분리 작업이 끝난 경계에서 더 짧은 공통 인증 배치를 인계받음 | 느린 동료 작업을 취소하지 않음. 같은 전체 마감 시간 사용 |
| D | 조각 검색을 시작할 때 비트 반전 순서의 변 인덱스를 한 번 생성해 반복 평가에서 재사용 | 변 방문 순서, SIMD 충돌 계산, 조기 종료 및 정밀 인증 |
| 독립 실행 대조군 | Explore 3개 × 내부 워커 1개. 최종 결과만 병합 | 다른 스트림의 배치를 탐색 출발점으로 받지 않음 |

B는 후보 목록 수집의 중복만 줄인다. 전체 배치 복원이나 tracker 복사를 제거한 실험이 아니다. D는 폴리곤의 변 수를 줄이거나 충돌 검사를 건너뛰지 않는다. 인덱스 벡터를 검색마다 준비하는 비용까지 포함해 측정했다.

C에서 배치를 인계받을 때 현재 배치·strip 폭·tracker를 함께 복원하고, 실패 후보 풀과 정체 카운터를 정리한다. 다른 워커가 만든 성과를 자신이 직전에 수행한 R&R 등에 잘못 귀속하지 않도록 `pending_op`도 지운다. 진행 중인 후보를 새 배치에 병합하지 않는다.

## 조건과 측정 범위

- WSjk 156조각, OPS2 52조각. 이전 비교에서 만든 동일 초기 인증 배치 사용.
- `piece_overlap_mm = 2.2`, `PolySimpl = 0.01`. 잠금·그룹 없음.
- 입력별 각 방식 20초 × seed 41, 42, 43. 총 36회. 방식 순서는 시드별로 회전.
- 기하 워커 총 3개. 원본 검증의 기존 병렬 실행은 유지하므로 OS 전체 스레드 수가 정확히 3개라는 뜻은 아니다. 호스트 논리 프로세서 8개, CPU affinity 미지정.
- R&R, Separator Swap, 샘플 수 등은 모든 방식에서 같은 현재 기본값 사용. NFP sampler는 기본 OFF.
- Release, fat LTO, `simd capmods adaptive structure-first`. SIMD를 켠 기존 코어와 비교했다.
- 초기 입력 처리·Separator 생성·최종 cold 인증 시간은 Explore 20초 예산 밖에서 각각 측정한다. 실행 제한은 모든 스트림이 같은 deadline을 사용한다.
- 모든 타이밍 실행은 순차 수행했다. 벤치마크 중 다른 빌드나 다른 CPU 벤치마크를 병행하지 않았다.

앞선 독립 실행 실험은 30초였다. 이번 비교는 **같은 시험 바이너리로 새로 측정한 20초 기준선**과 비교하며, 이전 30초 수치와 직접 합산하지 않는다. 짧은 실험이므로 장시간 최종 수율에 대한 확정 결론은 아니다.

## 충돌 검사 표본 측정

별도 WSjk 5초 실행에서 후보 평가 256회마다 한 번 시간을 계측했다. 표본 25,863개에서 평가 총시간 합계 49.5601ms, 형상 변환·SIMD 입력 준비 2.9882ms, 충돌 대상 수집·loss 계산 39.6910ms였다. 평가 표본 시간의 약 80.1%가 마지막 구간, 약 6.0%가 변환 구간이다. 중첩 타이머 자체의 비용이 포함되며 전체 Explore의 CPU 프로파일이라고 해석하면 안 된다.

처음 검토한 ‘허용 loss가 0이고 확실히 원단 밖인 후보 사전 탈락’은 표본 25,863개 중 4개에만 해당했다. 이 필터는 최종 D 시험에 사용하지 않았다. 반복되는 충돌 검사 순서 준비를 줄이는 안으로 바꿨다.

## 사전 검증

두 입력 각각 80개 고정 라운드에서 B와 D를 기존 방식과 비교했다. **매 라운드의 모든 조각 좌표·회전, 총 loss, 가중 loss, 평가 횟수 및 이동 횟수가 동일**했다. B가 실제로 80번 후보 목록을 준비한 것도 확인했다.

A의 빈 작업 경로는 두 입력 모두 실제 진입했으며, 길이·좌표를 바꾸지 않고 native 및 새 원본 guard 검사를 통과했다. 고정 반복 검증은 정확성 확인용이다. 짧고 한 번씩 실행한 타이밍을 속도 개선율로 주장하지 않는다.

시험 도구의 모드 접미사가 겹쳐 A가 선택되지 않는 문제를 사전 검증에서 발견했다. 수정 후 재빌드·재검증하고 타이밍 비교를 새로 시작했다. 수정 전 pilot은 `pilot-before-mode-fix`에 분리했으며 최종 집계에서 제외했다.

## 최종 길이·수율 및 목표 도달 시간

| 입력 | 방식 | 평균 길이(mm) | 기존 대비(mm) | 평균 수율 | 기존보다 짧음 / 동일 |
|---|---|---:|---:|---:|---:|
| wsjk | 기존 동기식 | 6,495.562 | +0.000 | 78.316% | 0/3 · 3/3 |
| wsjk | A: 동일 이동·빈 작업 생략 | 6,511.993 | +16.432 | 78.120% | 0/3 · 2/3 |
| wsjk | B: 준비 목록 공유 | 6,495.562 | +0.000 | 78.316% | 0/3 · 3/3 |
| wsjk | C: 완료 시 공통 최선 배치 인계 | 6,569.609 | +74.047 | 77.433% | 0/3 · 0/3 |
| wsjk | D: 변 검사 순서 재사용 | 6,491.505 | -4.057 | 78.365% | 1/3 · 2/3 |
| wsjk | 독립 실행 대조군 | 6,573.790 | +78.228 | 77.384% | 0/3 · 0/3 |
| ops2 | 기존 동기식 | 3,480.269 | +0.000 | 83.795% | 0/3 · 3/3 |
| ops2 | A: 동일 이동·빈 작업 생략 | 3,474.628 | -5.641 | 83.933% | 1/3 · 0/3 |
| ops2 | B: 준비 목록 공유 | 3,480.269 | +0.000 | 83.795% | 0/3 · 3/3 |
| ops2 | C: 완료 시 공통 최선 배치 인계 | 3,486.034 | +5.765 | 83.657% | 1/3 · 0/3 |
| ops2 | D: 변 검사 순서 재사용 | 3,480.269 | +0.000 | 83.795% | 0/3 · 3/3 |
| ops2 | 독립 실행 대조군 | 3,486.034 | +5.765 | 83.657% | 1/3 · 0/3 |

| 입력 | 방식 | seed 41 / 42 / 43 길이(mm) | 목표 수율 도달 시간(s) |
|---|---|---|---|
| wsjk | 기존 동기식 | 6,503.655 / 6,479.375 / 6,503.655 | 10.817 / 10.760 / 18.195 |
| wsjk | A: 동일 이동·빈 작업 생략 | 6,503.655 / 6,479.375 / 6,552.950 | 10.713 / 10.492 / 미도달 |
| wsjk | B: 준비 목록 공유 | 6,503.655 / 6,479.375 / 6,503.655 | 10.922 / 10.981 / 18.255 |
| wsjk | C: 완료 시 공통 최선 배치 인계 | 6,565.428 / 6,577.970 / 6,565.428 | 미도달 / 미도달 / 미도달 |
| wsjk | D: 변 검사 순서 재사용 | 6,491.484 / 6,479.375 / 6,503.655 | 10.297 / 10.896 / 18.197 |
| wsjk | 독립 실행 대조군 | 6,577.970 / 6,577.970 / 6,565.428 | 미도달 / 미도달 / 미도달 |
| ops2 | 기존 동기식 | 3,486.005 / 3,486.005 / 3,468.797 | 5.205 / 13.986 / 2.175 |
| ops2 | A: 동일 이동·빈 작업 생략 | 3,451.846 / 3,491.799 / 3,480.240 | 10.673 / 미도달 / 2.072 |
| ops2 | B: 준비 목록 공유 | 3,486.005 / 3,486.005 / 3,468.797 | 5.043 / 13.453 / 2.116 |
| ops2 | C: 완료 시 공통 최선 배치 인계 | 3,474.504 / 3,491.799 / 3,491.799 | 2.244 / 미도달 / 미도달 |
| ops2 | D: 변 검사 순서 재사용 | 3,486.005 / 3,486.005 / 3,468.797 | 5.367 / 14.046 / 2.129 |
| ops2 | 독립 실행 대조군 | 3,474.504 / 3,491.799 / 3,491.799 | 2.348 / 미도달 / 미도달 |

목표 도달 시간은 WSjk 78%, OPS2 83.6%의 인증 결과를 조정자가 수신한 시각이다. 미도달을 20초로 치환해 평균내지 않는다. 길이 차이는 시험−기존으로, 음수가 개선이다. 두 입력의 길이는 서로 합산해 평균내지 않는다.

## 판단

| 안 | 관측 결과 | 판단 |
|---|---|---|
| A: 동일 이동·빈 작업 생략 | WSjk 평균 +16.432mm, OPS2 평균 −5.641mm. 개선 1회, 악화 3회, 동일 2회 | 반복 작업은 줄지만 품질이 일관되게 좋아지지 않는다. 묶음 채택 보류 |
| B: 준비 목록 공유 | 6개 비교 모두 최종 길이 동일. OPS2 목표 도달은 세 시드 모두 소폭 빨랐고, WSjk는 세 시드 모두 소폭 느림 | 결과 보존은 확인. 범용 속도 개선이라고 결론내릴 근거는 부족 |
| C: 공통 최선 배치 인계 | WSjk 평균 +74.047mm, 수율 −0.883%p. OPS2 평균 +5.765mm | 현재 동기식을 대체할 대상으로는 비추천 |
| D: 변 검사 순서 재사용 | WSjk 평균 −4.057mm, 수율 +0.049%p. OPS2 동일. 6개 비교 중 1회 개선, 5회 동일 | 다음 검증 우선순위는 가장 높지만, 작은 차이여서 확정적인 성능 향상으로 보기는 어려움 |

**추천은 기존 동기식 구조를 유지하고, D처럼 검사 의미를 보존하는 작은 최적화를 우선하는 것이다.** B는 보조 후보로 남기고, A·C의 운영 반영은 보류한다. 이번에는 실험을 요청받았으므로 D도 운영 코드에 넣지 않았다.

### A: ‘같은 좌표’라도 이후 계산까지 완전히 같지는 않았다

20초 실행에서 WSjk는 시드별 12,448 / 14,125 / 14,709회, OPS2는 29,648 / 31,262 / 28,253회의 동일 좌표 재배치를 생략했다. 빈 진입 라운드도 모두 생략했다. 기능이 실행되지 않아 차이가 없었던 것은 아니다.

추가 고정 80라운드 비교에서 A의 조각 좌표는 두 입력 모두 기존과 같았다. 그러나 WSjk는 한 라운드의 평가 횟수가 2회 달랐고, 다른 라운드의 가중 loss가 0.015625 달랐다. OPS2에서도 한 라운드의 총 loss가 0.015625 달랐다. 동일 위치의 remove/place 및 tracker 재계산을 생략하면 내부 계산까지 완전히 동일하게 유지되는 것은 아니라는 증거다.

긴 탐색에서의 품질 차이에 수치 계산 차이와 시간 제한이 각각 얼마나 기여했는지는 분리해 입증하지 않았다. 단순한 ‘몇 % 빨라졌다’로 설명하지 않는다. 또한 A는 동일 이동 생략과 빈 라운드 생략을 함께 시험했으므로, 빈 라운드 생략 하나의 품질 효과를 따로 측정한 결과는 아니다.

### C: 인계는 작동하지만, 자연 종료 경계까지 기회가 오지 않는 경우가 있었다

공통 배치 인계 횟수는 WSjk seed 41/42/43에서 **2 / 1 / 0회**, OPS2에서 **3 / 0 / 0회**였다. 6회 중 3회에서는 20초 동안 실제 인계가 없었다. 진행 중인 분리를 취소하지 않으므로, 좋은 배치가 공통 저장소에 있어도 그 작업이 자연스럽게 반환하기 전에는 받지 못한다.

완전히 독립적으로 실행한 대조군과 비교하면 C는 WSjk 평균 약 4.181mm 짧았고 OPS2 평균은 같았다. 하지만 기존 동기식보다 두 입력 모두 평균 길이가 길었다. ‘배치를 공유하면 독립 실행의 약점이 충분히 해소된다’는 가설은 이번 시제품과 예산에서 지지되지 않았다.

이는 모든 비동기 스케줄러가 불리하다는 증명은 아니다. 이번 시제품은 **전체 분리 호출 완료**를 인계 경계로 사용했다. 더 세밀한 작업 경계나 미완료 후보 공급은 시험하지 않았다.

## 적용 상태와 한계

- 2입력 × 3시드 × 6방식 = **36회 모두 정상 종료**.
- 매 실행의 최종 공개 결과와 각 독립 스트림의 최선 결과가 native 검증 및 새 원본 guard 검사를 통과했다. 검색용 근사 loss만으로 통과시키지 않았다.
- 결과 이력은 길이가 엄격히 줄어드는 경우만 갱신했고, 최종 반환 길이가 마지막 공개 최선 길이와 일치했다.
- 실제 Explore 종료 시간은 **20.0014~20.0076초**. 이후 별도 cold 검증은 **0.0037~0.0593초**였다.
- Release 예제 빌드 성공. 시험 전 기록한 운영 코어 **110개 파일의 SHA256이 모두 동일**했다.
- 시험 스크립트의 Python·PowerShell 구문 검사와 `git diff --check`를 통과했다.
- 운영 Rust 소스·C ABI·사용자 Config·WPF·배포 DLL을 이번 실험으로 수정하거나 교체하지 않았다. 시험 복사본, 실행·집계 스크립트와 이 보고서만 추가했다.

이번 결과는 Explore·Separator 시제품 평가다. Compress·Polish, NFP ON, 다른 워커 수, 잠금·그룹 입력, WPF Stop 버튼을 통한 동시 취소는 검증하지 않았다. deadline 종료에서 마지막 인증 결과를 보존한 것과 사용자 Stop 경로 전체를 검증한 것은 구분한다.

각 개선 모드를 하나의 시험 바이너리에 넣었으므로 기준선에도 공통 계측·모드 확인 코드가 있다. 운영 DLL의 절대 속도와 같다고 보장하지 않는다. C는 검증된 전체 배치를 인계하는 시제품이며, 미완료 후보까지 유한 작업 큐로 공급하는 완성형 스케줄러가 아니다.

## 재현 자료

- [복사본 생성](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/setup_separator_variants_trial.py), [D 및 고정 반복 검증 추가](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/refine_separator_variants_trial.py)
- [실행 스크립트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/run_separator_variants_trial.ps1), [사전 검증](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/check_separator_variants_trial.py), [집계](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/summarize_separator_variants_trial.py)
- [조건·입력·실행 파일 SHA256](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/separator-variants-trial/manifest.json)
- [집계 JSON](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/separator-variants-trial/summary.json), [고정 반복 검증 결과](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/separator-variants-trial/verification.json)
- [동일 이동 생략의 추가 경로 비교](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/separator-variants-trial/noop-trajectory.json), [비교 스크립트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/check_noop_separator_trial.py)
- [시험 전 운영 코어 SHA256 목록](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/separator-variants-trial/production-source-hashes.json)

처음 만드는 출력 폴더에서의 순서:

```powershell
python scripts/setup_separator_variants_trial.py
python scripts/refine_separator_variants_trial.py
cargo build --manifest-path target/separator-variants-trial/source/Cargo.toml -p swarm-core --release --example async_separator_trial --features 'simd capmods adaptive structure-first' --target-dir target
Copy-Item -LiteralPath target/release/examples/async_separator_trial.exe -Destination target/separator-variants-trial/variants.exe
./scripts/run_separator_variants_trial.ps1 -VerifyOnly
./scripts/run_separator_variants_trial.ps1
python scripts/summarize_separator_variants_trial.py
```

기존 시험 source나 같은 이름의 타이밍 결과가 있으면 스크립트가 덮어쓰지 않고 중단한다. 재실행할 때는 보존한 기록과 분리된 경로를 사용한다. `target`은 Git 추적 대상이 아니므로 원시 결과·시험 실행 파일을 장기 보관하려면 별도 복사해야 한다. 기록 JSON은 벤치마크 자료이며 WPF Export JSON과 같은 형식의 Uttu 가져오기 파일이 아니다.

## 별도 질문: Uttu.Nest 화면의 Place 형상

사용자가 지정한 `C:\Caplanet\Products\uttu-cad\apps\Uttu.Nest`와 같은 저장소의 직접 모델·표시 의존 코드만 읽었다. Uttu와 fable 코드는 수정하지 않았고 fable은 읽지 않았다.

화면은 [SKCache](C:/Caplanet/Products/uttu-cad/domains/Uttu.Nest.Models/Painting/SKCache.cs:84)에서 `part.Shape.Surface.Polygon`을 SKPath로 바꾸어 그린다. Swarm의 PolySimpl 결과로 원래 외곽을 교체해 그리는 경로가 아니다. 다만 [MkPieceShapeBuilder](C:/Caplanet/Products/uttu-cad/domains/Uttu.Nest.Models/IO/Builder/MkPieceShapeBuilder.cs:172)가 CAD 곡선을 `GetFlattenPoints()`로 펼쳐 Polygon을 만들므로, 정확한 CAD 곡선 자체와는 구분한다.

직선은 끝점을 그대로 사용하고 곡선은 [CEdge.GetFlattenPoints](C:/Caplanet/Products/uttu-cad/domains/Uttu.CAD.Models/Models/Curves/CEdge/CEdge.cs:251)의 `Curve4Div`로 근사한다. 기본 `ApproximationScale = 10`이며, [내부 거리 기준 계산](C:/Caplanet/Products/uttu-cad/shared/Uttu.Geometrics/Curves/Curve4Div.cs:54)은 `0.5 / 10 = 0.05`다. mm 좌표에서는 **0.05mm 기준의 곡선 평탄화**에 해당한다. 이는 실제 모든 조각의 최대 경계 오차를 측정한 값이나 일정한 단순화율은 아니다. Swarm의 `PolySimpl = 0.01`과도 별개다.

Swarm은 이 외곽을 입력받은 뒤 탐색용 오프셋·단순화를 추가 적용한다. 결과의 위치·회전을 Uttu의 원래 Polygon에 적용하면 표시 경계가 Swarm의 탐색 경계와 달라질 수 있다.

위 0.05mm는 현재 코드의 기본 생성 경로에서 사용하는 기준이다. 특정 `.unest` 파일에 저장된 모든 형상의 실제 오차나 정점 감소율을 측정한 결과는 아니다.
