# Swarm Explore·Compress·Polish 코드 정밀 검토

검토일: 2026-09-08  
대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`  
검토 시작 커밋: `0d17477e4419cca621e2f5b6d34e8683adeba943`  
범위: 단계 실행 경로, 후보 생성·평가·채택, 공통 충돌 분리기, 원본 검사·보정, 잠금·그룹 제약, SIMD·병렬화, 중지·콜백 경계.

## 1. 먼저 결론

**지금 가장 유망한 개선은 알고리즘을 더 많이 돌리는 것이 아니라, 같은 상태를 복사하고 검증하는 비용을 줄이고 실제 원단 길이를 줄인 시도에 시간을 더 배분하는 것입니다.**

- Explore·Compress의 중심인 Separator에는 이미 worker 병렬 실행과 충돌 계산 SIMD가 있습니다. R&R 후보 생성과 LP 접촉 계산에도 병렬 경로가 있습니다.
- 탐색용 Swap/R&R이 일시적으로 충돌을 만들 수 있지만, 주요 단계에는 **후속 분리 → 정밀 검증 → 단계별 채택 조건 확인**이 있습니다. 충돌 비용이 줄었다는 사실만으로 최종 결과를 채택하지 않습니다.
- **원본 겹침 검사에서 중지 확인이 늦어지는 버그 1건을 재현하고 수정했습니다.** 이번 변경은 겹침 허용량·정밀도·최종 채택 기준을 바꾸지 않습니다.
- 최적화 우선순위는 **중복 정밀 검증 제거 → worker 상태 복사 절감 → Polish의 검증된 중간 개선 보존 → 비용·실제 길이 개선을 반영하는 연산자 선택**입니다.
- 이번 검토로 “Swap을 켜면 항상 수율이 오른다”거나 “몇 % 빨라진다”고 결론 내릴 수는 없습니다. 생산 입력을 사용한 새 A/B 성능 비교는 하지 않았습니다. 아래의 예상 효과는 구현·측정이 필요한 제안입니다.

`Config` 값을 낮추거나 기능을 꺼서 속도를 높이는 방식은 개선안에서 제외했습니다. 실행 경로 설명에 설정 이름을 사용한 것은 현재 동작을 구분하기 위해서입니다.

## 2. 단계가 하는 일을 쉽게 설명하면

| 단계 | 하는 일 | 중간에 겹쳐도 되는가? | 결과를 남기는 조건 |
|---|---|---|---|
| Explore | 조각의 큰 배치를 바꾸어 더 짧은 원단에서도 들어갈 가능성을 찾음 | 가능. 교란한 후보는 작업 중 상태 | 분리 성공과 정밀 검증을 통과한 해를 보관. 상위 실행 경계에서 기존 정상 해와 비교 |
| Compress | 정상 배치를 출발점으로 원단 길이를 조금 줄이고 조각들을 다시 분리 | 줄인 직후에는 가능 | 더 짧은 길이에서 분리·검증 성공. 실패하면 직전 정상 해로 복원 |
| Polish | LP로 전체 위치 조정, 왼쪽/아래 밀기, 빈 공간 채우기 | 개별 연산의 탐색 모델과 원본 검증 사이에 차이가 있을 수 있음 | 라운드의 원본 보정·정밀 검증 성공. 길이 감소 또는 같은 길이에서 더 조밀한 배치 |
| Reclaim | Polish에 남은 시간을 재압축·재탐색 등에 사용 | 선택한 연산에 따라 가능 | 검증된 기존 최선보다 실제 길이가 줄어든 경우 |

코드의 `strip_width`는 네스팅 진행 방향의 길이입니다. 보고서에서는 사용자 관점에 맞춰 **원단 사용 길이**라고 부릅니다. 고정 원단 폭·동일 조각 집합에서는 사용 길이가 줄면 면적 기반 수율은 증가합니다. 다만 단순화/침식 도형으로 계산한 내부 density와 Uttu의 원본 면적 기반 수율은 구분해야 합니다.

## 3. 실제 실행 경로: 모든 알고리즘이 동시에 실행되지는 않음

### 일반/Legacy 경로

`optimizer/mod.rs` 및 `src/ffi_step.rs::select_exploration_phase`의 Explore 선택 우선순위는 다음과 같습니다.

1. `ordering_exploration`: NFP 기반 배치 순서 탐색.
2. 그 외 `escape_enabled`: 정체 감지 및 단계적 탈출 탐색.
3. 그 외 `adaptive` 빌드: 적응형 Swap/R&R 탐색.
4. 그 외: 기본 Explore.

이후 Compress → Polish → 남은 예산의 Reclaim으로 이어집니다. 빌드 feature가 있다는 것과 해당 경로가 실제 선택된다는 것은 다릅니다. 로그의 단계 선택과 실제 입력을 함께 확인해야 합니다.

### Grouped 및 Structure-first 경로

- Grouped 엔진은 lane별 slice를 만들고 Explore/Compress/Polish를 호출합니다. `explore_slice`는 기본/Adaptive Explore를 직접 선택합니다. 일반 경로의 NFP ordering/Escape 선택기를 그대로 쓰지 않습니다.
- SizeRegions는 lane 바깥쪽 병렬화와 내부 worker 수를 조정합니다. 내부 worker만 보고 “병렬이 꺼졌다”고 판단하면 안 됩니다.
- Structure-first는 P0 전처리 → P1 초기 구성 → P2 압축 → P3 탐색 → P4 개선 → P5 마무리의 별도 조정 경로입니다. 일반 3단계와 단계 순서·시간 배분·후보 관리가 다릅니다.
- 본 보고서는 3단계와 이들을 호출하는 경계를 중심으로 검토했습니다. Structure-first P0/P1의 모든 초기배치 생성 알고리즘 자체에 대한 전면 감사는 별도 범위입니다.

**개선 제안:** 결과와 성능 로그에 실제 경로, 각 연산 호출/성공/채택 횟수, 검증된 길이 감소를 함께 기록하십시오. 같은 버튼·같은 설정 이름이라도 엔진에 따라 실행된 알고리즘이 달라지는 점을 먼저 드러내야 비교가 가능합니다.

## 4. 알고리즘 목록과 동작 점검

아래의 “확인”은 소스 경로와 관련 테스트에서 확인한 범위를 뜻합니다. 모든 생산 입력에 대한 최적성 증명이나 전수 검증을 뜻하지 않습니다.

| 위치 | 알고리즘/로직 | 확인한 동작 및 제한 | 개선 방향 |
|---|---|---|---|
| 공통 | 충돌 후보 수집 + 원/극점 기반 겹침 비용 | 빠른 비용은 탐색용 지표. 최종 원본 겹침 검사를 대체하지 않음 | 후보 수집·정밀 계산 비용을 분리 계측 |
| 공통 | GLS 충돌 가중치 갱신 | 반복 충돌에 가중치를 부여. raw loss와 weighted loss를 따로 보관 | 오래된 가중치의 효과·비용을 측정하되 rollback 의미 유지 |
| 공통 | 무작위/집중 샘플링 | 전역 후보와 현재 위치 주변 후보, 왼쪽 편향 후보 등을 평가 | 유사 후보 중복 제거, 막힌 경계에서 유도한 후보 추가 |
| 공통 | 좌표 하강법 | x/y 및 허용 회전 축 양방향 평가, 성공/실패에 따라 보폭 조절 | 같은 접촉 집합을 쓰는 평가의 재사용 검토 |
| 공통 | NFP 후보 제안 | 가능한 위치를 제안하고 실제 평가기로 확인. 실패 시 다른 샘플 경로 | NFP 준비 비용과 실제 후보 성공률 기반 예산 배분 |
| 공통 | 다중 Separator worker | 독립 상태에서 이동 후 최선 worker를 채택 | 반복 전체 복사 대신 변경분 반영 실험 |
| Explore | 기본 축소·분리 반복 | 가능한 해에서 길이를 줄이고, 불가능 해 풀에서 재시도 | 축소 지점을 충돌/빈 공간 구조와 연결 |
| Explore | 불가능 해 풀 + 순위 편향 선택 | Adaptive/기본 경로는 손실 순 풀을 최대 32개로 제한 | 동일 배치·유사 배치 중복 억제, 복사 전 탈락 판정 |
| Explore | 탐색용 Swap | 큰 배치 변화를 만들어 국소 정체 탈출. 일시 악화 가능 | 최종 길이 개선까지 추적하는 기여도 기록 |
| Explore | 적응형 연산자 선택 | 상태별 점수 + 학습 보상 + 미시도 탐색 + 잦은 선택 변경 억제 | 호출 비용과 검증된 길이 감소를 보상에 반영 |
| Explore | R&R Rightmost | 오른쪽 조각들을 제거 후 재배치 | 실제 오른쪽 끝을 결정하는 조각과 그 방해 조각 구분 |
| Explore | R&R LowDensity | 저밀도 구간을 대상으로 재구성 | 단순 밀도보다 옮길 수 있는 빈 공간의 형태 반영 |
| Explore | R&R Fragmented | 단편적인 빈 공간/배치 구간을 겨냥 | 진단 맵의 변경 구간 재사용 |
| Explore | R&R GapTarget | 빈 공간 진단으로 목표 영역 선택 | 진단 비용을 회수할 가능성이 있는 후보에 집중 |
| Explore | R&R Random + BandTabu | 다양성 확보, 같은 구간의 반복 제거 완화 | 실제 다른 배치를 만든 경우만 유효 시도로 집계 |
| R&R 공통 | regret 재삽입 | 대안이 적은 조각 우선. 최초 샘플 평가 병렬, 확정은 순차 재검사 | 상위 후보 목록을 보존하여 재계산 횟수 축소 |
| Separator | 비용 개선용 Swap | 가중 국소 비용 감소 + raw 국소/전체 비용 비악화 조건 | 대상 쌍을 무작위에서 충돌 연결/맞춤 가능성으로 보강 |
| Explore | 밀도 기반 축소·실패 피드백 | 선택된 모드에 따라 축소량 조절 | 길이 변화 대비 분리 작업량을 반영하는 제어 실험 |
| Explore | 폭 진동/폭 완화 | 정체 시 일시적으로 공간을 열고 재시도 | 효과 없던 완화의 반복 억제, 검증된 최선 보존 |
| Escape | 정체 감지 + 병목 조각 순위 | 효율, 실패, 오른쪽 경계, 다양성 등을 사용 | 가능한 해의 개선과 불가능 해의 지표 변화를 분리 |
| Escape | Soft/Medium/Hard 탈출 | 작은 이동·회전/부분 재삽입/큰 재구성으로 강도 변경 | 탈출 비용 대비 이후 분리 성공과 길이 개선으로 평가 |
| Escape | Conflict graph/Burst | 충돌 연결 성분, push, 부분 재구성, 공간 완화 | 이미 만든 충돌 그래프·진단 결과 공유 |
| Escape | 해 풀 계층별 다양화 | 좋은 해 보존, 중간 해 이동, 나쁜 해 재초기화 | 서로 다른 배치가 실제로 유지되는지 측정 |
| NFP Explore | 순서 Swap/Move/Block + SA | 순서 변경 후 재배치. SA는 임시 악화도 확률적으로 수용 | prefix 재사용 이득과 큰 순서 변화의 품질 이득 균형 |
| NFP Explore | prefix 재사용 + 주기적 LP | 재사용 가능한 앞부분만 유지. 완성 후보 정밀 검증 | 준비·배치·LP에 외부 중지와 남은 예산을 일관되게 전달 |
| Compress | 시간/실패 기반 축소 감쇠 | 실패 시 정상 해 복원. 성공한 더 짧은 배치만 보관 | 오른쪽 병목/빈 공간에 근거한 축소 후보 |
| Polish | LP/QP backend를 통한 동시 이동 | 선형 접촉 제약, trust region, 실제 배치 backtracking | 접촉 후보 생성과 모델 재구축 비용 절감 |
| Polish | Slide | 왼쪽/아래 이동 후보를 검사하며 작은 틈 회수 | 후보 경계·접촉점으로 탐색 범위를 줄임 |
| Polish | Hole Fill | 격자/BFS 빈 공간 탐색, 앵커·회전·국소 정련 후 검사 | 오른쪽 끝 감소를 직접 점수화, 빈 공간 맵 증분 갱신 |
| 종료 보강 | Reclaim | 남은 시간으로 Compress/Explore/R&R/Escape 재진입 | 각 재진입 비용과 실제 개선 확률에 근거해 선택 |
| 공통 최종 경계 | 원본 검사·보정 + 수량/회전/잠금/그룹/strip 검증 | 좌표를 기준으로 도형/CDE를 재구축하여 정밀 검증 | 검증 증명 재사용, 변경 조각·쌍 단위 무효화 |

## 5. Explore·Separator 상세

### 5.1 좋은 후보를 찾는 구조는 갖춰져 있음

Separator는 현재 충돌하는 이동 가능 조각을 대상으로 후보를 찾습니다. worker마다 배치·충돌 tracker·RNG·평가 버퍼가 독립적입니다. worker 이동은 병렬로 실행하고, 가중 비용이 가장 좋은 worker의 상태를 주 상태와 교환합니다. 이미 ownership 교환과 평가 버퍼 재사용이 적용되어 있습니다.

분리 도중 가장 좋은 raw loss의 해와 tracker를 함께 저장합니다. 충돌 가중치가 바뀌어도 raw loss로 실제 개선을 따로 봅니다. `separate()`가 반환한 최선 snapshot과 separator의 마지막 작업 상태가 같다고 가정하면 안 되는데, 주요 Explore 경로에는 반환 snapshot으로 명시적으로 rollback하는 코드가 있습니다.

좌표 하강법은 중지 확인과 평가 횟수 상한을 갖고 있습니다. 회전 제약이 있는 경우 반복 정련으로 허용 각도 기준점이 계속 이동하지 않도록 고정된 기준 회전에서 범위를 제한합니다. 최종 검증도 실제 허용 회전을 다시 확인합니다.

### 5.2 중요한 병목 후보: 상태 복사와 검사 반복

`move_items_multi()`는 반복마다 주 배치를 저장하고 모든 worker에 복원하며 tracker를 복사합니다. `PairMatrix`는 조각 수 n에 대해 `n(n+1)/2`개의 `CTEntry`를 저장하는 조밀한 삼각 행렬입니다. 따라서 worker 수를 늘려도 복사·메모리 비용이 함께 증가합니다.

**제안:** 먼저 snapshot 저장, CDE 복원, tracker 복사, 실제 후보 평가를 각각 계측하십시오. 복사가 큰 비중이면 변경된 조각/쌍 목록과 worker 세대를 사용해 동기화할 수 있습니다. 단, R&R은 placement key 자체를 바꿀 수 있고 GLS 가중치도 상태이므로 좌표만 복사하는 방식으로 바꾸면 안 됩니다. rollback·가중치·행 합계 cache를 포함한 동등성 검사가 필요합니다.

불가능 해 풀은 제한되어 있지만, 가능한 개선 해를 누적하는 벡터와 pose signature 생성·정렬에도 비용이 있습니다. 소비자가 마지막/최선 해만 필요하다면 제한된 개선 이력으로 바꿀 여지가 있습니다. Grouped 등 모든 소비자를 확인한 후 바꾸어야 합니다.

### 5.3 연산자 학습 목표와 사용자 목표의 차이

Adaptive 경로는 실제 상태 변화가 없는 연산을 no-op으로 처리하고, 실행된 연산의 성과를 반영합니다. 미시도 연산을 시험하는 장치와 선택이 잦게 뒤집히는 것을 억제하는 장치도 있습니다.

그러나 현재 보상은 주로 분리 가능 여부와 충돌 손실 감소에 근거한 10/5/1/0 수준의 값입니다. 오래 걸려 작은 충돌만 줄인 R&R이, 짧게 실행되어 이후 원단 끝을 회수하게 만든 Swap보다 유리하게 학습될 수 있습니다.

**제안:** 충돌 감소 보상은 분리 진행용으로 남기고, 연산 후 후속 분리·검증까지 포함한 경과 시간과 검증된 사용 길이 감소를 별도 성과로 기록하십시오. 최종 채택에 기여한 연산의 보상을 추가하는 방식부터 시험하는 것이 적절합니다. 모든 보상을 즉시 `mm/ms` 한 항으로 바꾸면 잠깐 나빠져야 탈출하는 연산을 억제할 수 있습니다.

### 5.4 Swap과 R&R이 실제 수율을 높였는지 판단하는 법

탐색용 Swap과 Separator Swap은 다릅니다.

- 탐색용 Swap: 충돌을 늘려서라도 배치를 크게 바꾸는 연산입니다. 성공 판정은 이후 분리와 최종 검증 뒤에 해야 합니다.
- Separator Swap: 현재 충돌 상태를 개선하는 연산입니다. 가중 국소 손실이 줄고 raw 국소/전체 손실이 나빠지지 않아야 그 교환을 유지합니다. 이것만으로 원단 사용 길이가 줄었다고 할 수는 없습니다.
- R&R: 제거 후 처음 만든 후보는 서로 아직 만나지 않은 상태에서 계산할 수 있습니다. 따라서 재삽입할 때 현재 배치에 다시 검증하고 필요하면 재샘플링합니다. 이 순차 확정 단계는 유지해야 합니다.

검토한 주요 경로는 임시 후보를 그대로 최종 해로 내보내지 않습니다. 다만 raw loss가 0인데 정밀 검증이 실패하면 기존 해를 유지하고 탐색을 끝내는 경로가 있습니다. 안전한 처리이지만 남은 시간을 활용하지 못합니다. **실패 쌍을 되돌려 제한적으로 재분리하는 제안**은 가능하며, 실패 반복 상한과 취소 시 정상 해 복원이 반드시 필요합니다.

이번에는 Swap/R&R 생산 데이터 A/B를 새로 실행하지 않았으므로 수율 개선량은 미측정입니다. 기존 문서의 과거 수치를 이 커밋의 성능으로 재사용하지 않았습니다.

## 6. Compress 상세

Compress는 직전 정상 해로 돌아간 다음 원단의 한 지점을 기준으로 길이를 줄이고 Separator를 실행합니다. 시간 기반 감쇠는 단계 진행에 따라 축소량을 줄이고, 실패 기반 감쇠는 실패 누적에 따라 축소량을 줄입니다. 축소 비율의 유효 범위 정규화도 있습니다.

**확인한 보호 장치:** 분리 결과의 비용 0만 확인하지 않고 정밀 검증까지 통과한 후보를 채택합니다. 실패한 시도는 보관한 정상 해를 훼손하지 않습니다. 잠금만 있는 배치에서는 끝없이 불필요하게 축소하려 하지 않는 처리도 있습니다.

**유망한 개선:** 현재의 무작위 축소 지점을 그대로 유일한 선택으로 두기보다, 오른쪽 끝을 결정하는 조각, 충돌이 몰린 구간, 연속 빈 통로에서 몇 개의 후보를 만들고 값싼 점수로 먼저 정렬할 수 있습니다. 후보 점수는 같은 읽기 전용 snapshot에서 병렬 계산할 수 있지만, 실제 길이 변경과 분리는 독립 상태 또는 순차 시도로 수행해야 합니다.

`capmods`의 분리 반복 하한은 조각 수에 따라 `max(설정값, 2.5 × n)`으로 올라갑니다. 그래서 이름이 Compress여도 큰 입력에서는 한 시도의 실제 비용이 커질 수 있습니다. 설정을 낮추는 대신 **예상 평가 비용과 최근 성공률에 따른 내부 작업 예산**을 시험할 가치가 있습니다. 성급한 중단으로 어려운 유효 후보를 버리지 않는지 함께 비교해야 합니다.

## 7. Polish 상세

### 7.1 실제 순서는 LP → Slide → Hole Fill

LP가 여러 조각의 이동 방향을 동시에 잡고, Slide가 왼쪽/아래 작은 틈을 회수하고, Hole Fill이 오른쪽 조각을 앞쪽 빈 공간으로 옮깁니다. 후속 pass가 켜져 있으면 LP는 남은 시간의 일부(현재 60%)를 받습니다.

그룹 배치에서는 개별 조각 LP를 실행하지 않고, Slide는 그룹 전체를 이동합니다. Hole Fill은 그룹에 속하지 않는 조각을 그룹 장애물 주위로 배치합니다. 그룹이 존재한다는 이유만으로 모든 개별 이동 연산이 동일하게 비활성화되는 것은 아닙니다.

### 7.2 LP: 선형 모델의 답을 실제 답으로 바로 믿지 않음

LP는 조각 이동량과 원단 길이를 변수로 두고, 가까운 정점/변 접촉으로 충돌 방지 제약을 만듭니다. trust region으로 한 번의 이동 범위를 제한합니다. 접촉 후보는 x축 정렬로 줄이고, 충분히 큰 작업은 병렬 계산합니다.

제약 수를 제한하고 비선형 도형 관계를 국소 근사하므로 모델 해가 실제로도 충돌이 없다고 단정할 수 없습니다. 현재는 실제 배치에 적용하고 CDE 검사를 하며, 실패 시 이동량을 줄이는 backtracking을 수행합니다. 마지막 원본 검사·라운드 채택 검증도 별도로 있습니다.

Microlp, Clarabel, CLP, Xpress 선택 경로와 모델/해 검증이 있습니다. 외부 backend는 라이브러리와 실행 환경에 따라 가용성이 달라지므로 모든 backend가 이 환경에서 검증됐다는 뜻은 아닙니다.

**제안:** 정점×변 비교의 사전 필터와 공간 인덱스, 이전 접촉 집합 재사용, 지원 backend의 이전 모델/기저 재사용을 순서대로 검토하십시오. 이동 후 접촉 관계가 바뀌므로 cache 무효화와 실제 배치 검증은 유지해야 합니다.

### 7.3 Slide: 연속 이동 경로와 최종 위치의 안전은 다른 문제

Slide는 거리를 훑어 막힌 구간을 찾고 이분 탐색으로 이동량을 정련합니다. 자기 자신을 장애물에서 제외하는 probe와 도형 버퍼 재사용이 있습니다. 최종 후퇴 위치도 재검사합니다.

오목 도형에서는 이동 거리에 따른 충돌 여부가 항상 단조롭지는 않습니다. 따라서 이분 탐색이 전 구간에서 가장 먼 이동량을 찾는다고 해석하면 안 됩니다. 네스팅은 최종 배치 위치가 목적이므로 중간 경로 통과 자체가 필수 조건은 아니지만, 더 좋은 틈을 놓칠 수는 있습니다.

**제안:** NFP 경계나 접촉 이벤트를 거리 후보로 사용해 필요한 정밀 probe 횟수를 줄이는 방안을 비교하십시오. 기존 최종 위치 검사로 안전성을 확인할 수 있습니다.

### 7.4 Hole Fill: 빈 공간에 넣었다고 원단 끝이 줄어들지는 않음

격자 셀의 점유를 계산하고 BFS로 닫힌 빈 공간과 오른쪽으로 열린 공간을 구분합니다. 오른쪽 구간의 이동 가능 조각을 골라 크기·앵커·허용 회전·좌표 하강법을 사용하고 실제 충돌 검사를 통과한 위치만 적용합니다. 격자 크기/할당에는 상한 및 오류 처리 코드가 있습니다.

격자는 후보 발견용입니다. 가는 틈이나 셀보다 작은 통로를 놓칠 수 있지만, 이것이 곧 겹친 결과의 채택으로 이어지는 구조는 아닙니다. NFP Hole Fill에서도 준비 cache는 재사용하고 현재 장애물은 갱신합니다.

현재는 먼저 들어가는 유효 공간을 찾는 성격이 강합니다. 가장 오른쪽 조각이 아닌 조각을 옮기면 보기에는 조밀해져도 실제 사용 길이는 그대로일 수 있습니다.

**제안:** 오른쪽 끝을 결정하는 조각을 우선하고, 상위 몇 개 공간의 `이동 후 전체 x_max 감소`를 비교하십시오. 점유 맵의 변경 구간과 연결 성분을 증분 갱신하는 것은 다음 단계입니다. 격자 후보를 더 촘촘하게 만드는 것만으로 해결하면 오히려 시간이 늘어날 수 있습니다.

### 7.5 중간 개선을 시간 종료 시 버리는 비용

현재는 라운드 도중 시간 종료/중지 시 새 검사를 시작하지 않고 마지막 **검증된** 최선으로 돌아갑니다. 원본 검증이 끝나지 않은 좌표를 내보내지 않는 올바른 보호 장치입니다.

하지만 LP가 개선한 뒤 긴 Slide/Hole Fill 중 시간이 끝나면 그 라운드의 LP 개선도 버릴 수 있습니다. **각 pass 경계에 잔여 예산이 충분할 때만 원본 검증·인증 checkpoint를 만드는 방식**을 제안합니다. 검사를 매번 추가하면 더 느려질 수 있으므로, 아래 중복 검증 제거와 함께 시험하는 것이 좋습니다. 취소 후 뒤늦게 인증을 시작하는 방식은 권하지 않습니다.

## 8. 원본 겹침·잠금·결과 검증

### 8.1 현재 겹침 정책

원본 도형 검사는 기존 `piece_overlap_mm`에서 유도된 허용량을 사용합니다. 5mm 고정이 아니며, 직선·유지 정점을 별도로 보호하는 선택적 겹침 방식도 아닙니다. 현재 검토 대상은 **전체 경계에 같은 허용량을 적용하고 원본 도형으로 다시 검사하는 방식**입니다.

원본 도형을 변환하고 정수 격자(1/10000mm)에 올린 뒤, 교집합을 만들고 허용 깊이에 해당하는 침식 후 면적이 남는지 판단합니다. 반허용량과 격자 안전 여유를 사용하는 기하 정책입니다. 이 값은 모든 원본 정점의 거리를 하나씩 측정한 값이나 화면 픽셀 겹침량과 같지 않습니다. 특히 모서리·가는 교차 영역의 판정은 교집합 침식 정의를 따릅니다.

Explore·Compress·Polish에서 사용하는 검색 도형과 원본 검사 경로를 확인했고, Compress·Polish에 겹침 설정과 콜백 인증이 유지되는 기존 회귀 테스트도 실행했습니다.

Uttu.Nest의 **2mm 겹침/원단 이탈 표시 기준**은 UI 정책입니다. 이를 근거로 Rust의 원단 경계까지 2mm 벗어나도 된다고 해석하면 안 됩니다. 이번 변경은 어느 쪽 허용 기준도 바꾸지 않았습니다.

### 8.2 잠금 및 그룹 제약

- 잠금끼리의 겹침과 잠금 조각의 strip 이탈은 비용/검증에서 예외 처리합니다.
- 이동 조각과 잠금 조각의 충돌까지 면제하는 것은 아닙니다.
- 정밀 인증 경계에서는 잠금 pose 보존과 그룹 상대 pose도 확인합니다.
- 수량, 알 수 없는 item ID, 유효하지 않은 숫자, 허용되지 않은 회전도 검사합니다.
- 최종 검증은 저장된 shape/CDE 복사본만 신뢰하지 않고 내보낼 `d_transf` 좌표에서 도형과 장애물을 재구축합니다. 내부 도형과 Export 좌표가 어긋나는 종류의 오류를 막는 데 중요한 방어입니다.

### 8.3 가장 우선할 성능 개선: 인증 결과의 재사용

원본 guard는 pose revision, 변환 도형, 쌍별 계산 결과를 cache합니다. TLS 기반 guard 재사용도 있습니다. FFI는 검증된 snapshot과 JSON payload의 version cache를 갖습니다. 따라서 “cache가 전혀 없으니 추가”가 아니라 **현재 cache 사이에서 동일 snapshot을 반복 인증하는 일을 줄이는 것**이 과제입니다.

`solution_validation_error_with_quantity_policy`는 매번 전체 pose를 읽고 도형/CDE를 재구축합니다. Separator의 원본 보정 후, 단계 인증 후, FFI 채택/콜백 인증에서 같은 해를 다시 만질 가능성이 있습니다.

**제안:** 검증된 snapshot에 명시적인 인증 정보를 연결하십시오. 인증 유효성에는 instance/원본 도형, overlap 정책, 전체 pose 세대, strip 길이, 수량, 잠금·그룹 정책을 포함해야 합니다. 일부 좌표나 strip만 바뀌어도 관련 인증은 무효화해야 합니다. 포인터가 같거나 raw loss가 0이라는 이유만으로 인증을 재사용해서는 안 됩니다.

다음 단계로 변경된 조각의 원본 변환과 그 조각에 접하는 쌍만 재검사할 수 있습니다. 새로 움직여 이웃이 된 쌍도 반드시 생성해야 하므로 예전 충돌 쌍만 검사하는 방식은 잘못됩니다.

## 9. 이번에 수정한 버그: 원본 검사 중지 확인 간격

**위치:** `swarm-core/src/original_overlap.rs::OriginalOverlapGuard::inspect`

기존 코드는 도형 변환과 바깥쪽 조각 반복에서 중지를 확인하지만, 한 조각이 뒤의 여러 조각과 비교되는 내부 SIMD batch 반복에서는 확인하지 않았습니다. 원본들이 밀집해 있으면 다음 중지 확인까지 많은 Clipper 교집합·침식 계산이 이어질 수 있었습니다.

32개 원본 사각형을 같은 위치에 둔 결정론적 회귀 테스트를 추가했습니다. 검사 횟수로 중지 신호를 발생시켜 재현했을 때, 수정 전에는 **정밀 쌍 계산 61회** 후 실패했습니다. 수정 후에는 **최대 4쌍짜리 batch마다 중지를 확인**합니다.

또한 취소한 guard를 그대로 재사용해 검사했을 때 모든 위반 쌍 `32 × 31 / 2 = 496`개를 다시 찾는지 확인했습니다. 부분 cache가 완전한 인증으로 잘못 취급되지 않도록 검증하는 테스트입니다.

변경 파일:

- `swarm-core/src/original_overlap.rs`: 내부 batch 시작에 `term.kill()` 확인 추가.
- `swarm-core/tests/original_overlap_trial.rs`: 재현·재검사 회귀 테스트 추가.

**효과의 범위:** 협조적 중지 신호가 전달된 원본 검사에서 불필요하게 계속하는 작업량을 줄였습니다. 전체 실행 속도 향상률이나 중지 지연 ms를 측정한 결과는 아닙니다. Clipper 호출 한 번의 내부까지 중단할 수 있게 된 것도 아닙니다. 최종 `validate()`의 별도 `BasicTerminator`를 사용하는 인증은 여전히 호출자 deadline으로 중단되지 않습니다.

## 10. SIMD·병렬화: 이미 있는 것과 추가할 것

| 구간 | 현재 | 추가 가능성/조건 |
|---|---|---|
| Separator worker | 독립 상태 병렬 실행 | worker 수 증가 전 복사·메모리 대역 비용 확인 |
| 원/극점 충돌 비용 | SoA 및 SIMD, CPU 기능에 따른 경로 선택 | 작은 후보에서는 SIMD 준비 비용이 이득을 넘는지 계측 |
| 원본 변환·AABB 후보 배제 | SIMD 및 scalar 대체 경로 | 정밀 Clipper 연산 자체까지 SIMD인 것은 아님 |
| R&R 최초 재삽입 제안 | 충분한 작업 수에서 샘플링/regret 평가 병렬. NFP 후보 추출은 소유 cache에서 먼저 수행 | 확정 배치 재검증은 순차 유지. 읽기 전용 후보 점수 확대 가능 |
| LP 접촉 생성 | 작업 크기 조건부 병렬 | 접촉 후보 수 자체를 먼저 줄이고 묶음 크기 조절 |
| NFP cache prewarm | NFP·mask·prepared region을 병렬 계산 후 순차 병합. 반전 방향도 병렬 준비 | 전체 pair를 준비하는 비용과 실제 조회율 비교, 중지 전파 보강 |
| Grouped lane | 독립 lane 병렬 | 바깥 병렬과 안쪽 worker를 함께 계산해 중첩 과다 실행 방지 |
| 원본 쌍별 정밀 교집합 | 현재 guard 내부 순차 처리 | cache miss 중 독립 쌍을 모아 작업별 결과 생성 후 순차 cache 병합 가능 |
| Hole Fill 후보 평가 | 실제 배치 갱신과 연동 | 같은 snapshot의 후보만 병렬 평가하고 승자 확정 후 재검사 |

원본 정밀 검사의 병렬화는 쌍이 많고 Clipper 계산이 비쌀 때 유망합니다. 먼저 실제 cache miss 수와 후보 쌍 크기를 계측해야 합니다. 쌍별 cache를 공유 mutex로 잠그면서 병렬화하면 이득이 사라질 수 있습니다. 각 작업이 독립 결과를 반환하고 한곳에서 병합하는 설계, Clipper wrapper의 동시 호출 안전성 확인, 결정론적 오류 선택, 중지 확인이 필요합니다.

이번 버그 수정은 기존 4개 후보 batch 구조에 중지 확인을 추가한 것입니다. **새 SIMD 커널이나 새 병렬 알고리즘을 이번에 구현한 것은 아닙니다.**

## 11. 남은 위험과 검토가 필요한 제한

| 구분 | 관찰 | 판단/대응 |
|---|---|---|
| 응답성 | 최종 정밀 인증, 일부 NFP 준비/배치·LP 하위 호출은 외부 중지 전파가 충분히 세밀하지 않음 | 단계 시간은 엄격한 전체 wall-clock 상한이 아님. 인증된 이전 해를 유지하면서 하위 제어를 전달하는 개선 필요 |
| 메모리 | 조밀한 PairMatrix와 worker/snapshot 복사가 조각 수의 제곱으로 증가 | 큰 입력의 메모리 예산·할당 실패 경로 검토. 전체를 무조건 sparse로 바꾸면 작은 입력은 느려질 수 있음 |
| 극단 입력 | PairMatrix 길이 계산에 checked 산술이 없음 | 정상 의류 입력에서 overflow를 재현한 것은 아님. 입력 상한과 할당 예산을 함께 검토해야 하는 견고성 과제 |
| 탐색 품질 | Escape의 일부 효율/정체 신호는 불가능한 배치에서도 계산 | 사용 길이만 줄고 아직 겹치는 상태의 지표 개선을 최종 품질 개선과 분리할 필요. 잘못된 최종 채택이 재현된 것은 아님 |
| 경로 일관성 | Grouped Explore는 일반 선택기를 그대로 사용하지 않음 | 실제 지원 경로를 로그/문서로 명시. 옵션 이름만으로 실행 여부 판단 금지 |
| 취소 품질 | Polish는 미인증 라운드를 버림 | 안전성을 유지하되 pass별 사전 인증 checkpoint 실험 |
| 관측 비용 | 콜백 진단·Export JSON이 phase 실행 시간에 포함될 수 있음 | 엔진 시간과 직렬화/소비자 시간을 별도 기록 |
| API 수명 | phase는 엔진 잠금을 유지하며 callback 실행. free/재진입 호출은 수명 계약의 영향을 받음 | 동일 handle의 callback에서 잠금을 다시 요구하는 동기 API를 호출하는 흐름을 피해야 함. 이번 검토가 임의 raw pointer 호출의 메모리 안전을 보증하지는 않음 |

이번 검토에서 사용자 데이터에 의한 메모리 침해를 재현하거나 보안 무결성을 전면 증명한 것은 아닙니다. 실제 재현한 버그와 위의 잠재 위험을 구분했습니다.

## 12. 권장 구현 순서와 검증 기준

| 순서 | 작업 | 기대 효과 | 반드시 확인할 것 |
|---|---|---|---|
| 1 — 이번 완료 | 원본 검사 내부 중지 확인 | 밀집 겹침 검사에서 중지 후 잔여 작업 감소 | 취소·재검사 및 기존 겹침/잠금 정책 유지 |
| 2 | 단계/연산별 시간·검증 횟수·복사량 계측 | 실제 병목과 수율 기여 분리 | 계측 자체 비용, callback 유무 영향 |
| 3 | 동일 snapshot의 중복 인증 제거 | 개선 해를 자주 찾거나 원본이 복잡할 때 유망 | pose/strip/정책 변경 시 인증 무효화 |
| 4 | Separator의 변경분 동기화 | worker·조각 수가 많을 때 복사 비용 절감 | RNG, placement key, GLS, tracker cache 동등성 |
| 5 | Polish pass별 인증 checkpoint | 시간 종료로 버리는 개선 감소 | 추가 인증 비용보다 회수한 개선이 큰지 |
| 6 | R&R/Swap 비용·최종 길이 성과 반영 | 비싼 무효 시도 감소 | 일시 악화가 필요한 탈출 능력 유지 |
| 7 | Hole Fill/Compress의 오른쪽 병목 우선화 | 실제 사용 길이 감소에 더 직접적 | 빈 공간 이동 횟수 대신 최종 x_max 비교 |
| 8 | LP 접촉 후보 축소 및 원본 쌍 병렬화 | 복잡한 도형의 정밀 처리 비용 절감 | 작은 입력 퇴행, backend/FFI 동시 호출 안전성 |

### 생산 데이터 비교 방법

기존 OPS2/여성WSjk 마커에서 만든 **같은 Swarm Input JSON**을 사용하고 파일 hash를 기록합니다. `.unest`를 매번 다른 전처리로 변환하면 비교 입력이 달라질 수 있습니다.

1. A/B가 동일한 초기 배치에서 출발하는 비교와, 초기 배치부터 포함하는 전체 실행 비교를 분리합니다.
2. 동일 feature·Release profile·CPU·worker 예산·phase 시간·overlap·단순화·seed를 유지합니다. 순서 변경으로 난수 소비가 달라지므로 동일 seed만으로 동일 경로가 보장되지는 않습니다.
3. 최소 여러 paired seed에서 반복하고 실행 순서를 교차합니다. 10개 paired seed를 시작점으로 제안하며, 편차가 크면 추가합니다.
4. 최종 **정밀 인증된** 사용 길이(mm), 원본 면적 수율, 목표 수율까지 시간, 원본 검사 시간/cache hit, peak 메모리, 중지 지연 p50/p95를 기록합니다.
5. Swap/R&R은 `시도 → 실제 pose 변화 → 분리 성공 → 원본 검증 성공 → 최선 길이 개선` 수를 각각 기록합니다. accepted swap 횟수만 비교하지 않습니다.
6. UI callback 비활성의 엔진 비교와 실제 UI callback 조건의 비교를 나눕니다. 실제 배포 조건에서 빨라져야 채택 가치가 있습니다.
7. 성능과 품질의 유지/롤백 판단은 이 측정 결과를 보고 사용자가 결정합니다. 본 보고서는 미측정 개선을 자동 적용하거나 성능 통과로 간주하지 않습니다.

## 13. 이번 실행 검증

이번 변경을 포함해 다음 검증을 수행했습니다.

| 검증 | 결과 |
|---|---|
| 추가 회귀 테스트를 수정 전 코드에 실행 | 의도대로 실패. 중지 확인 사이 정밀 쌍 계산 61회 재현 |
| `original_overlap_trial` | 8개 통과. 새 취소/496쌍 재검사 포함 |
| `frozen_policy` | 4개 통과 |
| `overlap_semantics` | 3개 통과, 기존 진단용 1개 ignored |
| `stop_latency` | 3개 통과 |
| `swarm-core` 단위 테스트 | 729개 통과, 기존 13개 ignored |

합계 **747개 통과, 14개 ignored, 실패 0개**입니다. ignored 항목은 실행한 것처럼 계산하지 않았습니다. 단위 테스트에는 Swap/R&R 후속 분리·정밀 검증, LP·SIMD, 경계·회전·pose 재구축 등의 회귀 검사가 포함됩니다.

실행 명령:

```powershell
cargo test -p swarm-core --release --features 'simd capmods adaptive structure-first' --test original_overlap_trial --test stop_latency --test overlap_semantics --test frozen_policy
cargo test -p swarm-core --release --features 'simd capmods adaptive structure-first' --lib
cargo build --release --lib --features 'simd capmods adaptive structure-first jni'
```

현재 `Cargo.toml`의 Release는 `opt-level = 3`, `lto = "fat"`입니다. `codegen-units = 1`은 주석이므로 적용된 설정으로 표시하지 않았습니다.

Native Release DLL 빌드도 성공했습니다. linker가 import library 생성 안내를 warning 1개로 출력했으며 빌드 오류는 없었습니다. 산출물은 `target/release/capswarmcore_100.dll`입니다. 이번 수정 DLL을 Uttu.Nest 설치/실행 폴더로 교체하지는 않았습니다.

`rustfmt --check`와 `git diff --check`가 통과했고 문서의 로컬 파일 링크 존재 여부를 확인했습니다. 이번 작업의 변경은 소스 1개, 테스트 1개, 이 보고서 1개입니다.

재현 및 결과 로그:

- [수정 전 실패 로그](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-review-cancel-before.log)
- [관련 통합 테스트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-review-focused.log)
- [핵심 단위 테스트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-review-core-unit.log)
- [Native Release 빌드](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-review-native-build.log)

이 테스트는 전체 생산 데이터의 품질·성능 비교, 모든 외부 LP backend 검증, WPF UI 종단 검증을 대신하지 않습니다. 그 작업은 이번 검토에서 수행하지 않았습니다.

## 14. 소스 근거

다음 링크는 검토한 로컬 소스입니다. 본문 함수 이름과 함께 보면 호출 경로를 추적할 수 있습니다.

- [일반 단계 실행 선택](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/mod.rs:317), [Native 단계 선택](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:2336), [Grouped 단계 slice](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/grouped_engine.rs:2348)
- [기본 Explore](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore.rs), [Adaptive Explore](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/explore_mods.rs:595), [연산자 선택](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/adaptive_selector.rs:128)
- [Separator·인증](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:247), [worker 병렬화](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:536), [worker 후보 이동](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/worker.rs)
- [샘플 검색](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/search.rs), [좌표 하강법](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/sample/coord_descent.rs:64), [평가 버퍼](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/sep_evaluator.rs)
- [충돌 tracker](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs), [쌍 행렬](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/pair_matrix.rs:14), [충돌 SIMD](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/simd/overlap_proxy_simd.rs)
- [R&R](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/lns.rs:194), [Separator Swap](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/lns.rs:1881), [Escape](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/escape/pipeline.rs:289)
- [NFP 순서 탐색](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/exploration.rs:159), [NFP 순차 배치](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/nfp_lbf/placement.rs)
- [Compress](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/compress.rs:11), [Polish 조정](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/mod.rs:109), [Reclaim](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/reclaim.rs:142)
- [LP](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/lp.rs:211), [LP backend](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/lp_solver/mod.rs), [Slide](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/slide.rs), [Hole Fill](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/polish/holes.rs:436)
- [원본 검사·이번 수정](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap.rs:126), [원본 SIMD](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/original_overlap/kernels.rs), [원본 취소 회귀 테스트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/tests/original_overlap_trial.rs:161)
- [공통 최종 검증](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/structure/pipeline.rs:1294), [Native 인증 cache](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:802), [Native handle 실행 수명](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/src/ffi_step.rs:257)
