# Explore 코드 최적화 추가 검토

검토일: 2026-09-07. 대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`의 현재 Rust 소스.

## 결론과 범위

추가 코드 최적화를 진행할 근거가 있다. 우선 후보 하나를 평가하는 비용을 줄이고, 이후 반복적인 상태 복사와 손실 합산을 줄이는 순서가 적절하다. 다만 현재 소스의 비용 구조를 확인한 결과이며, 각 항목의 실제 시간 점유율이나 전체 Explore 가속 배수는 이번 검토에서 측정하지 않았다.

Config 조정은 개선안에서 제외한다. 작업자 수, 샘플 수, CD 횟수, 탐색 시간, shrink step, 회전 범위, 연산자 활성화 설정을 변경하거나 기본값·상수를 바꿔 같은 효과를 내는 방법도 포함하지 않는다. 목표는 같은 설정과 탐색 조건에서 중복 연산을 제거하는 것이다. 이번 작업은 검토이며 실행 소스와 Config는 변경하지 않았다.

이전 구현 기록의 변경 파일 31개를 SHA-256으로 다시 확인했으며 기록된 수정 후 해시와 모두 일치했다. 이전 LP 접촉 생성 벤치마크는 해당 연산의 측정이므로 전체 Explore 성능 개선 근거로 사용할 수 없다.

## 우선순위

| 순서 | 대상 | 현재 확인한 비용 | 코드 개선 방향 | 주의점 |
|---|---|---|---|---|
| 1 | 충돌 손실 평가 | 상한을 넘을 후보도 충돌 쌍 내부 계산을 끝까지 수행 | 기존 bounded SIMD 커널을 평가 경로에 연결 | 경계값에서 유효 후보를 잘못 버리지 않아야 함 |
| 2 | 충돌 목록 처리 | 새 충돌이 추가될 때 기존 목록도 다시 순회 | 신규 충돌 목록과 처리 커서를 분리 | 기존 합산 순서·중복 제거·삭제 계약 유지 |
| 3 | 평가기 작업 버퍼 | 조각 검색마다 도형 복제와 collector 생성 | 작업자별 scratch 버퍼 재사용 | 배치 변경 후 오래된 참조·키 재사용 금지 |
| 4 | 작업자 결과 반영 | 매 반복 승자 snapshot 생성·restore·CT 복사 | 승자 problem/CT 소유권 교환 검토 | problem과 CT를 일관된 쌍으로 교환 |
| 5 | 충돌 손실 집계 | 조각별 O(N), 전체 O(N²) 합산 반복 | 충돌 존재 개수 및 손실 합계 캐시 | GLS 갱신·rollback·부동소수점 오차 관리 |
| 후속 | 후보 도형 변환 | 조기 탈락 후보도 전체 도형부터 변환 | pole 선행 평가와 지연 변환, 동일 회전 재사용 | 정확한 충돌 판정 및 회전 제약 유지 |

이는 코드 구조에 따른 착수 우선순위다. 실측된 비용 순위는 아니다. 조각 수, 정점 수, 충돌 밀도, 실제로 선택되는 탐색 경로에 따라 순서가 바뀔 수 있다.

## 1. 상한 기반 SIMD를 실제 평가 경로에 연결

근거: [specialized_jaguars_pipeline.rs:184](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/specialized_jaguars_pipeline.rs:184), [simd/mod.rs:40](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/simd/mod.rs:40).

평가기에는 이미 후보 손실 상한이 전달된다. 그러나 `calc_weighted_loss()`는 `quantify_collision_poly_poly_simd()`로 충돌 쌍의 손실을 끝까지 계산한다. 상한 기반 함수 `quantify_collision_poly_poly_simd_bounded()`는 구현과 테스트가 있지만, 현재 소스에서 실제 호출은 해당 함수의 테스트에만 있다.

남은 가중 손실 예산을 양의 유한한 쌍 가중치로 나눠 bounded 커널에 전달하면, 탈락이 확정된 후보의 pole 계산을 도중에 끝낼 수 있다. 신규 충돌을 여러 개 발견한 경우에도 예산을 순차적으로 소진하도록 평가 경로를 연결해야 한다. 상한이 무한대이거나 전제 조건이 성립하지 않으면 기존 계산을 유지한다.

핵심 검증은 상한 바로 아래·동일·바로 위의 값이다. 제곱·제곱근을 포함한 상한 역산은 f32 반올림으로 인해 너무 이른 탈락을 만들 수 있다. 보수적인 경계 처리와 경계 부근의 기존 함수 재평가가 필요하다. 기존 단위 테스트 통과만으로 탐색 경로 연결의 동등성이 보장되지는 않는다. SIMD 비활성 빌드에는 이 항목의 가속 효과가 없다.

## 2. 신규 충돌만 처리하는 구조

근거: [specialized_jaguars_pipeline.rs:151](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/specialized_jaguars_pipeline.rs:151), [specialized_jaguars_pipeline.rs:163](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/specialized_jaguars_pipeline.rs:163).

현재 `loss()`는 새 충돌이 있으면 `SecondaryMap`의 값을 순회하고 발견 인덱스로 필터링한다. 이미 계산한 충돌의 손실 자체는 다시 계산하지 않지만, 목록 순회는 반복된다. 충돌이 조금씩 추가되는 후보에서는 누적 순회량이 커진다.

중복 확인용 맵과 신규 충돌용 연속 배열을 분리하고 처리 커서를 두면 이 반복을 줄일 수 있다. 매 후보의 맵 초기화 비용도 실제 프로파일에서 크다면 세대 번호 방식의 초기화를 검토할 수 있다.

단순히 삽입 순서로 합산하도록 바꾸면 현재 맵 순회 순서와 달라져 수치 결과가 변할 수 있다. 신규 묶음의 기존 순서를 보존하고 `remove_by_key()`의 캐시 무효화, 자기 자신 제외, 중복 충돌 처리도 유지해야 한다. 1번 개선과 함께 검증할 수 있지만 각각의 효과를 분리 측정하는 것이 좋다.

## 3. 검색 호출 사이에서 작업 버퍼 재사용

근거: [sep_evaluator.rs:22](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/sep_evaluator.rs:22), [worker.rs:196](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/worker.rs:196), [tracker.rs:71](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:71).

`SeparationEvaluator::new()`는 검색할 조각의 도형을 복제하고 collector를 생성한다. 검색 한 번 안에서는 버퍼를 재사용하지만 다음 조각 검색에서는 새 평가기가 만들어진다. 실제 이동 후 CT 재계산도 `BasicHazardCollector`를 새로 만든다.

작업자에 도형·충돌 목록·SIMD SoA의 소유 버퍼를 두고, 평가 시 현재 layout/CT를 짧게 빌려 쓰도록 분리하면 할당과 도형 복제 비용을 줄일 수 있다. 서로 다른 조각의 정점 수에 맞춰 capacity를 재사용하되 모든 유효 데이터는 갱신해야 한다. 작업자 사이에 공유 락을 두는 구조는 피한다.

SoA는 매 후보마다 pole 좌표와 반지름을 복사한다. 반지름은 강체 변환에 불변이므로 같은 조각의 검색 동안 재사용할 수 있지만, 조각 전환 시 갱신을 보장해야 한다. 이 부분은 할당 횟수와 복사량을 계측해 효과를 확인한다.

## 4. 승자 상태 반영 비용 제거

근거: [separator.rs:391](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:391), [worker.rs:98](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/worker.rs:98), [tracker.rs:122](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:122).

매 반복마다 master 저장 → 각 작업자의 restore/CT 복사 → 병렬 이동 → 승자 저장 → master restore/CT 복사가 이어진다. CT 복사는 이미 버퍼를 재사용하므로, 재할당 제거를 새 개선으로 제안할 단계는 아니다. 남은 것은 복사 자체와 snapshot/restore 비용이다.

승자의 `prob`와 `ct`를 master와 함께 교환하면 마지막 저장·복원·복사를 줄일 가능성이 있다. 작업자 RNG와 NFP 캐시의 소유권은 별도로 검토해야 하며, 다음 load에서 캐시가 새 배치와 일치하는지 확인해야 한다. 기존 작업자 수가 1인 경로에 한해서는 동일 동작의 직접 실행 경로도 검토할 수 있다. 작업자 수를 1로 바꾸자는 제안은 아니다.

현재 쌍 행렬은 대각선을 포함해 `N(N+1)/2`개의 8바이트 항목을 가진다. 예를 들어 N=156이면 97,968바이트, N=1000이면 4,004,000바이트다. 작업자 W개로 load하고 승자를 다시 복사하면 쌍 행렬만 `(W+1)`회 복사한다. 이는 저장 구조로 계산한 복사량이며 실제 소요 시간 측정값은 아니다. 조각 수가 작으면 후보 평가 비용에 비해 미미할 수 있다.

## 5. 충돌 여부와 손실 합계 조회 개선

근거: [tracker.rs:192](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:192), [tracker.rs:249](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:249), [tracker.rs:269](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:269).

`get_loss()`는 행을 O(N)으로 합산한다. 작업자는 이동 대상 수집과 이동 직전에 이를 호출한다. 전체 손실 조회는 O(N²) 행렬을 순회하며, GLS 갱신도 최대 손실 검색과 가중치 수정으로 두 번 순회한다.

먼저 이동 대상 판정에 필요한 충돌 존재 여부를 조각별 양의 손실 항목 개수로 관리하는 방식을 검토할 수 있다. 이후 비가중 행 합계·전체 합계를 이동 시 증분 갱신하고, 가중 합계는 GLS 갱신 시 함께 계산하는 접근이 가능하다. 증분 합산의 잔여 오차가 `loss == 0` 판정을 오염시키지 않도록 충돌 존재 여부를 독립적으로 유지해야 한다.

현재 CT 필드는 공개되어 있다. 캐시를 추가하려면 직접 수정 경로, 복원, 고정 항목 처리, 가중치 갱신을 모두 포함한 갱신 계약을 먼저 정리해야 한다. 단순 희소 행렬 치환은 권하지 않는다. 현재 충돌이 없는 쌍에도 이전 GLS 가중치가 남고 매 반복 감쇠하므로, 항목을 버리면 탐색 의미가 바뀐다. 지연 감쇠 역시 반복 곱셈과 수치 결과가 달라질 수 있어 후속 연구 항목이다.

## 후속: 전체 도형 변환을 늦추기

근거: [specialized_jaguars_pipeline.rs:36](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/specialized_jaguars_pipeline.rs:36).

현재는 전체 `shape_buffer.transform_from()`과 pole SoA 적재 후 조기 종료용 pole 검사를 시작한다. 따라서 일찍 탈락할 후보도 앞부분의 변환 비용은 낸다. pole과 손실 정량화에 필요한 데이터만 먼저 변환하고, 살아남은 후보에서 꼭짓점·edge용 데이터를 완성하는 두 단계 평가를 검토할 수 있다.

이미 회전한 도형을 같은 회전의 평행 이동 후보에서 재사용하는 것도 후보지만, 연속 회전·tilt를 임의 각도로 양자화해서는 안 된다. containment와 기존 충돌 검출 불변식을 유지해야 하므로 앞 항목보다 변경 범위와 검증 부담이 크다.

## 병렬화 판단

현재 작업자별 독립 이동은 이미 병렬이다. 같은 layout의 조각 이동을 단순 `par_iter`로 바꾸면 앞 조각의 이동 결과를 다음 조각이 보는 현재 알고리즘과 달라진다. 동일 Config를 유지하더라도 알고리즘의 결과를 바꾸는 변경이다.

추가 병렬화가 필요하다면 읽기 전용 snapshot에서 후보를 평가한 뒤 순서대로 다시 검증·반영하는 구조를 검토해야 한다. 다만 재검증 비용과 이전 후보의 무효화 비율이 높을 수 있다. 기존 작업자 병렬화 안에 또 작은 작업을 병렬화하면 스케줄링·동기화 비용이 커질 수 있으므로, 먼저 후보당 연산과 복사를 줄인 뒤 측정 결과에 따라 결정한다. 앞서 구현한 R&R 후보 평가 병렬화나 LP 접촉 생성 병렬화를 새 제안으로 중복 계산하지 않는다.

## 검증 방법과 현재 한계

1. Release에서 실제 적용 Config 전체를 동일하게 고정하고, 동일 입력·동일 초기 배치·동일 seed를 사용한다. 초기 배치 생성 시간과 Explore 시간을 분리한다.
2. 후보 평가 내부, worker load, worker 이동, 승자 반영, CT 갱신, snapshot 시간을 계측한다. 병렬 작업자의 CPU 시간을 더한 값을 전체 wall time 점유율로 표시하지 않는다.
3. 현재 `worker.rs`의 `total_evals`는 검색이 취소되거나 결과가 없는 분기에서는 증가하지 않는다. 검색 호출 직후 평가 횟수를 계상하는 계측 보완이 필요하다. 성능 비교에는 시도한 후보 수, 탈락 수, pole 연산량, 할당 수를 함께 기록한다.
4. 고정 반복 수에서는 손실·선택 후보·이동 순서의 동등성을 확인하고, 동일 시간 예산에서는 처리량과 최종 폭을 여러 seed에서 함께 비교한다. RNG 소비 순서와 동률 선택 규칙을 보존한다.
5. 별도의 정확한 충돌 검사, 전체 배치 수, frozen 항목 위치, 그룹 강체 관계, 취소 후 복원 상태를 확인한다. 빠르게 종료한 것만으로 성능 개선을 판정하지 않는다.

이번 검토는 현재 호출 경로, 연산 복잡도, 이전 구현 파일 해시를 확인했다. 실행 소스를 수정하지 않았으며 새로운 벤치마크·빌드·회귀 테스트는 수행하지 않았다. 소스 주석의 과거 실행 시간 비율이나 다른 단계의 벤치마크를 현재 Explore 측정값으로 인용하지 않았다.
