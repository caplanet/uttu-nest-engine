# 탐색·압축 SIMD와 Polish 시간 예산

기준 브랜치는 `v1_0_1_3`, 기준 커밋은 `4346371`이다. 검증을 마친 이번 변경은 사용자의 승인에 따라 같은 브랜치에 커밋·푸시한다. 이전에 롤백한 지연 정점 변환과 손실 하한 조기 종료는 다시 적용하지 않는다. 겹침 허용 0.08mm와 원단 침투 0.2mm를 유지한다.

## SIMD 적용 범위

탐색과 압축이 공유하는 충돌 트래커의 GLS 가중치 갱신에 SIMD를 추가했다. 최대 유효 손실 검색과 각 가중치의 증가·감쇠·상한 처리는 서로 독립적인 연산이다. 충돌 손실 합산의 순서를 바꾸지 않고 각 가중치 결과를 기존 스칼라 구현과 비트 단위로 비교한다.

쌍 충돌·원단 충돌 엔트리의 합이 128개 이상인 경우에 적용한다. x86_64의 AVX2 지원은 실행 중 확인하고, 지원하면 8개씩 처리한다. 나머지는 4개씩 처리하는 portable SIMD 경로를 쓴다. 작은 행렬은 기존 스칼라 연산을 사용한다. 행렬과 캐시의 구조, 난수, 충돌 판정 기준과 ABI는 바꾸지 않는다.

이미 벡터화된 기하 커널과 별개로 GLS 갱신을 선택했다. 합산 순서를 바꾸는 벡터화는 작은 부동소수점 차이로 탐색 경로를 바꿀 수 있어 적용하지 않았다.

## Polish 종료 조건

변경 전에는 개선 없는 라운드 수, 라운드 상한, 연속 검증 실패가 남은 시간과 관계없이 Polish를 끝냈다. 이제 활성 LP·슬라이드 패스가 있으면 유효 시간 예산 또는 외부 정지까지 실제 패스를 반복한다. 공개 `n_rounds` 필드는 이전 설정과 ABI를 읽기 위해 남기되 실행 상한으로 사용하지 않는다. WPF의 라운드 수 입력은 제거했다.

직접 코어 호출도 `PolishConfig.time_limit`을 지킨다. 부모의 기한이 더 짧으면 그 기한을 적용한다. 정지 요청을 계속 확인하며, 개선 없는 매우 짧은 라운드는 최소 1ms 간격으로 진행해 빈 반복의 CPU 소모를 제한한다. LP와 슬라이드를 함께 켜면 LP에 남은 시간의 60%를 배정해 슬라이드 실행 시간을 남긴다.

검증된 최선의 해를 유지하고, 검증 실패한 라운드는 되돌린 후 재시도한다. 비활성 상태, 실행할 패스가 없는 상태, 실행 가능한 것으로 인증할 수 없는 초기 해, 0초 예산은 즉시 반환할 수 있다. Reclaim이나 다른 단계의 시간 예산을 늘리지는 않는다.

## 측정과 검증

GLS 커널 비교는 동일 엔트리와 작업량으로 스칼라·SIMD의 실행 순서를 번갈아 바꾸어 5회 측정한다. 최종 가중치도 비트 단위로 비교한다. Release 최적화, LTO off, codegen units 16, 단일 논리 CPU에서 실행한 성능 측정이며 제품 DLL의 fat LTO 빌드와 구분한다.

최종 코드의 5회 시간 합계(`target/simd-polish-bench/gls-kernel-large-final.txt`): 각 경우에 쌍 엔트리 외 원단 엔트리 12개가 있다. 스칼라는 기존 연산을 보존한 테스트 오라클이다. 이 표는 GLS 갱신 함수만의 시간이다.

| 쌍 엔트리 | 충돌 손실 | 스칼라(s) | SIMD(s) | 비율 |
|---|---|---:|---:|---:|
| 128 | 없음 | 0.864108 | 0.221908 | 3.894 |
| 128 | 있음 | 0.830098 | 0.267442 | 3.104 |
| 12402 | 없음 | 0.577780 | 0.126755 | 4.558 |
| 12402 | 있음 | 0.672991 | 0.181305 | 3.712 |
| 131328 | 없음 | 0.567569 | 0.152340 | 3.726 |
| 131328 | 있음 | 0.658182 | 0.172777 | 3.809 |

전체 라운드 비교는 fu와 wsjk156 입력, 1·2·4워커, 시드 42/71/1234, 동일 작업 100라운드다. 배치, 손실, 평가 수, 이동 수와 워커 난수가 일치하는지 확인한다. 이전 측정에서는 느린 반복도 섞였다. 커널의 개선 비율을 전체 탐색·압축 속도나 배치 효율 개선 비율로 해석하지 않는다.

최종 라운드 비교도 다음처럼 빠른 경우와 느린 경우가 섞였다(`target/simd-polish-bench/gls-round-final.txt`). 푸시된 기준 DLL의 전체 단계 비교가 아니라 같은 라운드 구현에서 GLS 스칼라 오라클과 제품 경로를 비교한 것이다. fu의 작은 행렬에는 SIMD를 실행하지 않는다. 전체 단계의 확실한 속도 개선은 이 측정으로 입증되지 않았다.

| 입력 | 워커 | 오라클(s) | 제품 경로(s) | 비율 |
|---|---:|---:|---:|---:|
| fu | 1 | 0.319641 | 0.328883 | 0.972 |
| fu | 2 | 0.341568 | 0.322565 | 1.059 |
| fu | 4 | 0.488574 | 0.497444 | 0.982 |
| wsjk156 | 1 | 4.296107 | 4.176102 | 1.029 |
| wsjk156 | 2 | 3.546583 | 3.434811 | 1.033 |
| wsjk156 | 4 | 4.838488 | 4.883906 | 0.991 |

기존 `prepared_geometry::tracker_prepared_losses_match_reference_before_and_after_rotated_moves`는 이번 변경 전 tracker와 SIMD 모듈로 되돌려도 같은 실패를 재현했다. 저장된 대칭 쌍의 손실과 반대 인자 순서로 재계산한 손실이 1 ULP 다르다(`1108135482` / `1108135483`). 이번 작업에서 이 테스트나 기하 연산을 변경하지 않았다. 이 알려진 실패를 포함해 전체 테스트가 모두 통과했다고 주장하지 않는다.

통과한 검증:

- 최종 코어 단위 테스트: 741개 통과, 장시간·벤치마크 등 24개 제외.
- 최종 코어 통합 테스트: 44개 통과. postprocess 15, rigid_unit_lp 7, frozen_policy 4, parallel_optimization 7, stop_latency 3, explore_code_optimization 3, separation_edge_order 1, prepared_geometry 4. 위에서 재현한 기존 실패 1개는 마지막 통합 실행에서 명시적으로 제외했고, prepared_geometry의 기존 벤치마크 1개는 ignore 상태다.
- 최종 공개 FFI 테스트: 12개 통과. disabled_work 3, frozen_policy 4, low_core_phases 1(1·2·4워커 × 두 시드), step_phases 4.
- C# 호스트 테스트: 182개 통과. 오프라인 환경의 NuGet 취약점 정보 조회 경고는 발생했다.
- GLS의 4·8레인, 잔여 엔트리, 정상·빈 손실, NaN·무한대·음수·subnormal 및 사용자 학습률에 대해 기존 연산과 가중치·손실의 비트가 일치했다. 라운드 비교 18가지도 배치·손실·작업량·워커 난수가 일치했다.
- Polish는 개선 없는 고정 조각·강체 그룹, LP 단독·슬라이드 단독, 더 짧은 부모 기한, 0초 예산, 외부 정지와 유효하지 않은 초기 해를 검증했다. Legacy와 V2 FFI 모두 n_rounds=1인 경우에도 활성 Polish가 시간 예산을 사용한다.
- 읽기 전용 코드 리뷰에서 추가 수정 사항은 발견되지 않았다.

재실행 예:

```powershell
$env:CARGO_PROFILE_RELEASE_LTO='off'
$env:CARGO_PROFILE_RELEASE_CODEGEN_UNITS='16'
cargo test --offline -p swarm-core --release --features 'simd capmods adaptive structure-first' --lib --quiet
cargo test --offline -p swarm-core --release --features 'simd capmods adaptive structure-first' --lib gls_kernel_benchmark -- --ignored --nocapture --test-threads=1
cargo test --offline -p swarm-core --release --features 'simd capmods adaptive structure-first' --lib gls_simd_round_benchmark -- --ignored --nocapture --test-threads=1
```

성능 측정 시 CPU affinity를 고정하고 다른 빌드·테스트와 겹치지 않게 실행했다. 제품 빌드는 위 테스트용 profile 환경 변수 없이 기본 Release(fat LTO, codegen units 1)를 사용한다.

## 제품 빌드와 실제 DLL

`cargo build --offline --release --features 'simd,capmods,adaptive,structure-first,jni'`와 WPF Release 빌드를 완료했다. WPF 빌드의 기존 nullability·ReactiveUI 경고는 남아 있다. XML 파싱, 변경 핵심 파일의 rustfmt 검사와 diff 공백 검사도 통과했다.

실제 fat LTO DLL을 C# FullBench에서 직접 로드했다. fu, 시드 42, 겹침 0.08mm, 원단 침투 0.2mm, Explore 0.3초, Compress 0.3초, Polish 0.5초로 실행했다. 이전 R&R·swap·repair 플래그를 켠 기존 override를 사용해 무동작 호환성도 확인했다. 세 실행 모두 ABI·필수 export·조각 수·콜백 인증·최종 충돌 검사를 통과했다.

| 워커 | 요청한 Polish 예산(s) | 실제 Polish 시간(s) |
|---|---:|---:|
| 1 | 0.5 | 0.5022351 |
| 2 | 0.5 | 0.5023039 |
| 4 | 0.5 | 0.5023185 |

결과는 `target/simd-polish-smoke/workers-{1,2,4}.summary.json`에 저장했다. 시간 제한이 있는 확률적 실행이므로 이 실행의 최종 효율 차이로 SIMD의 배치 효율 향상을 주장하지 않는다.

새 DLL SHA-256: `251A7F9C7793852AD2342D496B93307AEE9A50C12D496D16EBB8E271C9F7A911`. Cargo 출력과 WPF 출력의 DLL 해시가 일치한다. 이번 작업의 소스와 보고서를 기준 커밋 `4346371` 이후의 새 커밋으로 저장한다.
