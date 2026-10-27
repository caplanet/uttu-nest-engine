# Explore 코드 최적화 구현 결과

대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`, 2026-09-07.

## 결과

Config를 변경하지 않고 후보 평가, 작업 버퍼, 승자 상태 반영, 손실 조회 비용을 줄였다. `Cargo.toml`, `Cargo.lock`, `swarm-core/Cargo.toml`, `config.rs`, `consts.rs`는 작업 시작 시점과 바이트 단위로 동일하다. 작업자 수·샘플 수·시간 제한·축소율·회전 범위·탐색 연산자의 설정이나 기본값을 변경하지 않았다.

Release에서 같은 입력·seed·초기 배치·Config로 분리 연산 3회를 비교했다. 세 번씩 번갈아 실행한 중앙값 기준으로 WSjk156은 **11.5~13.0%**, fu는 **4.0~17.5%** 시간이 감소했다. 비교한 배치 좌표·회전·폭·손실·작업자 RNG 상태는 수정 전후 일치했다. 전체 Explore/Compress/Polish 완료 시간의 측정은 아니다.

## 구현 내용

1. **상한 기반 SIMD를 후보 평가에 연결했다.** [simd/mod.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/simd/mod.rs:52)의 역산 상한은 빠른 비교용 힌트로만 사용한다. 실제 탈락 여부는 기존 가중치와 합산 순서를 그대로 적용한 정방향 f32 계산으로 확인한다. 상한과 같은 값, 바로 아래·위의 인접 부동소수점 값, 여러 가중치·누적 손실에 대해 기존 계산과 비교했다.
2. **신규 충돌만 처리한다.** [specialized_jaguars_pipeline.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/specialized_jaguars_pipeline.rs:189)에 pending 목록을 추가했다. 기존 SecondaryMap의 슬롯 순서로 신규 묶음을 정렬해 합산 순서를 보존한다. 삭제, 재삽입, 초기화, 조기 탈락 후 재사용 시 캐시를 올바르게 재구성한다.
3. **작업자별 평가 버퍼와 CT 충돌 수집 버퍼를 재사용한다.** [sep_evaluator.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/sep_evaluator.rs:19)는 도형 정점·충돌 맵·pending 목록·SIMD SoA 버퍼를 검색 종료 시 작업자 풀에 돌려준다. 다른 조각에서는 도형 크기와 메타데이터를 갱신하며 layout/CT 참조는 보관하지 않는다. NFP 검색에서 일반 검색으로 폴백해도 같은 풀이 재사용된다. CT의 임시 collector는 snapshot에 복제하지 않는다.
4. **승자 problem/CT를 함께 교환한다.** [separator.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/optimizer/separator.rs:479)에서 승자를 다시 저장·복원하고 충돌 행렬을 복사하는 대신 소유권을 교환한다. RNG와 NFP 캐시는 각 작업자에 유지하고, 다음 반복에서는 기존처럼 모든 작업자가 master 상태를 load한다. 설정된 작업자 수와 동률 선택 규칙은 그대로다.
5. **기존 합산 순서를 보존하는 지연 캐시를 추가했다.** [tracker.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/quantify/tracker.rs:17)에 행별 손실·가중 손실·전체 손실 캐시를 추가했다. 변경된 항목에 영향을 받는 캐시만 무효화하고 다음 조회에서 원래 순서로 합산한다. 증분 덧셈/뺄셈으로 인한 오차 누적을 피한다. 외부 행렬/컨테이너 항목 수정은 캐시 무효화 accessor를 거치도록 관련 escape·그룹 테스트 경로를 함께 수정했다. GLS 가중치 갱신·복사·rollback·고정 항목 재구축을 검증했다.
6. **평가 횟수 누락을 수정했다.** 검색이 끝난 직후 `n_evals`를 집계하므로 이후 중단되거나 후보가 없는 경우도 수행한 검색 비용이 통계에 포함된다. 실제 탐색 횟수를 늘리거나 줄이는 변경은 아니다.

새 캐시 때문에 Rust 내부의 직접 필드 접근 두 종류가 accessor 호출로 바뀌었다. C 구조체·C# marshaling·외부 C ABI에는 변경이 없다.

## 성능 비교

| 입력 | seed | 수정 전 중앙값(ms) | 수정 후 중앙값(ms) | 시간 감소 |
|---|---:|---:|---:|---:|
| fu | 17 | 9.45 | 8.60 | 9.0% |
| fu | 42 | 19.61 | 16.19 | 17.5% |
| fu | 83 | 12.07 | 11.59 | 4.0% |
| WSjk156 | 17 | 1306.75 | 1136.45 | 13.0% |
| WSjk156 | 42 | 191.76 | 167.70 | 12.5% |
| WSjk156 | 83 | 673.93 | 596.67 | 11.5% |

- 기존 Release 라이브러리로 수정 전 실행 파일을 만들고, 새 Release 라이브러리로 수정 후 실행 파일을 만들었다. 두 실행 파일을 순서를 바꾸어 각각 3회 실행했다. 측정 중 별도의 빌드/회귀 테스트는 실행하지 않았다.
- 각 입력·seed에서 기본 LBF 초기 배치를 만든 다음 기본 shrink step으로 폭을 줄여 `Separator::separate`와 rollback을 3회 수행했다. 측정에는 폭 변경·분리·복원이 포함되고 import/LBF 생성 및 최종 독립 검증은 제외된다.
- 6개 입력/seed 조합의 각 3개 중간 결과에 대해 폭, 손실, 모든 조각의 좌표·회전 비트와 실행 종료 후 작업자 RNG 상태를 비교했다. 3회 재실행에서도 일치했다.
- 최종 결과 6개는 공개 좌표에서 도형을 다시 생성하는 `solution_validation_error()`를 모두 통과했다. 수량·허용 회전·컨테이너 경계·조각 간 충돌을 검사하는 기존 검증기를 그대로 사용했다.
- WSjk156 seed 83은 수정 전후 모두 `Layout::is_feasible()`가 false이지만 좌표에서 재구성한 독립 검증은 통과했다. 이 판정 차이의 세부 원인까지 확정한 것은 아니다. 최적화로 새로 발생한 결과가 아니며, native 판정과 독립 검증을 측정 기록에서 구분했다. 검증 정책이나 허용오차는 변경하지 않았다.
- 수십 ms 이하의 fu 측정은 실행 환경 노이즈의 영향을 더 크게 받는다. 위 수치는 해당 입력·seed·기기에서의 분리 경로 측정이며 전체 솔버의 가속 배수로 일반화하지 않는다.

재현 소스: [swarm_explore_code_probe.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/swarm_explore_code_probe.rs), [실행 스크립트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/run_swarm_explore_code_probe.ps1). 초기 수정 전 실행 파일을 만들 때에는 수정 전의 대응 Release rlib가 필요하다. 현재 스크립트의 label만 바꿔 실행한다고 이전 소스가 복원되지는 않는다.

## 검증

총 **891건 통과, 실패 0건, 기존 제외 16건**이다. 서로 다른 feature 조합에서 실행한 동일 회귀 테스트 3건을 각각 포함한 실행 건수다.

| 검증 | 통과 | 제외 | 로그 |
|---|---:|---:|---|
| SIMD 포함 코어 단위 테스트 + 신규 회귀 + LNS | 728 | 13 | `target/explore-code-tests.log` |
| SIMD 없는 기본 빌드의 신규 회귀 | 3 | 0 | `target/explore-code-scalar-tests.log` |
| 루트/FFI 테스트 | 136 | 3 | `target/explore-code-ffi-tests.log` |
| 병렬화·그룹·후처리·취소·NFP 결정성 | 24 | 0 | `target/explore-code-phase-tests.log` |

상한 경계, 버퍼 재사용, 충돌 목록 삭제·재삽입, 손실 캐시 무효화, 승자 교환을 검증했다. 승자 교환 회귀 테스트는 12회 반복에서 기존 snapshot/restore 방식과 이동 및 RNG 상태가 일치하는지 확인한다. 기존 frozen·warm start·그룹 강체 관계·취소 후 rollback·FFI JSON/콜백 테스트도 통과했다. 설정된 테스트의 시간 제한/샘플 수를 성능 개선 목적으로 변경하지 않았다.

```powershell
cargo test -p swarm-core --features "simd capmods adaptive structure-first" --lib --test explore_code_optimization --test lns_smoke --offline
cargo test -p swarm-core --no-default-features --test explore_code_optimization --offline
cargo test -p swarm --features "simd capmods adaptive structure-first" --tests --offline -- --test-threads=1
cargo test -p swarm-core --features "simd capmods adaptive structure-first" --test postprocess_phases --test place_groups --test stop_latency --test nfp_parallel_determinism --test parallel_optimization --offline -- --test-threads=1
cargo build -p swarm --release --features "simd capmods adaptive structure-first" --offline
```

Release DLL은 [target/release/capswarmcore_100.dll](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/release/capswarmcore_100.dll)에 생성했다. 실제 DLL을 로드한 handshake는 ABI 15, Config 776 bytes, Structure Config 116 bytes, feature flags 15로 확인했다. Release 빌드는 성공했으며 링크 라이브러리 생성에 관한 경고 1건이 있었다. WPF 배포 폴더의 DLL 교체나 C# 애플리케이션 빌드는 이번 범위에 포함하지 않았다.

## 범위와 기록

이번에는 검토 보고서의 우선순위 1~5와 평가 횟수 집계를 구현했다. 후속 제안이었던 전체 도형의 지연 변환, 회전별 geometry 캐시, 조각 이동의 추가 병렬화는 포함하지 않았다. 후보 순서나 탐색 전략을 바꾸는 방법도 적용하지 않았다.

- [변경 패치](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_EXPLORE_CODE_OPTIMIZATION_2026-09-07.patch): 이번 작업의 기존 파일 11개와 신규 회귀 테스트 1개만 포함한다.
- [소스/설정/DLL 해시](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_EXPLORE_CODE_OPTIMIZATION_HASHES_2026-09-07.json): 이전 작업의 변경과 구분할 수 있도록 작업 시작 시점 대비 기록했다.
- [성능 측정 JSON](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_EXPLORE_CODE_OPTIMIZATION_BENCHMARK_2026-09-07.json): 입력 해시, 개별 실행 시간, 중앙값, 일치 여부, 독립 검증 결과를 포함한다.
