**Swarm 병렬화·최적화·안정성 수정 결과 — 2026-09-07**

대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`. 원본 검토 시점의 SHA-256과 비교한 뒤 수정했다. Git 저장소가 없는 작업 폴더이므로 커밋 대신 별도 변경 파일 해시와 패치를 남긴다. 원본 검토 보고서는 수정 전 기록으로 유지한다.

**구현 범위**

| 영역 | 반영 내용 | 안전 조건 |
|---|---|---|
| LP 접촉 제약 | x축 sweep의 후보 쌍을 최대 64쌍씩 평가. 4쌍 이상이며 정점 곱 합계가 16,384 이상이면 기존 Separator pool에서 병렬 평가 | indexed collect로 쌍과 행 순서 고정. LP 풀이·실제 이동은 순차 수행 |
| LP 후보 정리 | 정점-모서리 후보를 전부 저장·정렬하지 않고 방향별 강한 제약 상위 K개를 유지. 전체 행 상한은 배치 사이에서 조기 확인 | 기존 전체 수집→정렬→방향 중복 제거와 비트 단위 비교 테스트 |
| R&R | 피해 조각을 제거한 동일 레이아웃에 대한 초기 regret 샘플 평가를 병렬화. 조각별 RNG seed를 배정한 뒤 실행 | NFP 후보 준비는 캐시 소유자에서 순차 처리. 실제 재삽입·예약 재검증·relax는 순차. 취소 시 배치/트래커/RNG/tabu 복원 |
| 작업자 상태 | 폭 변경 후 작업자 전체 문제/트래커를 복제하던 비용 제거. 다음 sweep의 load에서 필요한 상태 전달 | master와 작업자가 동일 snapshot에서 출발하는 계약 유지 |
| NFP 준비 형상 | `PreparedLibrary`를 `Arc`로 공유하여 cache fork에서 깊은 형상 복제 제거 | 가변 캐시 상태는 작업자별로 유지 |
| Polish | 동일 폭에서도 좌측·하단 compactness가 개선된 배치 보존. 실제 pose 변경으로 무변화 종료 판정 | 폭 악화는 허용하지 않음. 무변화 종료를 최적성 증명으로 해석하지 않음 |
| 그룹 Polish | rigid unit 전체를 장애물에 대해 검사하고 평행이동. 그룹 활성 시 reclaim도 실행 가능 | 고정된 그룹 보존. 그룹 LP·개별 hole reconstruction은 계속 제외 |

LP 한 방향에서 후보 수를 M, 유지할 방향 수를 K라 하면 후보 저장은 O(M)에서 O(K)로 줄어든다. 탐색 비용은 정점-모서리 열거를 그대로 포함하며, 방향 선택은 O(MK)다. 따라서 모든 입력에 대한 시간복잡도 개선을 주장하지 않는다. 기본 K=8의 작은 상수와 할당·정렬 감소를 겨냥했다. 전체 행 저장은 설정 상한에 더해 처리 중인 유한 배치가 필요하다.

R&R은 작업자 스케줄에 관계없이 동일 작업 목록에 동일 seed를 사용한다. 이전 직렬 구현과 RNG 소비 방식이 달라지므로 **수정 전과 동일 seed가 같은 최종 배치를 낸다는 보장은 없다**. 시간 제한에 의해 실행 횟수까지 달라지는 전체 탐색의 재현성도 별개다.

**수정한 결함과 방어 처리**

1. **NFP 순서 탐색의 고정 조각 손실**: frozen 또는 rigid group이 있으면 제약을 보존하는 기존 탐색 경로로 전환한다. 최종 FFI 후보 채택 시에도 고정 조각의 item ID·회전·위치 멀티셋을 검사한다. 충돌 없는 해라도 잠금이 이동했다면 인증된 fallback으로 복원한다.
2. **좁은 해 복원 중 중간 상태 오류**: 과거 배치를 먼저 좁히고 snapshot을 복원하던 호출을 제거했다. `SPProblem::restore`로 대상 컨테이너와 배치를 함께 설치한다. 일반·step·grouped 단계 전환에 적용했다.
3. **중단 지연**: 좌표 하강·NFP 후보 열거·LP 접촉 생성·LP 모델 조립·이동 묶음에 취소 확인을 전달했다. microlp는 동일 simplex 상태를 50ms 조각으로 재개한다. 폭 변경과 LP 이동은 도중 취소 시 snapshot을 복원한다.
4. **조각마다 반복하던 전체 검증**: 배치 이동에서 트래커를 각 이동 후 일치시키되 전체 debug 검증은 묶음 경계에서 수행한다. 취소 확인은 조각 사이에 수행한다. 개별 이동 API의 검증은 유지한다.
5. **실행 간 축소율 혼선**: 핸들별 `RunControl`을 만들고 단계·그룹 작업자에 전달했다. C ABI·C# wrapper·WPF 적용 버튼을 연결했다.
6. **좌표 하강의 비정상 장기 실행**: debug assertion뿐이던 1,000회 평가 경계를 실제 Release 실행 상한으로 바꾸고 가장 좋은 평가 후보를 반환한다.
7. **그룹 Polish 폭 맞춤 assertion**: `fit_strip` 전에 실제 레이아웃 실행 가능성을 확인한다. frozen/frozen을 제외하는 트래커의 loss=0만으로 폭 맞춤의 전제를 충족했다고 간주하지 않는다. 최종 독립 검증은 유지한다.
8. **단계 진행 알림 누락**: 압축 baseline을 올바른 report type으로 전달한다. Structure 단계가 바뀔 때 첫 인증 baseline은 직전 단계의 throttle 시간 때문에 사라지지 않는다.

**C API 호환성과 사용**

추가 export:

```c
int swarm_set_handle_shrink_step(const void *handle, float step);
void swarm_clear_handle_shrink_step(const void *handle);
float swarm_get_handle_shrink_step(const void *handle);
```

실행 중 변경에는 이 핸들 API 또는 `SwarmStepOptimizer.TrySetShrinkStep`을 사용한다. 기존 `swarm_set_shrink_step`와 C# static setter는 **이후 생성하는 실행의 기본값**으로 의미가 변경됐다. 기존 심볼은 유지한다. C 구조체 필드는 추가하지 않았으며 ABI 버전은 15다. C# loader는 새 export의 존재도 검사하므로 이전 DLL을 잘못 로드하면 명시적으로 거절한다.

**검증 결과**

| 최종 검증 | 결과 |
|---|---|
| swarm-core 단위 테스트 | 717 통과, 13 ignored |
| core 통합 테스트: LP backend, NFP 병렬 결정성, 새 회귀, 그룹, 후처리, 중단 | 29 통과 |
| root/FFI 단위·통합 테스트 전체 | 136 통과, 3 ignored |
| C# 기존 테스트 | 75 통과 |
| 최초 추가 검증: lns_smoke | 6 통과. R&R 취소 시 배치/트래커/RNG 복원 포함 |

최종 Rust 검증 집계는 882 통과다. ignored 16개는 대부분 별도 실측 하네스이며 통과 수에 넣지 않았다. 그중 새 LP 접촉 측정 1개는 별도로 명시 실행했다. 테스트 프로필은 `opt-level=3`, debug assertions 활성, feature는 `simd capmods adaptive structure-first`다.

취소 성능은 다른 빌드가 끝난 뒤 `shirts` 입력으로 직렬 실행했다. 세 테스트 모두 1,500ms 이내 반환을 요구하며 Polish의 기존 6초 기준도 강화했다.

| 요청 이후 반환 지연 | 최종 측정 |
|---|---:|
| Explore | 150ms |
| Polish | 716ms |
| Reclaim | 79ms |

수정 전 같은 fixture의 reclaim은 4,249ms와 3,240ms로 1,500ms 기준을 두 번 실패했다. 이 수치는 debug assertions가 활성인 테스트 프로필의 결과이며 Release 전체 속도 향상률이 아니다.

LP 접촉 생성 단독 실측: 16개 128정점 다각형, 동일 672개 결과 행, 준비 실행 후 9회 중앙값. 작업자별 결과는 별도의 비트 단위 결정성 테스트로 확인했다.

| 작업자 수 | 중앙값 |
|---:|---:|
| 1 | 14.279ms |
| 2 | 9.941ms |
| 4 | 5.162ms |

이 합성 입력에서는 4개 작업자가 1개 대비 약 2.77배 빠르다. 범위는 접촉 생성에 한정되며 LP 풀이·전체 Polish·최종 nesting 시간은 포함하지 않는다.

재현 명령과 상세 로그는 `target/swarm-core-final-tests.log`, `target/swarm-ffi-integration-tests.log`, `target/swarm-contact-benchmark.log`, `target/swarm-csharp-tests.log`에 기록했다.

**빌드와 실제 연동 확인**

- `cargo build --offline --release --features "simd capmods adaptive structure-first"`: 성공. Cargo 출력의 linker 경고는 `.lib/.exp` 생성 안내다.
- `cargo check --offline --no-default-features`: 성공. feature를 끈 경로의 `ExactSolutionCache::from_certified` 미사용 경고 1개가 있다.
- WPF Release 빌드: 성공, 오류 0개·경고 14개. 경고는 nullability와 ReactiveUI source-generator 관련이며 `target/swarm-wpf-build.log`에 남겼다. WPF GUI를 직접 조작하는 테스트는 실행하지 않았다.
- 실제 Release DLL을 C# wrapper에서 로드한 별도 smoke: ABI/필수 export 검사, 두 핸들 초기화, 실행 중 축소율 변경, NaN 거절, 다른 핸들 불변, override 해제, phase 완료와 JSON export 모두 통과했다.
- DLL에서 읽은 handshake: ABI **15**, `CSwarmConfig` **776 bytes**, `CStructureFirstConfig` **116 bytes**, feature flags **15**. 추가한 세 export를 실제 로드하여 확인했다.
- `target/release/capswarmcore_100.dll`과 WPF Release 출력 폴더, C# smoke 출력 폴더의 DLL SHA-256이 일치한다.

Release DLL SHA-256:

```text
a403a78ba00864339e68b077851f9f47744bc5eae644f7eccf47ab8af670c230
```

**주요 재현 명령**

```powershell
cargo test --offline -p swarm-core --features "simd capmods adaptive structure-first" --lib --test parallel_optimization --test postprocess_phases --test place_groups --test stop_latency --test nfp_parallel_determinism --test lp_backends -- --test-threads=1 --nocapture
cargo test --offline --features "simd capmods adaptive structure-first" --tests -- --test-threads=1
cargo test --offline -p swarm-core --features "simd capmods adaptive structure-first" --lib contact_generation_parallel_benchmark -- --ignored --nocapture --test-threads=1
cargo build --offline --release --features "simd capmods adaptive structure-first"
dotnet test csharp/CAPlanetSwarmSample.WPF.Tests/CAPlanetSwarmSample.WPF.Tests.csproj --no-restore -c Release
dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj --no-restore -c Release
dotnet run --project target/swarm-native-smoke/smoke.csproj --no-restore -c Release
```

측정 명령은 다른 빌드·성능 테스트가 종료된 뒤 실행한다. C#의 처음 누락된 `project.assets.json`은 로컬 NuGet 캐시로 복원한 뒤 검증했다.

**변경 검토 자료**

- [31개 변경·추가 코드 파일의 패치](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_OPTIMIZATION_2026-09-07.patch)
- [변경 전후 파일 해시와 DLL 식별 정보](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_OPTIMIZATION_SOURCE_HASHES_2026-09-07.json)
- [수정 전 분석 보고서](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_PARALLELISM_REVIEW_2026-09-07.md)
- [새 회귀 테스트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/tests/parallel_optimization.rs)

원본 검토용 `swarm_review_frozen_probe.rs`도 수정 후 고정 조각 보존을 요구하도록 assertion을 변경했다. 새 통합 테스트에는 quadtree depth 0·4에서의 회귀가 포함되어 있다.

**남아 있는 범위와 해석 제한**

- microlp의 예산은 모델 조립도 포함한다. 예산 초과 또는 취소 뒤 도착한 해는 `Limit`로 버리므로, 과거 늦은 최적해를 채택하던 경계 사례에서 품질·시간 절충이 달라질 수 있다.
- microlp의 한 pivot, NFP의 단일 형상 연산, CDE의 단일 검사·최종 인증은 중간에 강제 중단하지 못한다. 50ms는 하드 실시간 중단 보장이 아니다.
- Clarabel 및 외부 LP 백엔드는 기존 backend 시간 제한을 사용한다. 이번 microlp 재개 방식과 같은 내부 취소 폴링을 추가하지 않았다.
- dense CollisionTracker의 O(N²) 구조 자체를 sparse로 바꾸지 않았다. 여기서는 중복 복제와 반복 합산·검증 비용을 줄였다.
- island 탐색, 여러 압축 split의 동시 시도, ALNS 보상 재설계, 전체 pool 다양성 재설계는 이번 수정에 포함하지 않았다. 현재 변경의 품질·비용 비교와 분리하여 검증해야 하는 알고리즘 교체에 해당한다.
- 전체 생산 마커 집합에 대한 최종 밀도 향상률이나 전체 실행 시간 배수는 측정하지 않았다. LP 접촉 생성 단독 실측과 취소 지연을 전체 solver 성능으로 확대 해석하지 않는다.
