# Swarm 원본 폴리곤 검사·보정 시험 — 2026-09-08

현재 상태는 **사용자 유지/롤백 결정 대기 중인 시험 작업본**이다. 기존 `piece_overlap_mm`를 탐색과 원본 검사·보정에 공통으로 사용한다. 1mm 또는 5mm를 고정하지 않았으며 Config 필드·기본값·ABI를 변경하지 않았다. 외부 Uttu.Nest 설치 폴더에는 배포하지 않았다.

## 결과와 판단 자료

156조각, 원본 정점 33,477개인 `data/input/wsjk156.json`에 PolySimpl 0.01을 적용했다. 허용치 1/5mm, seed 17/31/73, Init·Explore·Compress·Polish의 총 24개 SIMD 시험본 보정 결과를 별도 GEOS/Shapely 코드로 검사했다. 3,744개 배치에서 **허용치를 넘는 조각 쌍은 0개**였다. 최대 겹침 깊이는 1mm 설정에서 0.998933mm, 5mm 설정에서 4.998858mm였다. 별도 기능 시험으로 1/3/5mm 입력의 동적 적용도 확인했다.

성능 비용은 있다. 기준본·SIMD 적용 전·적용 후를 번갈아 측정한 결과, 정상 배치들을 바꿔가며 **전체 결과 검증**을 반복하면 기존 약 3.1~3.5ms가 SIMD 시험본 약 11.0~13.1ms로 증가했다. 이는 검증 호출 자체의 약 3.5~3.8배 비용이며, 전체 네스팅이 그만큼 느려진다는 뜻은 아니다. 같은 배치를 재검증하는 경우에는 캐시가 작동해 증가율이 약 6~8%였다. SIMD 추가만의 효과는 -2.4%~+1.7% 수준으로 작아 확실한 속도 향상을 주장하기 어렵다.

같은 시간 예산의 최종 사용 폭 변화는 기준본 대비 -2.28%~+0.99%였다. 이 6회 결과만으로 전체 속도 저하가 없거나 품질이 개선됐다고 일반화할 수 없다. **유지나 롤백은 자동으로 결정하지 않는다.**

## 구현 범위

- `swarm-core/src/original_overlap.rs`: 원본 외곽의 교집합을 구하고 허용치 절반만큼 안쪽으로 offset한 영역이 남는지 검사한다. 교차하는 변과 오목한 구간도 검사한다. 원본 bounds를 이용한 후보 축소, 변환·조각 쌍 캐시, 스레드별 최대 4개 인스턴스 캐시를 사용한다.
- 원본은 Swarm에 입력된 `shape_orig`다. Swarm에 전달되기 전 CAD 곡선이나 이미 삭제된 정점을 복원하는 기능은 아니다. 공개 좌표 변환으로 원본을 배치하고 0.0001mm 정수 격자를 사용한다. 경계 반올림에는 전체 깊이 기준 약 0.001mm의 보수적 여유를 둔다.
- 원본 허용치는 기존 importer가 보관한 조각별 음수 offset의 2배로 얻는다. 일반 입력에서는 모든 조각이 같은 `piece_overlap_mm`를 공유한다. 서로 다른 offset이 섞인 내부 인스턴스에서는 최소 허용치를 사용한다. 양수 겹침 허용이 없는 경로는 기존 검증을 유지한다.
- 위반 조각 또는 강체 그룹만 이동 후보를 만들고, 영향을 받는 원본 쌍을 먼저 평가한다. 원본 위반 면적 감소와 기존 탐색 CDE의 충돌 해소를 함께 요구한다. 실패·취소하면 입력 배치를 복구한다. Init에는 제한된 폭 확장 보정도 있다.
- Init, Explore, Compress, Polish, Separator, Escape 및 공통 Structure 결과 인증 경로에 연결했다. Native Legacy/그룹 Init과 JSON 콜백·내보내기도 같은 인증 규칙을 따른다.
- 잠금/잠금 겹침과 잠금 조각의 strip 이탈은 기존 예외를 유지한다. 잠금 조각은 보정으로 이동하지 않으며, 이동 가능한 조각과 잠금 조각의 쌍은 허용치 검사를 받는다. 그룹 상대 좌표도 유지한다.
- 복구 또는 strip 폭 변경이 배치 SlotMap 키를 바꾸는 경로에서 이전 키를 재사용하던 시험 구현 오류를 수정했다. 이동 직전에 현재 배치에서 키를 다시 찾는다.

## 동일 작업량 검증 비용

사용자 추가 요청에 따라 `original_overlap/kernels.rs`에 SIMD를 적용했다. 원본 좌표의 f64 회전·이동은 x86_64에서 런타임 AVX-512/AVX2 확인 후 각각 8/4개 좌표를 처리하며, 그 외에는 portable SIMD 2개 경로를 사용한다. 경계상자는 최대 4개씩 비교해 불필요한 교집합 호출과 캐시 조회를 줄인다. 기존 `simd` 빌드 기능을 사용하며 사용자 Config는 추가하지 않았다. SIMD 기능이 없는 빌드는 같은 스칼라 계산을 사용한다.

좌표 변환은 계산 순서를 유지하고 FMA 근사를 추가하지 않았다. 회전·잔여 정점·경계 허용치·비정상 좌표에 대해 SIMD/스칼라 일치 시험을 수행한다. 다각형 교집합과 안쪽 offset의 최종 판정은 Clipper에 남아 있으므로 검증 전체를 SIMD로 대체한 것은 아니다. SIMD 적용 전 결과는 시험 폴더의 `pre-simd-*`에 별도 보관한다.

동일한 벤치마크 소스를 변경 전 core와 시험 core에 각각 Release 빌드했다. 외부 좌표를 읽어 배치를 재구성한 뒤 기존 공개 `solution_validation_error`를 60회씩 3라운드 호출했다. 실행 순서는 기준본→SIMD 전→SIMD 후, SIMD 후→SIMD 전→기준본으로 총 6개 프로세스다. 표는 각 구현의 6개 라운드별 호출당 시간 중앙값이다. 기존 검증은 CDE, 시험본은 CDE와 원본 검사를 수행한다. 동일하게 보정된 24개 배치를 `certified-validation-input`에 고정해 사용했고 측정 중 다른 빌드/벤치마크는 실행하지 않았다.

| 보정된 정상 배치 | 허용치 | 기존 | 원본 검사(SIMD 전) | 원본 검사(SIMD 후) | SIMD 전 대비 | 기존 대비 | 통과 횟수(각 빌드) |
|---|---:|---:|---:|---:|---:|---:|---:|
| 같은 배치 반복 | 1mm | 3.192ms | 3.428ms | 3.382ms | -1.35% | 1.06배 | 360/360 |
| 12개 배치 순환 | 1mm | 3.141ms | 10.905ms | 11.041ms | +1.25% | 3.52배 | 360/360 |
| 같은 배치 반복 | 5mm | 3.389ms | 3.588ms | 3.650ms | +1.72% | 1.08배 | 360/360 |
| 12개 배치 순환 | 5mm | 3.455ms | 13.380ms | 13.060ms | -2.40% | 3.78배 | 360/360 |

동일 배치의 작은 시간 차이는 측정 변동을 포함한다. 변경 배치의 12개 순환은 모든 최적화 반복에서 모든 조각을 움직이는 실사용 상황과 동일하지 않으므로 전체 단계의 저하율로 환산하지 않는다.

보정 전 기준 배치도 별도로 비교했다. 기준 검증은 모두 승인했지만 SIMD 원본 검증은 1mm 순환에서 120/180, 5mm 순환에서 105/180만 승인했다. 이는 검사 기준이 강화되어 기존에 허용되던 일부 배치를 거절한다는 증거이며, 기존 결과가 모두 원본 기준으로 유효했다는 의미로 해석하면 안 된다.

별도로 원본 검사만 계측한 24개 SIMD 단계 결과는 최초 검사 중앙값 9.032ms(5.702~12.224ms), 같은 배치 캐시 검사 중앙값 0.088ms(0.062~0.132ms)였다. 보정은 위반이 없는 단계에서 대부분 약 0.1ms, 측정된 최대 62.221ms였다. 더 복잡한 데이터에서 보정 시간이 항상 이 범위에 머문다는 보장은 없다.

교차 실행 전의 별도 정상 배치 검증 프로세스에서 720회 합산 실행 시간은 기존 9.530초, SIMD 시험본 10.401초였다. 이 시간에는 import 등도 포함되고 측정 시점도 다르므로 검증 비용 판단은 위 교차 실행 표를 우선한다. 50ms 간격으로 측정한 peak RSS는 각각 97,857,536 / 95,346,688bytes였다. 이 한 쌍의 RSS 결과를 메모리 절감 효과로 주장하지 않는다.

## 같은 시간 예산의 네스팅 결과

worker 1, Explore 4초·Compress 3초·Polish 2초, 동일 입력·설정·seed를 사용했다. 초기 LBF 작업은 별도 계측했다. 표의 폭은 낮을수록 좋다. 기준본의 단계 결과는 보정 전이며, 시험본은 원본 인증/보정 경로를 포함한다.

| piece_overlap_mm | seed | 기존 최종 폭(mm) | 시험 최종 폭(mm) | 변화 |
|---:|---:|---:|---:|---:|
| 1 | 17 | 6900.212 | 6742.889 | -2.280% |
| 1 | 31 | 6711.174 | 6777.839 | +0.993% |
| 1 | 73 | 6851.257 | 6799.255 | -0.759% |
| 5 | 17 | 6563.275 | 6615.180 | +0.791% |
| 5 | 31 | 6615.001 | 6633.342 | +0.277% |
| 5 | 73 | 6626.716 | 6524.336 | -1.545% |

시간 제한 탐색은 실행 타이밍에 따라 시도 횟수와 결과가 달라진다. 전체 단계의 동일 반복 수 처리량은 이번에 측정하지 않았다. 긴 실제 작업, 더 큰 조각 수, 여러 worker, 다른 형상 데이터로 일반화하려면 추가 측정이 필요하다.

## 검증과 재현 자료

- 기능 회귀: 원본 허용치 1/3/5mm, 직사각형 초과 겹침 보정, 잠금 예외, 회전/변 교차, 취소 복구, 강체 그룹, 폭 확장 후 키 갱신.
- 기존 영향 범위: frozen policy, stop latency, overlap semantics(Compress/Polish LP on/off 포함), native Init·단계·콜백·export 단위시험.
- 독립 검사: Shapely 2.1.2로 입력 JSON의 원본 double 좌표와 공개 배치 좌표를 합성했다. 교집합과 half-budget bevel erosion으로 허용 여부를 판정하고, 별도 maximum inscribed circle 계산으로 깊이를 표시했다. UI 화면 검사를 대신하는 수치 시험이며 Uttu.Nest 실행 화면은 검사하지 않았다.
- `target/original-overlap-final-regression.log`, `target/original-overlap-constraints-tests.log`, `target/original-overlap-shared-tests.log`, `target/original-overlap-native-tests.log`에 시험 결과를 남겼다.

최종 SIMD 작업본 검증은 원본 보정 7개 + SIMD 일치 2개 + frozen 4개 + stop 3개 + overlap 3개 + native 101개로 **120개 통과**다. 기존 진단용 overlap 시험 1개는 ignored다. SIMD 없는 core의 `cargo check`도 통과했다. 전체 기존 테스트 전부를 재실행한 수치는 아니다. 최신 영향 범위 로그는 `target/original-overlap-simd-constraints.log`, SIMD 일치 로그는 `target/original-overlap-simd-tests.log`다.

Native Release DLL 빌드와 실제 `swarm_get_abi_info` 호출도 성공했다. ABI 15 / CSwarmConfig 776 / CStructureFirstConfig 116 / features 0x0F를 유지한다. 워크스페이스 `target/release/capswarmcore_100.dll`은 현재 시험 DLL이며, 별도 보관본은 시험 폴더의 `capswarmcore_100-trial.dll`이다. 시험 DLL SHA256은 `c5875d962df2227ca81757d9280817951c7055fa10d587e5578a578b581c01aa`다. Uttu.Nest 폴더 복사·실행 배포는 하지 않았다.

시험 자료 폴더: `target/original-overlap-trial-20260908-112417/`

- `source-baseline.zip`, `manifest.json`, `capswarmcore_100.dll`: 작업 전 소스와 DLL. 원본 DLL SHA256 `22b92a21db15ac67f297d4c92448a66d945674f12b5943119a520f612126d3f8`.
- `comparison-summary.json`: 위 표의 원시 집계.
- `simd-interleaved-comparison.json`: 교차 순서로 측정한 SIMD 전/후/기준본 비교(허용치·모드별 각 360회).
- `shared-wsjk-*.json/.log`, `independent-original-audit.json`: 공통 허용치 시험과 독립 검사.
- `baseline[-certified]-validation.*`, `candidate[-certified]-validation.*`: 동일 작업량 성능 결과.
- `trial-source-changes.json` 및 `docs/SWARM_ORIGINAL_OVERLAP_TRIAL_2026-09-08.patch`: 이번 제품 코드 변경 범위와 해시.
- `fixed-1mm-*`, `growth-*`는 요구사항 정정 전의 폐기된 1mm 고정 시험 증거다. 위 최종 공통 허용치 결과와 혼용하지 않는다.

실행 스크립트는 `scripts/run_original_overlap_trial.py`, `scripts/run_original_validation_bench.py`, `scripts/compare_original_overlap_simd.py`, `scripts/audit_original_overlap_trial_results.py`, `scripts/summarize_original_overlap_trial.py`, `scripts/check_original_overlap_trial_abi.py`다. 복원 스크립트나 자동 롤백은 실행하지 않았다. 이후 사용자가 롤백을 선택하면 기준본과 이번 변경 해시를 대조해 이번 작업만 복구해야 한다.
