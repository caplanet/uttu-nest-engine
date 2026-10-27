# Swarm 단계 개선 적용 및 비교 결과

대상: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`  
기준: 2026-09-09 개선 작업 직전 소스. 전날 적용한 원본 검사 중지 수정은 기준 버전에도 포함되어 있습니다.

## 적용 내용

설정값을 낮추거나 기능을 꺼서 실행 시간을 줄이지 않았습니다. `piece_overlap_mm`, 단순화 설정, 잠금 예외, 최종 정밀 검사 기준, Native ABI는 유지했습니다.

| 개선 | 실제 적용한 내용 | 안전 조건 |
|---|---|---|
| 중복 인증 제거 | thread별 최근 성공 인증 4개 재사용 | 전체 pose/key/ID, 수량, 회전, 도형 Arc, 품질 정책, strip, 잠금, 완전/부분 검사 모드 비교. 해시 충돌에 의존하지 않음 |
| 충돌 행렬 복사 절감 | snapshot과 worker가 불변 행렬을 공유하고 최초 쓰기 시 분리하는 copy-on-write 적용 | worker RNG·키 매핑·가중치·합계 cache는 독립 유지. SIMD에 필요한 연속 메모리 유지 |
| Polish 중간 개선 보존 | LP/Slide 후 후속 pass가 남아 있고 실제 길이가 줄면 원본 보정·인증 checkpoint 생성 | 중지 후에는 시작하지 않음. 직전 검사 비용의 2배 이상 시간 여유가 있을 때 시도. 검증된 결과만 보관·공개 |
| Swap/R&R 성과 반영 | 교란 시작부터 후속 분리·인증까지 걸린 시간과 검증된 길이 개선을 보상에 반영 | 기존 충돌 감소 보상 유지. 비용 배율 제한. 취소된 미완료 시도는 실패 표본으로 학습하지 않음 |
| Compress 후보 개선 | 무작위 후보를 유지하면서 오른쪽 조각의 경계에서 추가 축소 지점 제안 | 저렴한 경계 통과량 점수로 선택. 실제 채택은 기존 분리·정밀 검사로 판정 |
| Hole Fill 후보 개선 | 오른쪽 끝에 있는 조각 우선, 같은 끝이면 작은 조각 우선. 격자 경로는 제한된 복수 pocket 후보 중 조각의 오른쪽 끝이 가장 작아지는 위치 선택 | 실제 위치 평가·원본 검사 유지. NFP 경로에도 동일한 조각 우선순위 적용 |
| LP 접촉 후보 축소 | 연속 변 8개 단위 bounding box로 한꺼번에 후보 제외 | 원래 정점/변 순서와 동점 처리 유지. 기존 직렬 참조와 접촉 행 동등성 테스트 |
| 원본 쌍 정밀 검사 병렬화 | 큰 입력의 비싼 미계산 쌍을 최대 128개까지 병렬 선계산 후 순차 cache 병합 | 각 작업의 Clipper 입력/출력 독립. 작은 입력·적은 후보는 직렬. 최종 판정은 기존 검사 경로 |
| 계측 | 단계 경과 시간, 인증 호출/cache hit/시간, worker 작업·load 시간, 행렬 공유/복사 byte, ALNS 비용·길이 성과 | 후보 하나마다 전역 lock을 잡지 않음. thread별 계측 후 worker byte 합산 |
| 견고성 | 행렬 크기 checked 산술, Escape 정체 신호에서 미분리 배치의 가짜 효율 개선 제외 | 기존 최선 해와 정확성 기준 유지 |

### 인증 재사용이 검사를 생략해도 되는 이유

인증된 배치와 지금 배치의 판정 입력을 모두 비교합니다. 좌표, 허용 회전, 조각 수, 원본 또는 검색 도형, strip 또는 잠금 정책이 달라지면 다시 검사합니다. 원본 Arc를 소유해 해제된 주소가 다른 도형에 재사용되는 경우도 방지합니다. 그룹 상대 pose와 잠금 pose의 **기준 해 대비 보존 검사**는 Separator에서 매번 그대로 수행합니다.

따라서 “충돌 비용이 0이면 이미 검증된 것으로 간주”하는 변경은 아닙니다. 수량이 부족한 부분 해에 준 인증을 완전 해 검사에 재사용하지도 않습니다.

### 복사 최적화의 범위

이번에는 충돌 행렬 전체를 copy-on-write로 공유합니다. 모든 worker가 실제로 행렬을 수정하면 각 worker의 첫 쓰기 시 전체 행렬 복사가 필요합니다. 그러므로 공유 byte를 그대로 절감 byte라고 계산하면 안 됩니다.

**CDE 전체 상태의 변경분 동기화나 행렬의 페이지 단위 복사는 이번 적용에 포함되지 않습니다.** 이는 키 재발급·GLS rollback과 더 깊이 연결되므로, 이번 계측으로 남은 비중을 확인한 뒤 진행할 후속 과제입니다. 원본 cache miss별 변경 검사, LP backend 기저 재사용, Hole Fill 연결 성분의 국소 갱신도 후속 최적화입니다.

### Polish 종료 시 동작

예를 들어 LP가 길이를 줄였고 원본 인증까지 끝났다면, 이후 Hole Fill 중 중지하더라도 그 LP 개선을 반환할 수 있습니다. 아직 인증하지 않은 작업 중 좌표는 반환하지 않습니다. 새 회귀 테스트는 실제 콜백에서 중지를 요청하고, 최종 좌표가 공개된 개선 좌표와 같은지 확인합니다.

## WPF 진행 시간 표시

`AdaptiveAlgorithm2-fable`의 Stop 버튼 옆 표시를 현재 Sample에 옮겼습니다. **Init 시간은 제외**하고 Explore → Compress → Polish 또는 Structure 실행 시간을 0.1초 간격으로 갱신합니다. 완료·중지·오류 후 마지막 값은 남고, 새 Run에서 초기화합니다. 화면 폐기 시에도 DispatcherTimer를 중지합니다. 24시간 이상 실행 시에도 누적 시간이 표시되도록 시간 형식을 보완했습니다.

설정·내보내기·최적화 흐름 전체를 복사하지 않고 해당 UI와 타이머 수명 처리만 적용했습니다. 이 UI 변경은 아래 엔진 성능 비교 이후 추가했으며, 엔진 벤치마크 결과에 포함되지 않습니다.

## 성능 비교 방법

- 입력: 기존 OPS2 V1 Swarm Input, `wsjk156.json`을 별도 입력 폴더에 복사하고 SHA256 기록.
- overlap 5mm, PolySimpl 0.01, 기본 worker/연산 설정. 테스트 시간만 Explore 4초·Compress 3초·Polish 2초로 양쪽 동일하게 제한.
- 각 입력에서 seed 11~20의 10쌍. 수정 전후 순서를 seed마다 교대로 실행.
- 같은 harness를 수정 전후 정적으로 링크한 별도 실행 파일 사용. 개선 버전에만 측정 종료 후 counter 직렬화 추가.
- 각 쌍의 초기 배치 길이가 정확히 같은지 확인. 각 단계 결과는 공통 정밀 검사와 새로 만든 독립 원본 guard 검사에 모두 통과해야 기록.
- 시간 제한 탐색이므로 처리량 증가가 실행 시간 감소 대신 **동일 시간의 길이 개선**으로 나타날 수 있음.
- 동일 해의 정밀 검사 100회 반복 비용을 별도 측정. 이 값은 변경된 해의 cold 검사나 전체 네스팅 시간의 개선율이 아님.

이번 비교는 Adaptive Explore → Compress → Polish의 짧은 예산 비교입니다. Reclaim, NFP 순서 탐색, 모든 외부 LP backend, 실제 WPF 콜백 비용, 장시간 목표 수율 도달 시간, peak RSS의 전면 비교는 아닙니다. seed가 같아도 연산 순서와 시간 종료 지점이 달라지므로 결과가 항상 같은 방향으로 좋아진다고 보장하지 않습니다.

## 결과

**여성WSjk는 유망하지만 OPS2까지 일괄 성능 통과라고 결론 내릴 수 없습니다.** 20쌍 40회 실행 모두 종료 코드 0이며, 각 쌍의 초기 길이가 일치하고 각 단계의 정밀·원본 검사를 통과했습니다. 아래 값은 같은 seed에서 `수정 전 길이 - 수정 후 길이`이므로 양수가 개선입니다.

| 입력 | 쌍 수 | 평균 절감(mm) | 절감 중앙값(mm) | 개선/동일/악화 | 가장 악화된 쌍 |
|---|---:|---:|---:|---:|---:|
| 여성WSjk | 10 | **38.585** | 50.928 | 7 / 0 / 3 | 27.888mm 길어짐 |
| OPS2 | 10 | **-0.880** | 1.818 | 7 / 0 / 3 | 57.983mm 길어짐 |

- 여성WSjk의 최종 길이 중앙값: 6594.177 → 6565.547mm. 가장 큰 절감 93.294mm.
- OPS2의 최종 길이 중앙값: 3477.853 → 3474.000mm. 가장 큰 절감 43.249mm.
- 각 버전 길이의 중앙값 차이와, 같은 seed에서 구한 절감의 중앙값은 서로 다른 통계입니다.
- 단계 시간 중앙값은 양쪽 거의 동일했습니다. 여성WSjk Polish는 약 2.126초로 설정한 2초를 조금 넘었습니다. 단계 종료 시 인증 등을 포함하므로 시간 제한은 엄격한 전체 경과 시간 상한이 아닙니다.

### 동일 결과 인증 cache 비용

100회 반복한 동일 해 검사에서 1회당 시간 중앙값은 여성WSjk **4.354279 → 0.0007945ms**, OPS2 **1.853786 → 0.000279ms**였습니다. 이미 성공 인증된 같은 해의 반복 비용만 감소한 것입니다. 전체 엔진이 이 비율만큼 빨라졌다는 뜻이 아닙니다.

계측의 인증 cache hit 중앙값은 두 입력 모두 102회인데 **100회는 이 반복 측정에서 의도적으로 발생**했습니다. 나머지 실행에서의 재사용은 이 harness에서는 적었습니다. 실제 WPF 콜백 조건에서의 재사용률과 총 이득은 별도 측정해야 합니다.

행렬 계측도 여성WSjk 공유 약 701MB / 실제 복사 약 699MB, OPS2 공유 약 120MB / 실제 복사 약 120MB로 비슷했습니다. 이는 실행 누적량이지 peak 메모리가 아닙니다. 움직이는 worker는 거의 모두 첫 쓰기를 하므로, copy-on-write를 전체 복사 제거로 해석할 수 없습니다.

### 판단 범위

이번 결과는 여러 변경을 함께 적용한 A/B입니다. Swap 하나의 기여나 각 변경의 독립 효과를 증명하는 ablation 비교는 아닙니다. 목표 수율까지 시간과 장시간 수율은 아직 측정하지 않았습니다. 짧은 시간 제한 탐색의 편차도 있으므로 OPS2의 평균 악화를 숨기거나 전체 통과로 표시하지 않았습니다. **현재 후보를 유지한 상태로 사용자에게 유지/롤백 결정을 요청합니다.**

전체 seed별 결과는 [비교표](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-bench/summary.md), 수치·입력 hash·계측은 [집계 JSON](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-bench/summary.json)에 보관했습니다. [집계 스크립트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/summarize_phase_improvements.py)로 다시 계산할 수 있습니다.

## 검증·빌드

| 검증 | 결과 |
|---|---|
| Rust core 단위 테스트 | 731 통과, 13 ignored |
| 인증 cache·원본 병렬 검사·Polish 중지·잠금·그룹·Swap/R&R 등 통합 회귀 | 49 통과, 1 ignored |
| Native 단위 및 단계·잠금·그룹·힌트·OrgID C ABI 회귀 | 128 통과, 1 ignored |
| C# Init preview 및 raw callback 회귀 | 12 통과 |
| Rust Release DLL | 성공, linker import-library 안내 warning 1개 |
| WPF Sample Release | 성공, 경고 14개, 오류 0개 |
| rustfmt / git diff --check / XAML XML 파싱 | 통과 |
| 복구 manifest 및 원본/최종 hash 검사 | preview 통과, 실제 복구 미실행 |

Rust는 **908 통과, 15 ignored, 실패 0개**입니다. C# 12개는 별도 집계입니다. 새 검사는 행렬 공유 후 쓰기 분리, 인증 정책 변경 시 무효화, 64개 복잡 도형의 직렬/병렬 원본 판정 동등성, Polish 중간 인증 콜백에서 중지한 뒤 최종 좌표 보존을 포함합니다.

WPF 타이머는 XAML·바인딩 코드·빌드와 시작/종료/폐기 경로를 확인했습니다. **실제 창을 실행한 시간 표시·클릭 검증은 수행하지 않았습니다.** C# 테스트가 이 UI 동작을 직접 검증한 것은 아닙니다.

최종 빌드 명령:

```powershell
cargo build --release --lib --features 'simd capmods adaptive structure-first jni'
dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj -c Release --no-restore
```

Release 설정은 `opt-level = 3`, `lto = "fat"`이며 `codegen-units = 1`은 주석으로 적용되지 않았습니다. 새 SIMD 커널 추가 없이 기존 SIMD 경로를 유지하고, LP의 변 묶음 사전 제외 및 원본 쌍 병렬화를 추가했습니다.

[실행 파일](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/csharp/CAPlanetSwarmSample.WPF/bin/Release/net10.0-windows10.0.19041.0/CAPlanetSwarmSample.WPF.exe)과 같은 폴더의 DLL은 새 [Rust DLL](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/release/capswarmcore_100.dll)과 SHA256이 같습니다.

`DEFC52692A7B56A9DF15A7645C15AC88C65461938DFE89DC723DF9B428A3C3C0`

로그: [core](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-final-core.log), [통합](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-integration.log), [FFI](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-final-ffi.log), [C#](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-final-wpf-tests.log), [Rust 빌드](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-final-native-build.log), [WPF 빌드](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-final-wpf-build.log).

## 복구

백업: `target/phase-improvements-before-20260909-093112`  
수정 전 DLL, 관련 소스, 작업 직전 diff를 보관했습니다. 최종 변경 파일 manifest와 [Rollback.ps1](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/phase-improvements-before-20260909-093112/Rollback.ps1)도 이 백업에 저장했습니다.

복구는 이번 변경 이후 사용자가 수정한 파일을 덮어쓰지 않도록 최종 hash를 대조합니다. 후속 수정이 발견되면 중단하도록 구성합니다. 유지/롤백 결정은 사용자에게 요청하며, 성능 측정 결과만으로 자동 롤백하지 않습니다.


복구 스크립트의 기본 실행은 미리보기이며 `-Apply`가 있어야 변경합니다. 복구 대상은 **이번 Rust 소스·테스트와 Rust/Sample native DLL**입니다. 이전 원본 중지 버그 수정과 새 WPF 시간 표시는 유지합니다. WPF 원본도 별도로 백업했지만 엔진 성능 판단용 복구 대상에는 넣지 않았습니다. 보고서·측정 결과는 남기며, 나머지 컴파일 산출물은 Rust/WPF 재빌드로 갱신해야 합니다.
