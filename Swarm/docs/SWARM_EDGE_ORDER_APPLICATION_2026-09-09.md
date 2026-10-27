# 변 검사 순서 재사용 적용 및 WPF 테두리 조정

2026-09-09 / `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`

사용자가 승인한 D(변 검사 순서 재사용)만 운영 코어에 적용했다. 기존 동기식 Separator를 유지하며, A(동일 이동 생략), B(준비 목록 공유), C(비동기 배치 인계)는 적용하지 않았다.

## Rust 변경

- `SeparationEvaluator`가 조각의 비트 반전 변 순서를 검색 시작 시 한 번 생성한다. 후보 이동·회전·허용 loss가 바뀌어도 같은 인덱스를 재사용한다.
- pooled evaluator도 다른 조각으로 전환할 때 현재 조각의 변 개수로 새 순서를 만든다. 이전 조각의 순서를 잘못 사용하지 않는다.
- 공개된 단발 충돌 검사 함수의 시그니처는 유지했다. 캐시 경로와 단발 경로는 같은 내부 충돌 검사 코드를 실행하며, 각각 준비된 인덱스와 기존 iterator를 전달한다.
- 새 Config나 컴파일 기능 없이 적용했다. 검사 대상·순서, SIMD 계산, 조기 종료와 원본 인증 규칙은 유지한다.

변경 파일: [sep_evaluator.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/sep_evaluator.rs), [specialized_jaguars_pipeline.rs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/src/eval/specialized_jaguars_pipeline.rs).

## WPF 변경

Place 외곽선의 `StrokeWidth`를 `1.2f * inverseScale`에서 `0f`로 바꿨다. 설치된 SkiaSharp 4.151.1의 문서에 따르면 0은 hairline이며, 변환 행렬과 무관하게 1픽셀 두께로 그린다. 기존에는 추가 확대 배율 `_Zoom`이 선 두께에도 적용됐다. 이제 확대해도 Place 외곽선은 1픽셀을 유지한다.

변경 파일: [WorkspaceView.xaml.cs](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/csharp/CAPlanetSwarmSample.WPF/Views/WorkspaceView.xaml.cs:221). 원단 경계·그리드·채우기·그림자는 이번 변경 대상이 아니다.

## 검증

- SIMD 활성: 코어 단위 테스트 732개, 기존 관련 통합 테스트 3개, 새 비교 테스트 1개 통과. 기존 무시 테스트 13개.
- SIMD 비활성: 관련 통합 테스트 4개 통과.
- 새 [회귀 테스트](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/swarm-core/tests/separation_edge_order.rs)는 변 개수가 다른 도형, 회전, 원단 이탈, 유한/무한 loss 한도, strip 폭 변경과 배치 복원을 검사한다. 새 evaluator와 pooled evaluator를 기존 단발 경로와 비교하며 결과 종류와 loss의 비트 일치를 확인한다.
- 별도 읽기 전용 코드 검토에서 차단할 문제 없음.
- Rust Release 빌드 성공: `cargo build --release --lib --features 'simd capmods adaptive structure-first jni'`. `lto = "fat"` 사용. 라이브러리·exp 생성 안내에 따른 linker 경고 1개.
- WPF Release 빌드 성공: `dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj -c Release --no-restore`. 오류 0개, 이번에 수정하지 않은 파일의 nullable·ReactiveUI 경고 14개.
- WPF 출력 폴더의 native DLL이 이번 Rust Release DLL과 SHA256 일치함을 확인했다. 실제 WPF 창을 실행해 육안으로 확인하지는 않았다.
- 시험 전 운영 코어 해시 목록과 비교해 이번 변경이 승인된 두 Rust 파일에 한정됨을 확인했다. `git diff --check` 통과.

[빌드한 WPF Sample 실행 파일](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/csharp/CAPlanetSwarmSample.WPF/bin/Release/net10.0-windows10.0.19041.0/CAPlanetSwarmSample.WPF.exe)

이번 적용으로 새로운 속도 향상률을 측정하지 않았다. 이전 20초·3시드 실험의 D 결과는 [분리 실험 보고서](C:/Caplanet/AiWorks/AdaptiveAlgorithm2/docs/SWARM_SEPARATOR_VARIANTS_TRIAL_RESULTS_2026-09-09.md)에 보존했다.

변경 전 두 Rust 파일, WPF 표시 파일과 기존 Release DLL은 `target/edge-order-application`에 보관했다. Uttu.Nest와 fable은 이번 작업에서 읽거나 수정하지 않았다.
