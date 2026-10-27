# 선택적 겹침 보호 롤백 — 2026-09-08

사용자가 실제 효율 달성 시간이 너무 느리다고 판단하여, 직선·유지 정점을 별도로 보호하는 시험 기능의 롤백을 요청했다.

## 복원한 동작

- 직선·유지 정점·단순화 변경 구간을 구분하지 않고 기존 `piece_overlap_mm` 한도를 공통 적용한다. 5mm 고정이 아니며 1/3/5mm 등 입력값을 따른다.
- Swarm 입력 원본 폴리곤의 교집합을 허용값 절반만큼 안쪽으로 offset한 뒤 잔여 영역으로 초과 겹침을 판정하던 원본 검사·보정을 유지한다. 기존 수치 격자와 오차 여유도 그대로다.
- 탐색 형상에 보호 띠를 복원하는 처리, 유지 변·정점 분류 및 보호 영역 교집합 검사를 제거했다. 원단 컨테이너 보정도 직전 균일 축소 방식으로 복구했다.
- Init·Explore·Compress·Polish 및 공개 결과에 연결된 원본 인증/보정 경로, 잠금 정책, SWAP·R&R의 후속 분리와 채택 검증은 유지한다.
- 기존 원본 좌표 변환·경계상자 SIMD를 유지한다. 선택적 보호 시험에서 추가된 보호 띠용 SIMD와 충돌 쌍 병렬 처리도 함께 이전 상태로 돌아갔다.
- Config 설정·기본값·ABI를 바꾸거나 새 비활성화 옵션을 추가하지 않았다. 소스를 실제로 복원했다.

## 복원 범위와 보존

기준은 `target/selective-overlap-20260908-130235/source-baseline.zip`과 파일별 SHA256 manifest다. 현재 파일이 시험 종료 candidate와 동일한지 먼저 확인하여 이후 변경이 섞이지 않았음을 검증했다.

다음 6개 파일을 백업 바이트와 정확히 동일하게 복원했다.

1. `swarm-core/src/original_overlap.rs`
2. `swarm-core/src/original_overlap/kernels.rs`
3. `swarm-core/src/util/io.rs`
4. `swarm-core/tests/original_overlap_trial.rs`
5. `swarm-core/tests/overlap_semantics.rs`
6. `src/ffi_step.rs` — 선택적 보호에 맞춰 바뀌었던 시험 부분

추가 파일 `swarm-core/src/original_overlap/protection.rs`, `swarm-core/tests/selective_overlap.rs`는 작업 소스에서 제거했다. 롤백 직전 8개 파일은 [pre-rollback-changes.zip](../target/uniform-overlap-rollback-20260908-141547/pre-rollback-changes.zip)에 보관했다.

복원 과정에서 나머지 360개 파일의 해시가 유지됨을 확인했다. 최근 추가한 WPF **Export PLT** 기능도 보존한다. 이전 성능 비교 데이터·소스 백업·선택 보호 DLL은 기록으로 남기며, 이전 보고서에는 롤백 상태를 표시했다. 그 보고서의 벤치마크 스크립트는 당시 시험용이다.

복원 증거: [rollback-manifest.json](../target/uniform-overlap-rollback-20260908-141547/rollback-manifest.json).

## 빌드 및 검증

빌드 옵션은 기존 Release 그대로 `opt-level=3`, `lto="fat"`, features `simd,capmods,adaptive,structure-first,jni`다. 새 네이티브 DLL을 빌드한 뒤 WPF 출력에 복사한다.

완료 결과:

- Rust Release DLL 빌드 성공. 링크 산출물 안내 warning 1건.
- Rust 단위 시험: native 101건 + core 729건 = **830건 통과**. 기존 ignored 13건 제외.
- 관련 통합 시험: 원본 겹침·보정 7건, 겹침/원단/Compress·Polish·콜백 3건, 잠금 정책 4건, 중지 응답 3건 = **17건 통과**. 기존 ignored 1건 제외. Rust 합계 **847건 통과**.
- 직선 사각형에 대해 입력 예산 1/3/5mm보다 0.02mm 작은 겹침은 허용되고, 예산보다 0.5mm 큰 겹침은 거부되는 기존 시험을 통과했다.
- WPF Release x64 증분 빌드 성공, 경고/오류 0건. WPF 테스트 **81건 통과**: PLT 기능 포함.
- 관리 코드에서 실제 새 DLL의 ABI/초기화/phase/solution export 호출 시험 통과.
- ABI 15 / Config 776 / Structure 116 / features 0x1F 유지. 빌드 DLL과 WPF 폴더 DLL의 SHA256이 `05938f1749660da7a608c70d168323eccf8f6f420ee9d941c338f3af10e77ef5`로 동일하다.
- 재검증에서 복원 파일 6개의 baseline 해시 일치, 기타 소스 보존, 보호 모듈 제거를 확인했다. [verification.json](../target/uniform-overlap-rollback-20260908-141547/verification.json)

검증 명령:

```powershell
cargo build --release --lib --features 'simd,capmods,adaptive,structure-first,jni'
cargo test -p swarm-core -p swarm --release --features 'simd,capmods,adaptive,structure-first,jni' --lib
cargo test -p swarm-core --release --features 'simd,capmods,adaptive,structure-first' --test original_overlap_trial --test overlap_semantics --test frozen_policy --test stop_latency
dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj -c Release -p:Platform=x64 --no-restore
dotnet test csharp/CAPlanetSwarmSample.WPF.Tests/CAPlanetSwarmSample.WPF.Tests.csproj -c Release --no-restore
dotnet run --project target/swarm-native-smoke -c Release
python -X utf8 target/verify_uniform_rollback.py
```

로그는 `target/uniform-rollback-rust-build.log`, `target/uniform-rollback-rust-unit-tests.log`, `target/uniform-rollback-rust-integration.log`, `target/uniform-rollback-wpf-build.log`, `target/uniform-rollback-wpf-tests.log`, `target/uniform-rollback-managed-native.log`에 보관했다. 작업공간 밖의 별도 앱 복사본에는 배포하지 않았다.

이번 작업은 복원과 기능 회귀 확인이며, 새로 전체 효율 달성 시간을 재측정하는 성능 실험은 아니다.
