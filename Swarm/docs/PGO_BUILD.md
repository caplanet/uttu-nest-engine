# Windows Release: codegen-units=1 + PGO

`Cargo.toml`의 Release 설정은 `opt-level=3`, `lto="fat"`, `codegen-units=1`이다. Windows MSVC의 정적 CRT 설정도 유지한다. PGO는 실제 실행 프로파일이 필요하므로 Cargo 설정만으로 자동 적용되지 않는다.

## 재현 방법

저장소 루트 PowerShell에서 실행한다. `llvm-tools-preview`는 현재 선택된 Rust toolchain에 한 번 설치한다.

```powershell
rustup component add llvm-tools-preview
./scripts/build_pgo_release.ps1 -Deploy
```

기본 학습 입력은 기존 여성 WSjk JSON과 `target/phase-improvements-bench/inputs/ops2.json`이다. 다른 PC에서는 실제 입력 경로를 지정한다.

```powershell
./scripts/build_pgo_release.ps1 -Inputs 'C:/markers/wsjk.json','C:/markers/ops2.json' -Deploy
```

입력은 단순 아이템 목록의 Swarm Input JSON이어야 한다. 이 실행기는 그룹·잠금 힌트를 별도로 주입하지 않는다. 학습 설정은 PolySimpl=0.01, 겹침 허용 2.2mm, Separator 3 workers, seed 41/42, Explore 10초·Compress 5초·Polish 5초다. Polish의 LP·Slide·Hole Fill을 켜며 나머지는 native 기본 설정을 사용한다. 실행 시 설정 전체를 summary JSON에 기록한다. 학습 예산은 생산 설정을 수정하지 않는다.

단계:

1. 새 `target/pgo-build-날짜-시간` 디렉터리를 만들고 기존 DLL을 `before`에 백업한다.
2. 모든 Rust 대상 crate에 `profile-generate`를 적용한 **cdylib만** 빌드한다. 명시적 `--target`으로 host build script/proc-macro를 학습 대상에서 제외한다.
3. C# 실행기가 해당 DLL의 절대 경로를 지정해 두 마커를 실행한다. 현재 체크아웃의 `SwarmNative.cs`를 링크해 구조체를 중복 정의하지 않으며 ABI를 검사한다. 프로세스 종료 시 DLL의 `.profraw`가 실제 생성됐는지 확인한다.
4. 같은 toolchain의 `llvm-profdata`로 프로파일을 병합한다.
5. `profile-generate` 대신 `profile-use`를 전달해 최종 DLL을 다시 빌드한다.
6. 학습에 쓰지 않은 seed 73으로 두 마커의 네 단계를 실행한다. 조각 수·ID별 demand, native 충돌 상태, 단계 간 원단 길이 비증가를 검사한다. `-Deploy`가 있으면 통과한 DLL을 `target/release/capswarmcore_100.dll`에 복사한다.

중간 단계 실패 후 조사할 때는 `-Stage Train`, `-Stage Use`, `-Stage Validate`와 기존 `-RunDirectory`를 명시할 수 있다. 이미 raw profile이 있는 상태에서 Train을 다시 실행하면 중복 학습 자료를 섞지 않고 실패한다. 소스 또는 Rust 버전이 바뀌었으면 새 실행 디렉터리에서 다시 학습한다.

## 실제 컴파일 옵션

공통:

```text
cargo rustc --locked --release --target x86_64-pc-windows-msvc
  --target-dir <run>/cargo --lib --crate-type cdylib
  --features "simd capmods adaptive structure-first jni" -vv -- -C lto=fat
```

학습 빌드의 `RUSTFLAGS`:

```text
-C target-feature=+crt-static -C profile-generate=<absolute-run>/raw
```

최종 빌드의 `RUSTFLAGS`:

```text
-C target-feature=+crt-static -C profile-use=<absolute-run>/merged.profdata
-C llvm-args=-pgo-warn-missing-function
```

`RUSTFLAGS`가 `.cargo/config.toml`을 대체하므로 정적 CRT를 명시적으로 반복한다. `codegen-units=1`과 `opt-level=3`, fat LTO도 스크립트에서 보장한다. 실행 후 환경 변수는 복원한다. 특정 CPU 전용 `target-cpu=native`는 적용하지 않는다.

## 적용 범위와 보관 자료

- PGO는 실행 빈도를 컴파일러에 제공하는 최적화다. 학습하지 않은 경로를 삭제하거나 기능 사용을 제한하지 않는다. 비활성 기능도 빌드 feature에 포함돼 있다면 유지된다.
- Rust 및 Rust 의존 crate가 대상이다. MSVC로 컴파일하는 Clipper2 C++ 코드에 별도 C++ PGO를 적용한 것은 아니다. Clipper2의 기존 최적화와 정적 CRT는 유지한다.
- 이 학습은 두 실제 마커의 일반 단계 경로를 대표한다. 모든 초기배치 전략, Structure-first, grouped/frozen, JNI 호출을 모두 학습했다는 뜻은 아니다.
- `profile-summary.txt`에 함수 실행 빈도, `training-manifest.json`에 toolchain·입력·DLL·프로파일 hash를 보관한다. `use-build.log`에 실제 `profile-use`와 경고를 남긴다.
- `final`은 배포 DLL, `instrumented`는 수집용 DLL이다. **수집용 DLL은 배포하지 않는다.**
- 일반 `cargo build --release` 또는 위 static CRT 문서의 일반 빌드를 실행하면 PGO 없는 DLL로 다시 생성될 수 있다. PGO 배포 빌드는 이 스크립트로 만든다.
- 최적화 적용 사실과 실제 속도 향상은 별개다. 시간 제한 네스팅 결과만으로 처리 속도 향상률을 단정하지 않는다.

설계 근거: [Rust 공식 PGO 절차](https://doc.rust-lang.org/rustc/profile-guided-optimization.html).

## 2026-09-10 실제 빌드·검증 결과

실행 디렉터리: `target/pgo-build-20260910-141519`.

- Rust `1.99.0-nightly (c4af71034 2026-07-06)`, LLVM `22.1.8`, Windows x64 MSVC.
- 계측 빌드 6분 51초, 최종 profile-use 빌드 5분 36초. 실제 root rustc 명령에서 `opt-level=3`, `codegen-units=1`, `lto=fat`, `+crt-static`, `profile-use`를 모두 확인했다.
- WSjk 156조각과 OPS2 52조각을 각각 seed 41/42로 학습했다. 네 실행의 16개 단계 종료점에서 누락·충돌·길이 증가가 없었다. 원본 입력 파일은 수정하지 않았다.
- `.profraw` 4개가 생성됐고, 병합 프로파일은 3,907,976 bytes다. 10,257개 함수 레코드 중 실행된 함수는 1,313개, 그중 `swarm_core`를 포함하는 함수는 633개다. Separator 후보 평가가 최대 블록 실행 횟수의 최상위에 기록됐다. 실행되지 않은 함수 레코드를 학습 완료로 세지 않았다.
- 최종 빌드에서 `windows_sys`의 문자열 변환 함수 2개와 `num_cpus`의 정보 조회 함수 2개에 프로파일 누락 경고가 있었다. Swarm core와 C API의 프로파일 누락 경고는 없었다. 그 외 의존 crate의 기존 컴파일 경고와 MSVC import library 생성 안내도 로그에 남아 있다.

학습하지 않은 seed 73의 **최종 DLL 기능 검증**:

| 마커 | 조각 수 | Init 길이(mm) | Explore | Compress | Polish | 모든 단계 native 충돌 표시 |
|---|---:|---:|---:|---:|---:|---:|
| 여성 WSjk | 156 | 7104.878 | 6720.680 | 6653.432 | 6624.439 | 0 |
| OPS2 | 52 | 4053.311 | 3632.278 | 3620.436 | 3600.661 | 0 |

이는 배치 단계가 정상 완료되는지 확인한 결과이며, PGO 전후 속도·수율 비교표가 아니다. 별도 통제된 성능 비교를 수행하지 않았으므로 속도 향상률을 주장하지 않는다.

최종 DLL 검증:

- ABI 15, config 776 bytes, Structure config 116 bytes, feature flags 31, 필수 export 33개 통과.
- 작은 도형 3개의 Init→JSON export→전용 메모리 해제를 5회 반복해 통과.
- VC++ runtime 및 동적 UCRT DLL 의존성이 없으며, Clipper2 정적 라이브러리에서도 `LIBCMT`·`libcpmt` 사용을 확인했다.
- 계측 DLL의 `.lprfc`·`.lprfn` 섹션이 최종 DLL에서는 제거됐다. 최종 DLL에 학습용 카운터가 남지 않았다.
- 전체 Rust 회귀 테스트 모음을 다시 실행하거나 WPF 화면에서 직접 네스팅한 것은 아니다. C# 실행기가 최종 DLL을 직접 호출한 검증이다.

배포 산출물:

- `target/release/capswarmcore_100.dll`에 최종 DLL 반영 완료. 크기 6,401,536 bytes.
- SHA256: `65689606F2C941F7694EFA381865B8FCFA89313F1F452260ECA8D0A406B3062C`.
- 새 DLL을 포함한 WPF: `target/pgo-build-20260910-141519/wpf/CAPlanetSwarmSample.WPF.exe`. Release x64 빌드 경고 0·오류 0이며, 포함 DLL의 SHA256·ABI·필수 export를 확인했다.
- 기존 WPF 폴더의 DLL 잠금이 풀린 것을 확인해 `csharp/CAPlanetSwarmSample.WPF/bin/x64/Release/net10.0-windows10.0.19041.0/CAPlanetSwarmSample.WPF.exe`도 다시 빌드했다. 경고 0·오류 0이며, 이 폴더의 DLL도 위 최종 SHA256·ABI와 일치한다. 기존 경로의 Sample을 실행해도 이번 PGO DLL을 사용한다.
- 변경 전 DLL과 Cargo 설정은 실행 디렉터리의 `before`에 보관했다. 병합 프로파일·빌드 로그·마커 결과·검증 manifest도 같은 실행 디렉터리에 있다.
