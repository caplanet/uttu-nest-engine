# cpguard 비교 빌드

두 배치 파일은 기본적으로 **Release / x64 / Rust + WPF Sample**을 빌드한다.
파일 위치를 기준으로 저장소를 찾으므로 다른 작업 디렉터리에서도 실행할 수 있다.

| 배치 파일 | 시드 VM 보호 | 추가한 정책 함수 6개 보호 | 출력 루트 |
|---|---|---|---|
| `build_cpguard.bat` | 켬 | 켬 | `target/cpguard-policy` |
| `build_no_cpguard.bat` | 켬 | 끔 | `target/cpguard-seed-only` |

`build_no_cpguard.bat`은 2026-10-22 A/B 테스트의 **적용 전** 조건이다.
cpguard 전체를 제거하는 빌드가 아니다. 원래의 `protected_seed` VM 보호와
Release 심볼 제거·경로 제거·로그 제거는 양쪽 모두 유지된다.
antitamper/telemetry/degrade 및 init-profile은 양쪽 모두 활성화하지 않는다.

```bat
build_cpguard.bat
build_no_cpguard.bat

rem Rust만 빌드
build_cpguard.bat rust
build_no_cpguard.bat rust

rem 해당 모드의 Rust DLL을 먼저 빌드한 뒤 WPF만 빌드
build_cpguard.bat wpf
build_no_cpguard.bat wpf
```

x64 Windows에서 기본 실행 시 DLL은 각각 다음 위치에 생성된다.

- `target/cpguard-policy/release/cpswcore.dll`
- `target/cpguard-seed-only/release/cpswcore.dll`

WPF 실행 파일은 각각 다음 위치에 생성된다.

- `target/cpguard-policy/wpf/x64/Release/CAPlanetSwarmSample.WPF.exe`
- `target/cpguard-seed-only/wpf/x64/Release/CAPlanetSwarmSample.WPF.exe`

각 WPF 폴더에는 해당 모드의 DLL이 복사되며 빌드 스크립트가 바이트 일치를 검사한다.
일반 `target/release`와 기존 WPF 출력 폴더는 이 두 배치 파일의 출력 대상이 아니다.
다른 CPU에서 교차 빌드하면 기존 `build.bat` 규칙대로 Rust 출력에 target triple이 추가된다.

기존 `build.bat`의 `debug`, `arm64`, `rust`, `wpf`, `test`, `help` 인자를 전달할 수 있다.
인자를 생략하면 이전 성능 측정과 같은 Release x64 조건이다. 빌드 오류는 0이 아닌 종료
코드로 반환된다. `test`는 기존 전체 테스트를 추가 실행하는 선택 옵션이다.

## 구현

공통 빌드는 `build.bat`이 수행하며 두 진입점은 마지막에 각각 `cpguard-policy`,
`cpguard-seed-only` 모드를 고정한다. 모드별 Cargo target과 WPF 출력 경로를 사용한다.

일반 빌드는 기존처럼 정책 보호가 켜져 있다. 다음 Cargo feature만 추가하면 6개 함수의
`#[cpguard::obfuscate]`를 조건부로 제외한다.

```text
cpguard-policy-off -> swarm-core/cpguard-policy-off
```

공통 기능은 `simd,capmods,adaptive,structure-first,jni`다. 시드 보호 코드, 정책 함수 본문,
충돌 검사 및 최적화 계산식은 변경하지 않는다. 소스 파일을 빌드 때 수정하거나 복원하지 않는다.
새 빌드는 현재 작업 트리를 컴파일한다. 과거 테스트 DLL의 해시까지 동일한 결과를 보장하는
보관본 재생 기능은 아니며, 과거 비교 바이너리는 `target/cpguard-policy/before`와 `after`에 남아 있다.

## 2026-10-22 실행 확인

- 두 배치 파일을 기본 인자로 실제 실행: Rust Release + WPF Release x64 모두 성공.
- 양쪽 모두 기존 바이너리 단서 감사 PASS, WPF 폴더 DLL 바이트 일치.
- 각 빌드에서 Rust 미사용 함수 경고 1개, WPF null 참조/소스 생성 경고 7개, 오류 0개.
- Cargo `swarm-core` fingerprint에서 비교용 빌드에만 `cpguard-policy-off`가 들어 있음을 확인.
- 태평양 겉감 4,000개, seed=1, 탐색/압축 각 1초 예산의 실행 검사 모두 통과.
  Init 배치 해시 동일, 최종 4,000개, 네이티브 충돌 상태 0개. 성능 재측정이나 WPF UI 실행 시험은 아니다.
- 로그: `target/build-cpguard-policy.log`, `target/build-cpguard-seed-only.log`.
  실행 검사: `target/cpguard-build-check/verification.json` 및 단계별 JSON.
- 측정 결과와 로그 파일은 로컬에 보관하며 Git에 포함하지 않는다.
