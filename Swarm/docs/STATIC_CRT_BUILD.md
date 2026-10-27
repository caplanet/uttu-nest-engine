# Windows Rust 정적 CRT 빌드

`.cargo/config.toml`에서 Windows MSVC 대상의 CRT를 정적으로 링크한다.

```toml
[target.'cfg(all(target_os = "windows", target_env = "msvc"))']
rustflags = ["-C", "target-feature=+crt-static"]
```

저장소에서 실행하는 Cargo 및 CMake의 Cargo 빌드에 적용된다. 다른 OS·GNU·WASM 대상의 설정은 바꾸지 않았으며, 기존 `target/release` 출력 경로를 유지한다. `.gitignore`에서도 이 설정 파일을 버전 관리할 수 있게 했다.

현재 Clipper2 C++ 의존성은 `cc`를 사용한다. `cc`가 Cargo의 `crt-static` 정보를 읽고 `/MT`를 선택하므로 Rust 최종 링크에만 옵션을 붙이는 방식과 달리 C++ 정적 라이브러리에도 같은 CRT 정책이 전달된다. [Rust 공식 설명](https://doc.rust-lang.org/reference/linkage.html#static-and-dynamic-c-runtimes), [cc static_crt](https://docs.rs/cc/latest/cc/struct.Build.html#method.static_crt).

## 배포 DLL과 WPF 빌드

저장소 루트에서 실행한다.

```powershell
cargo rustc --release --lib --crate-type cdylib --features "simd capmods adaptive structure-first jni" -- -C lto=fat
dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj -c Release -p:Platform=x64 --no-restore
```

정적 CRT는 Cargo 설정에서 적용하고, fat LTO는 배포 DLL 명령에서 명시한다. `rlib`까지 동시에 출력하는 기존 일반 빌드는 실제 DLL의 fat LTO 여부가 다르므로 위의 `cdylib` 전용 명령을 사용한다. 정적 CRT는 **DLL을 없애는 설정이 아니라 CRT 코드를 DLL 안에 포함하는 설정**이다. 이후 Release에 `codegen-units=1`을 추가했다. **PGO까지 적용하려면 [PGO 빌드 절차](PGO_BUILD.md)의 스크립트를 사용한다.** 위 일반 명령은 PGO를 적용하지 않는다.

외부 빌드 환경에서 `RUSTFLAGS`나 `CARGO_ENCODED_RUSTFLAGS`를 지정하면 Cargo 설정의 rustflags를 대체할 수 있다. 이 경우에도 `-C target-feature=+crt-static`을 포함하고 최종 DLL의 의존성을 확인해야 한다.

## 검증과 배포 범위

빌드한 DLL을 `dumpbin /DEPENDENTS`로 검사한다. `MSVCP140*.dll`, `VCRUNTIME140*.dll` 같은 VC++ 런타임 DLL 의존성이 없어야 한다. Windows 시스템 DLL 의존성은 정상이다.

DLL이 반환한 문자열·객체는 기존 `swarm_free_string`, `swarm_free` 등 해당 DLL의 해제 함수로 돌려준다. C#/다른 DLL에서 임의로 `free/delete`하지 않는 기존 소유권 규칙을 유지한다.

이 조치는 Swarm Rust DLL의 VC++ 재배포 패키지 의존성을 제거하기 위한 것이다. WPF/.NET Desktop Runtime과 다른 제품 DLL의 배포 요건까지 변경하는 것은 아니다.

## 2026-09-10 정적 CRT 첫 빌드 검증 이력 (PGO 적용 전)

아래 해시와 WPF 경로는 정적 CRT만 먼저 적용했던 산출물의 기록이다. 이후 `codegen-units=1`·PGO 빌드의 최종 산출물은 [PGO 빌드 문서](PGO_BUILD.md)를 기준으로 확인한다.

- x64 Release DLL 빌드 성공. 실제 compiler 명령에 `target-feature=+crt-static`, `lto=fat` 확인.
- Clipper2 정적 라이브러리의 COFF 지시문에서 `LIBCMT`, `libcpmt` 확인. `/MT` 계열 CRT/C++ 표준 라이브러리를 사용한다.
- 변경 전 DLL의 `MSVCP140.dll`, `VCRUNTIME140.dll`, `VCRUNTIME140_1.dll` 및 `api-ms-win-crt-*` 의존성이 모두 제거됐다.
- 변경 후 DLL은 `api-ms-win-core-synch-l1-2-0.dll`, `kernel32.dll`, `ntdll.dll`, `bcryptprimitives.dll`, `KERNEL32.dll`만 직접 참조한다. Swarm DLL 자체에 VC++ v14 재배포 패키지 DLL 의존성이 남지 않았다.
- 실제 DLL 로딩: ABI 15, config 776bytes, Structure config 116bytes, feature flags 31, 필수 export 33개 확인.
- 매회 조각 3개로 Init→JSON export→전용 메모리 해제를 5회 반복해 통과했다. 전체 네스팅 알고리즘 회귀 테스트나 VC++ 재배포 패키지가 없는 새 Windows PC의 UI 실행까지 시험한 것은 아니다.
- WPF x64의 `libSkiaSharp.dll`, `libHarfBuzzSharp.dll` 직접 의존성에도 VC++/동적 UCRT DLL이 없음을 확인했다. .NET Desktop Runtime 배포 요건은 별도다.

최종 DLL: `target/release/capswarmcore_100.dll`.

SHA256: `BA5F25D156767E1264E15E5B95FA1D9F5F081922F7EF36B4FCB99FBBB48F1343`.

기존 WPF의 `bin/x64/Release/.../capswarmcore_100.dll`은 다른 프로세스에서 사용 중이라 교체하지 못했다. 대신 **`target/static-crt-validation-20260910/wpf/CAPlanetSwarmSample.WPF.exe`**에 새 DLL을 포함한 WPF 빌드를 만들었다(경고 0·오류 0). 이 폴더 DLL과 Rust DLL의 해시 및 실제 ABI가 일치한다. 기존 WPF를 종료한 후 위의 일반 WPF 빌드 명령을 다시 실행하면 기존 폴더에도 반영된다.

검증 로그·변경 전 DLL 백업·산출물 manifest는 `target/static-crt-validation-20260910`에 보관했다. 이번 수정 파일은 `.cargo/config.toml`, `.gitignore`, 이 문서다.
