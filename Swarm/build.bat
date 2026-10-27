@echo off
chcp 65001 >nul
rem ===========================================================================
rem  Swarm 빌드: Rust 네이티브 라이브러리(cpswcore.dll) + WPF 샘플
rem
rem  사용법:  build.bat [release^|debug] [x64^|arm64] [all^|rust^|wpf] [test]
rem    인자 순서는 상관없고, 생략하면 release / 이 PC의 CPU / all 이다.
rem
rem    release  배포용. Rust release 프로필(opt 3, fat LTO, codegen-units 1, 정적 CRT),
rem             WPF Release. 결과 DLL: target\release\cpswcore.dll
rem    debug    개발용. Rust dev-fast 프로필(opt 2, 디버그 정보), WPF Debug.
rem             결과 DLL: target\dev-fast\cpswcore.dll
rem    x64      x64 Windows 용.
rem    arm64    ARM64 Windows 용. 다른 CPU 에서 만들 때는 결과 DLL 이
rem             target\aarch64-pc-windows-msvc\<프로필>\ 에 생긴다.
rem    all      Rust 다음 WPF (기본).   rust  Rust 만.   wpf  WPF 만(DLL 이 있어야 함).
rem    test     빌드 뒤 Rust 전체 테스트와 C# 테스트까지 돌린다(이 PC CPU 일 때만).
rem
rem  예)  build.bat                  x64 PC 에서 x64 Release 전체 빌드
rem       build.bat debug            개발용 빌드
rem       build.bat release arm64    ARM64 배포 빌드
rem       build.bat rust test        Rust 만 빌드하고 테스트
rem
rem  Rust 기능: simd, capmods, adaptive, structure-first, jni (WPF 가 요구하는 조합).
rem  PGO 까지 적용한 배포 DLL 은 docs\PGO_BUILD.md 의 절차를 따른다.
rem ===========================================================================
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "FEATURES=simd,capmods,adaptive,structure-first,jni"
set "WPF_PROJ=%ROOT%\csharp\CAPlanetSwarmSample.WPF\CAPlanetSwarmSample.WPF.csproj"
set "WPF_TESTS=%ROOT%\csharp\CAPlanetSwarmSample.WPF.Tests\CAPlanetSwarmSample.WPF.Tests.csproj"
set "WPF_EXE_NAME=CAPlanetSwarmSample.WPF.exe"

rem ---- 인자 ------------------------------------------------------------------
set "CONFIG=release"
set "ARCH="
set "SCOPE=all"
set "RUN_TESTS=0"
set "GUARD_MODE="
:parse
if "%~1"=="" goto parsed
set "ARG=%~1"
if /i "!ARG!"=="release" (set "CONFIG=release" & shift & goto parse)
if /i "!ARG!"=="debug"   (set "CONFIG=debug"   & shift & goto parse)
if /i "!ARG!"=="x64"     (set "ARCH=x64"       & shift & goto parse)
if /i "!ARG!"=="amd64"   (set "ARCH=x64"       & shift & goto parse)
if /i "!ARG!"=="arm64"   (set "ARCH=arm64"     & shift & goto parse)
if /i "!ARG!"=="all"     (set "SCOPE=all"      & shift & goto parse)
if /i "!ARG!"=="rust"    (set "SCOPE=rust"     & shift & goto parse)
if /i "!ARG!"=="wpf"     (set "SCOPE=wpf"      & shift & goto parse)
if /i "!ARG!"=="test"    (set "RUN_TESTS=1"    & shift & goto parse)
if /i "!ARG!"=="cpguard-policy" (set "GUARD_MODE=cpguard-policy" & shift & goto parse)
if /i "!ARG!"=="cpguard-seed-only" (set "GUARD_MODE=cpguard-seed-only" & shift & goto parse)
if /i "!ARG!"=="help"    goto usage
if /i "!ARG!"=="-h"      goto usage
if /i "!ARG!"=="/?"      goto usage
echo [오류] 알 수 없는 인자: !ARG!
goto usage_fail
:parsed

rem ---- 이 PC 의 CPU ----------------------------------------------------------
set "HOST_ARCH=x64"
set "HOST_RAW=%PROCESSOR_ARCHITECTURE%"
if defined PROCESSOR_ARCHITEW6432 set "HOST_RAW=%PROCESSOR_ARCHITEW6432%"
if /i "%HOST_RAW%"=="ARM64" set "HOST_ARCH=arm64"
if not defined ARCH set "ARCH=%HOST_ARCH%"

if /i "%ARCH%"=="x64" (
    set "TRIPLE=x86_64-pc-windows-msvc"
) else (
    set "TRIPLE=aarch64-pc-windows-msvc"
)

if /i "%CONFIG%"=="release" (
    set "CARGO_PROFILE_ARGS=--release"
    set "PROFILE_DIR=release"
    set "DOTNET_CONFIG=Release"
) else (
    set "CARGO_PROFILE_ARGS=--profile dev-fast"
    set "PROFILE_DIR=dev-fast"
    set "DOTNET_CONFIG=Debug"
)

rem 다른 CPU 용으로 만들 때만 --target 을 준다. 같은 CPU 면 기존 경로(target\<프로필>)를 쓴다.
rem Explicit Cargo output root prevents inherited CARGO_TARGET_DIR from selecting a stale DLL.
set "CARGO_TARGET_DIR=%ROOT%\target"
set "WPF_OUTPUT_ARGS="
if defined GUARD_MODE (
    set "CARGO_TARGET_DIR=%ROOT%\target\%GUARD_MODE%"
    set WPF_OUTPUT_ARGS="-p:OutputPath=%ROOT%\target\%GUARD_MODE%\wpf\%ARCH%\%DOTNET_CONFIG%"
)
if /i "%GUARD_MODE%"=="cpguard-seed-only" set "FEATURES=%FEATURES%,cpguard-policy-off"
set "CARGO_TARGET_ARGS="
set "NATIVE_DIR=%CARGO_TARGET_DIR%\%PROFILE_DIR%"
if /i not "%ARCH%"=="%HOST_ARCH%" (
    set "CARGO_TARGET_ARGS=--target %TRIPLE%"
    set "NATIVE_DIR=%CARGO_TARGET_DIR%\%TRIPLE%\%PROFILE_DIR%"
)
set "NATIVE_DLL=%NATIVE_DIR%\cpswcore.dll"

echo.
echo ===========================================================================
echo  Swarm 빌드  구성=%CONFIG%  CPU=%ARCH% (이 PC: %HOST_ARCH%)  범위=%SCOPE%  테스트=%RUN_TESTS%
echo  Rust 대상=%TRIPLE%  DLL=%NATIVE_DLL%
if defined GUARD_MODE echo  Protection=%GUARD_MODE% ^(seed VM always enabled^)
echo ===========================================================================

rem ---- 도구 확인 ---------------------------------------------------------------
if /i not "%SCOPE%"=="wpf" (
    where cargo >nul 2>nul
    if errorlevel 1 (
        echo [오류] cargo 를 찾을 수 없습니다. https://rustup.rs 에서 Rust 를 설치한 뒤
        echo        새 명령 창에서 다시 실행하세요. 툴체인은 rust-toolchain.toml 의 nightly 를 씁니다.
        goto fail
    )
)
if /i not "%SCOPE%"=="rust" (
    where dotnet >nul 2>nul
    if errorlevel 1 (
        echo [오류] dotnet 을 찾을 수 없습니다. .NET 10 SDK 를 설치하세요.
        goto fail
    )
)

rem 실행 중인 WPF 는 DLL 을 잠가 덮어쓰기를 실패시킨다.
tasklist /FI "IMAGENAME eq %WPF_EXE_NAME%" 2>nul | find /i "%WPF_EXE_NAME%" >nul
if not errorlevel 1 (
    echo [오류] %WPF_EXE_NAME% 가 실행 중입니다. 종료한 뒤 다시 실행하세요.
    goto fail
)

rem ---- Rust ------------------------------------------------------------------
if /i "%SCOPE%"=="wpf" goto wpf

if defined CARGO_TARGET_ARGS (
    rustup target list --installed 2>nul | find /i "%TRIPLE%" >nul
    if errorlevel 1 (
        echo [준비] Rust 대상 %TRIPLE% 를 설치합니다.
        rustup target add %TRIPLE%
        if errorlevel 1 goto fail
    )
)

echo.
echo [1/3] Rust: cargo build -p swarm --lib %CARGO_PROFILE_ARGS% %CARGO_TARGET_ARGS% --features "%FEATURES%"
pushd "%ROOT%"
cargo build -p swarm --lib %CARGO_PROFILE_ARGS% %CARGO_TARGET_ARGS% --features "%FEATURES%"
set "RC=%ERRORLEVEL%"
popd
if not "%RC%"=="0" (
    echo [오류] Rust 빌드 실패 ^(exit %RC%^).
    if /i "%ARCH%"=="arm64" if /i not "%HOST_ARCH%"=="arm64" (
        echo        ARM64 교차 빌드에는 Visual Studio Installer 의
        echo        "MSVC v143+ ARM64/ARM64EC 빌드 도구" 구성 요소가 필요합니다.
        echo        clipper2 의 C++ 코드를 ARM64 로 컴파일해야 하기 때문입니다.
    )
    goto fail
)
if not exist "%NATIVE_DLL%" (
    echo [오류] 빌드는 끝났지만 %NATIVE_DLL% 가 없습니다.
    goto fail
)
echo [완료] %NATIVE_DLL%

rem ---- 디컴파일 방어 감사 게이트 (release 전용) ------------------------------
rem release DLL 에 복원 단서(소스경로/크레이트버전/PDB/모듈명/알고리즘 문자열/사용자명)가
rem 다시 새지 않았는지 검사한다. 하드닝 회귀 방지용. dev-fast/debug 는 진단 문자열을
rem 의도적으로 유지하므로 건너뛴다. python 이 없으면 건너뛴다(게이트는 선택적 강제).
if /i not "%CONFIG%"=="release" goto audit_done
where python >nul 2>nul
if errorlevel 1 (
    echo [감사] python 이 없어 감사를 건너뜁니다. 설치 시 자동 검사됩니다.
    goto audit_done
)
echo.
echo [감사] 디컴파일 단서 검사: scripts\audit_leaks.py
python "%ROOT%\scripts\audit_leaks.py" "%NATIVE_DLL%"
if errorlevel 1 (
    echo [오류] 디컴파일 단서 감사 실패 - 하드닝이 회귀했습니다. Sources\Swarm\HARDENING.md 참고.
    goto fail
)
:audit_done

if /i "%SCOPE%"=="rust" goto tests

rem ---- WPF -------------------------------------------------------------------
:wpf
if not exist "%NATIVE_DLL%" (
    echo [오류] %NATIVE_DLL% 가 없습니다. 먼저 Rust 를 빌드하세요: build.bat %CONFIG% %ARCH% rust
    goto fail
)

rem x64 는 솔루션의 x64 플랫폼, ARM64 는 AnyCPU 프로젝트를 ARM64 프로세스로 빌드한다
rem (프로젝트에 ARM64 플랫폼이 따로 없다). DLL 위치는 명시적으로 넘긴다.
if /i "%ARCH%"=="x64" (
    set "WPF_PLATFORM_ARGS=-p:Platform=x64"
) else (
    set "WPF_PLATFORM_ARGS=-p:Platform=AnyCPU -p:PlatformTarget=ARM64"
)

echo.
echo [2/3] WPF: dotnet build -c %DOTNET_CONFIG% %WPF_PLATFORM_ARGS%
dotnet build "%WPF_PROJ%" -c %DOTNET_CONFIG% %WPF_PLATFORM_ARGS% %WPF_OUTPUT_ARGS% -p:SwarmNativeProfile=%PROFILE_DIR% "-p:SwarmNativeDirectory=%NATIVE_DIR%" -nologo -v minimal
if errorlevel 1 (
    echo [오류] WPF 빌드 실패.
    goto fail
)

set "WPF_OUT="
set "OUT_TXT=%TEMP%\swarm_wpf_outdir_%RANDOM%.txt"
dotnet msbuild "%WPF_PROJ%" -nologo -getProperty:TargetDir -p:Configuration=%DOTNET_CONFIG% %WPF_PLATFORM_ARGS% %WPF_OUTPUT_ARGS% > "%OUT_TXT%" 2>nul
if exist "%OUT_TXT%" (
    set /p WPF_OUT=<"%OUT_TXT%"
    del "%OUT_TXT%" >nul 2>nul
)
if not defined WPF_OUT (
    echo [경고] WPF 출력 폴더를 알아내지 못했습니다.
    goto tests
)
if "%WPF_OUT:~-1%"=="\" set "WPF_OUT=%WPF_OUT:~0,-1%"
if not exist "%WPF_OUT%\%WPF_EXE_NAME%" (
    echo [오류] %WPF_OUT%\%WPF_EXE_NAME% 가 없습니다.
    goto fail
)
fc /b "%NATIVE_DLL%" "%WPF_OUT%\cpswcore.dll" >nul 2>nul
if errorlevel 1 (
    echo [오류] WPF 폴더의 cpswcore.dll 이 방금 빌드한 DLL 과 다릅니다: %WPF_OUT%
    goto fail
)
echo [완료] %WPF_OUT%\%WPF_EXE_NAME%
echo        cpswcore.dll 이 방금 빌드한 DLL 과 같습니다.

rem ---- 테스트 ----------------------------------------------------------------
:tests
if "%RUN_TESTS%"=="0" goto done
if /i not "%ARCH%"=="%HOST_ARCH%" (
    echo [건너뜀] 다른 CPU 용 빌드는 이 PC 에서 테스트를 실행할 수 없습니다.
    goto done
)
if /i not "%SCOPE%"=="wpf" (
    echo.
    echo [3/3] Rust 테스트: cargo test --workspace --features "simd capmods adaptive structure-first"
    pushd "%ROOT%"
    cargo test --workspace --no-fail-fast --features "%FEATURES%"
    set "RC=!ERRORLEVEL!"
    popd
    if not "!RC!"=="0" (
        echo [오류] Rust 테스트 실패.
        goto fail
    )
)
if /i not "%SCOPE%"=="rust" (
    echo.
    echo [3/3] C# 테스트: dotnet test -c %DOTNET_CONFIG%
    dotnet test "%WPF_TESTS%" -c %DOTNET_CONFIG% -p:SwarmNativeProfile=%PROFILE_DIR% "-p:SwarmNativeDirectory=%NATIVE_DIR%" -nologo
    if errorlevel 1 (
        echo [오류] C# 테스트 실패.
        goto fail
    )
)

:done
echo.
echo 빌드 성공.
endlocal
exit /b 0

:usage
echo.
echo 사용법:  build.bat [release^|debug] [x64^|arm64] [all^|rust^|wpf] [test]
echo   release  배포용 Rust release + WPF Release (기본)
echo   debug    개발용 Rust dev-fast + WPF Debug
echo   x64      x64 Windows 용 (x64 PC 의 기본)
echo   arm64    ARM64 Windows 용 (ARM64 PC 의 기본)
echo   all      Rust 다음 WPF (기본)
echo   rust     Rust 만
echo   wpf      WPF 만 (해당 구성의 DLL 이 이미 있어야 함)
echo   test     빌드 뒤 Rust 와 C# 테스트까지 실행
echo   cpguard-policy     Seed VM + six policy wrappers, isolated output
echo   cpguard-seed-only  Seed VM only, isolated output ^(A/B baseline^)
endlocal
exit /b 0

:usage_fail
echo 도움말: build.bat help
:fail
echo.
echo 빌드 실패.
endlocal
exit /b 1
