@echo off
REM ── Swarm JNI Sample: build & run ──────────────────────────────
REM Prerequisites: CMake build complete (swarm.dll in target\debug\ or target\release\)
REM Usage: run.bat [input.json] [output_dir]

setlocal
cd /d "%~dp0.."

REM ── Find JDK ───────────────────────────────────────────────────
set "JAVAC="
set "JAVA="
if defined JAVA_HOME (
    if exist "%JAVA_HOME%\bin\javac.exe" set "JAVAC=%JAVA_HOME%\bin\javac.exe"
    if exist "%JAVA_HOME%\bin\java.exe"  set "JAVA=%JAVA_HOME%\bin\java.exe"
)
if not defined JAVAC for %%d in (
    "C:\Program Files\Android\openjdk\jdk-21.0.8"
    "C:\Program Files\Android\openjdk\jdk-17"
    "C:\Program Files\Java\jdk-21"
    "C:\Program Files\Java\jdk-17"
    "C:\Program Files\Eclipse Adoptium\jdk-21"
    "C:\Program Files\Eclipse Adoptium\jdk-17"
    "C:\Program Files\Microsoft\jdk-21"
    "C:\Program Files\Microsoft\jdk-17"
) do (
    if exist "%%~d\bin\javac.exe" if not defined JAVAC set "JAVAC=%%~d\bin\javac.exe"
    if exist "%%~d\bin\java.exe"  if not defined JAVA  set "JAVA=%%~d\bin\java.exe"
)
if not defined JAVAC (
    echo ERROR: javac not found. Install JDK 17+ or set JAVA_HOME.
    exit /b 1
)
if not defined JAVA (
    echo ERROR: java not found. Install JDK 17+ or set JAVA_HOME.
    exit /b 1
)
echo JDK: %JAVAC%

REM ── Detect build config (Debug vs Release) ─────────────────────
if exist "target\release\swarm.dll" (
    set "CONFIG=release"
) else if exist "target\debug\swarm.dll" (
    set "CONFIG=debug"
) else (
    echo ERROR: swarm.dll not found. Build with CMake first.
    exit /b 1
)

set "DLL_DIR=target\%CONFIG%"
set "CLASS_DIR=target\%CONFIG%\java"

echo === Config: %CONFIG% ===

REM ── Compile Java classes ───────────────────────────────────────
echo === Building Java classes ===
if not exist "%CLASS_DIR%" mkdir "%CLASS_DIR%"
"%JAVAC%" -encoding UTF-8 -d "%CLASS_DIR%" java/com/swarm/*.java java-sample/SampleRunner.java
if %errorlevel% neq 0 (
    echo ERROR: Java compilation failed.
    exit /b 1
)

echo.
echo === Running Sample ===
set INPUT=%1
if "%INPUT%"=="" set INPUT=data/input/albano.json
set OUTDIR=%2
if "%OUTDIR%"=="" set OUTDIR=output

"%JAVA%" -Djava.library.path=%DLL_DIR% -cp "%CLASS_DIR%" com.swarm.sample.SampleRunner "%INPUT%" "%OUTDIR%"

endlocal
