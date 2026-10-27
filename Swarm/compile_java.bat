@echo off
set "OUTDIR=%~1"
set "JAVAC=%~2"
set "SRCDIR=%~3"
if not exist "%OUTDIR%" mkdir "%OUTDIR%"
"%JAVAC%" -d "%OUTDIR%" "%SRCDIR%/java/com/swarm/SwarmOptimizer.java" "%SRCDIR%/java/com/swarm/SwarmConfig.java" "%SRCDIR%/java/com/swarm/OptimizationResult.java" "%SRCDIR%/java/com/swarm/ProgressListener.java"
exit /b %ERRORLEVEL%
