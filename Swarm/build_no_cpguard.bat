@echo off
setlocal EnableExtensions
rem A/B baseline: six policy wrappers OFF; seed VM protection remains ON.
rem This matches the previous benchmark's BEFORE condition, not a fully unprotected build.
call "%~dp0build.bat" release x64 %* cpguard-seed-only
exit /b %ERRORLEVEL%
