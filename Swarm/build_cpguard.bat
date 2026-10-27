@echo off
setlocal EnableExtensions
rem Tested protection: seed VM + six policy wrappers. Defaults: Release x64, Rust + WPF.
call "%~dp0build.bat" release x64 %* cpguard-policy
exit /b %ERRORLEVEL%
