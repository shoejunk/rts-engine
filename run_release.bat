@echo off
setlocal

call "%~dp0build_release.bat"
if errorlevel 1 exit /b %ERRORLEVEL%

"%~dp0build\Release\rts_engine.exe"
set "EXIT_CODE=%ERRORLEVEL%"

endlocal & exit /b %EXIT_CODE%
