@echo off
setlocal

call "%~dp0generate_project_files.bat"
if errorlevel 1 exit /b %ERRORLEVEL%

set "PROJECT_DIR=%~dp0"
set "BUILD_DIR=%PROJECT_DIR%build"
set "CMAKE_EXE="

for /f "delims=" %%C in ('where.exe cmake 2^>nul') do if not defined CMAKE_EXE set "CMAKE_EXE=%%C"

if not defined CMAKE_EXE (
    echo Could not find cmake.exe on PATH.
    exit /b 1
)

set "RTS_PATH=%PATH%"
set "PATH="
set "Path=%RTS_PATH%"

"%CMAKE_EXE%" --build "%BUILD_DIR%" --config Release --parallel
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%" == "0" echo Release build failed.

endlocal & exit /b %EXIT_CODE%
