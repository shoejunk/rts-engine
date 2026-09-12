@echo off
setlocal

set "PROJECT_DIR=%~dp0"
set "SOURCE_DIR=%~dp0."
set "BUILD_DIR=%PROJECT_DIR%build"
set "CMAKE_EXE="

for /f "delims=" %%C in ('where.exe cmake 2^>nul') do if not defined CMAKE_EXE set "CMAKE_EXE=%%C"

if not defined CMAKE_EXE (
    echo Could not find cmake.exe on PATH.
    exit /b 1
)

rem Avoid the PATH/Path casing conflict that can prevent MSBuild from launching cl.exe.
set "RTS_PATH=%PATH%"
set "PATH="
set "Path=%RTS_PATH%"

"%CMAKE_EXE%" -S "%SOURCE_DIR%" -B "%BUILD_DIR%" -G "Visual Studio 17 2022" -A x64
set "EXIT_CODE=%ERRORLEVEL%"

if not "%EXIT_CODE%" == "0" echo Project file generation failed.

endlocal & exit /b %EXIT_CODE%
