@echo off
rem ============================================================
rem  Physics/FlameView/shaders/compile_shaders.bat
rem
rem  Compile FlameView's GLSL shaders to SPIR-V using glslc.
rem  Requires the VULKAN_SDK environment variable to be set
rem  (installed automatically by the Vulkan SDK installer).
rem
rem  Output files (written next to this script):
rem    flame_point.vert.spv / flame_point.frag.spv
rem    flame_smoke.vert.spv / flame_smoke.frag.spv
rem    flame_pbvr.vert.spv  / flame_pbvr.frag.spv
rem
rem  Run manually or register as a Visual Studio pre-build event.
rem ============================================================

setlocal

set GLSLC="%VULKAN_SDK%\Bin\glslc.exe"

if not exist %GLSLC% (
    echo [ERROR] glslc.exe not found. VULKAN_SDK=%VULKAN_SDK%
    exit /b 1
)

set OUTDIR=%~dp0

echo Compiling flame_point shaders...
%GLSLC% -fshader-stage=vert "%OUTDIR%flame_point.vert" -o "%OUTDIR%flame_point.vert.spv"
if errorlevel 1 ( echo FAILED: flame_point.vert & exit /b 1 )
%GLSLC% -fshader-stage=frag "%OUTDIR%flame_point.frag" -o "%OUTDIR%flame_point.frag.spv"
if errorlevel 1 ( echo FAILED: flame_point.frag & exit /b 1 )

echo Compiling flame_smoke shaders...
%GLSLC% -fshader-stage=vert "%OUTDIR%flame_smoke.vert" -o "%OUTDIR%flame_smoke.vert.spv"
if errorlevel 1 ( echo FAILED: flame_smoke.vert & exit /b 1 )
%GLSLC% -fshader-stage=frag "%OUTDIR%flame_smoke.frag" -o "%OUTDIR%flame_smoke.frag.spv"
if errorlevel 1 ( echo FAILED: flame_smoke.frag & exit /b 1 )

echo Compiling flame_pbvr shaders...
%GLSLC% -fshader-stage=vert "%OUTDIR%flame_pbvr.vert" -o "%OUTDIR%flame_pbvr.vert.spv"
if errorlevel 1 ( echo FAILED: flame_pbvr.vert & exit /b 1 )
%GLSLC% -fshader-stage=frag "%OUTDIR%flame_pbvr.frag" -o "%OUTDIR%flame_pbvr.frag.spv"
if errorlevel 1 ( echo FAILED: flame_pbvr.frag & exit /b 1 )

echo All shaders compiled successfully.
endlocal
