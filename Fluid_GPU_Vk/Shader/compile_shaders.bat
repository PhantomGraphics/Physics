@echo off
setlocal

set GLSLC="%VULKAN_SDK%\Bin\glslc.exe"
if not exist %GLSLC% (
    echo [ERROR] glslc.exe not found. VULKAN_SDK=%VULKAN_SDK%
    exit /b 1
)

set OUTDIR=%~dp0

echo Compiling CSPH GPU compute shaders...
%GLSLC% -fshader-stage=comp "%OUTDIR%csph_clear.comp" -o "%OUTDIR%csph_clear.comp.spv"
if errorlevel 1 ( echo FAILED: csph_clear.comp & exit /b 1 )
%GLSLC% -fshader-stage=comp "%OUTDIR%csph_count.comp" -o "%OUTDIR%csph_count.comp.spv"
if errorlevel 1 ( echo FAILED: csph_count.comp & exit /b 1 )
%GLSLC% -fshader-stage=comp "%OUTDIR%csph_prefix.comp" -o "%OUTDIR%csph_prefix.comp.spv"
if errorlevel 1 ( echo FAILED: csph_prefix.comp & exit /b 1 )
%GLSLC% -fshader-stage=comp "%OUTDIR%csph_scatter.comp" -o "%OUTDIR%csph_scatter.comp.spv"
if errorlevel 1 ( echo FAILED: csph_scatter.comp & exit /b 1 )
%GLSLC% -fshader-stage=comp "%OUTDIR%csph_density.comp" -o "%OUTDIR%csph_density.comp.spv"
if errorlevel 1 ( echo FAILED: csph_density.comp & exit /b 1 )
%GLSLC% -fshader-stage=comp "%OUTDIR%csph_force.comp" -o "%OUTDIR%csph_force.comp.spv"
if errorlevel 1 ( echo FAILED: csph_force.comp & exit /b 1 )
%GLSLC% -fshader-stage=comp "%OUTDIR%csph_integrate.comp" -o "%OUTDIR%csph_integrate.comp.spv"
if errorlevel 1 ( echo FAILED: csph_integrate.comp & exit /b 1 )

echo Done.
endlocal
