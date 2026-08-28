@echo off
@echo off
setlocal

pushd "%~dp0"

if "%VULKAN_SDK%"=="" (
  echo [ERROR] VULKAN_SDK is not set.
  exit /b 1
)

"%VULKAN_SDK%\Bin\glslc.exe" ssfr_depth.vert -o ssfr_depth.vert.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_depth.frag -o ssfr_depth.frag.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_thickness.vert -o ssfr_thickness.vert.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_thickness.frag -o ssfr_thickness.frag.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_bilateral.vert -o ssfr_bilateral.vert.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_bilateral.frag -o ssfr_bilateral.frag.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_reflection.vert -o ssfr_reflection.vert.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_reflection.frag -o ssfr_reflection.frag.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_refraction.vert -o ssfr_refraction.vert.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_refraction.frag -o ssfr_refraction.frag.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_composite.vert -o ssfr_composite.vert.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" ssfr_composite.frag -o ssfr_composite.frag.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" skybox.vert -o skybox.vert.spv
if errorlevel 1 goto :fail
"%VULKAN_SDK%\Bin\glslc.exe" skybox.frag -o skybox.frag.spv
if errorlevel 1 goto :fail

echo Done.
popd
exit /b 0

:fail
echo [ERROR] SSFR shader compile failed.
popd
exit /b 1
