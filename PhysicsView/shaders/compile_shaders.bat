@echo off
setlocal

if "%VULKAN_SDK%"=="" (
  echo [ERROR] VULKAN_SDK is not set.
  exit /b 1
)

cd /d "%~dp0"

"%VULKAN_SDK%\Bin\glslc.exe" fluid_point.vert -o fluid_point.vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" fluid_point.frag -o fluid_point.frag.spv
"%VULKAN_SDK%\Bin\glslc.exe" line.vert -o line.vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" line.frag -o line.frag.spv
"%VULKAN_SDK%\Bin\glslc.exe" point.vert -o point.vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" point.frag -o point.frag.spv
"%VULKAN_SDK%\Bin\glslc.exe" triangle.vert -o triangle.vert.spv
"%VULKAN_SDK%\Bin\glslc.exe" triangle.frag -o triangle.frag.spv

echo Done.
