# run_physics_scenarios.ps1 - Run PhysicsView scenario tests
# Usage: .\Physics\PhysicsView\run_physics_scenarios.ps1 [-Configuration Debug|Release] [-Filter <wildcard>]
#                                                          [-Tag <name>...] [-ExcludeTag <name>...] [-List] [-FailFast] [-TimeoutSeconds 120]
# Run from the repository root or the Physics\PhysicsView directory.
#
# Renamed from run_fluid_scenarios.ps1 (2026-08) -- the scenario suite has covered
# PhysicsView as a whole (fluid + rigid + soft-body + coupling) since the
# RigidBodyView/SoftBodyView merges, not just the fluid solver.
#
# Examples:
#   .\run_physics_scenarios.ps1                       # everything except tags:["known-fail"]
#   .\run_physics_scenarios.ps1 -Filter '1*_fluid_*'   # only the 10-19 fluid group
#   .\run_physics_scenarios.ps1 -Tag slow              # only scenarios tagged "slow"
#   .\run_physics_scenarios.ps1 -ExcludeTag slow        # everything except "slow" (still excludes known-fail too)
#   .\run_physics_scenarios.ps1 -List                  # list matching scenarios (with tags) without running them
#   .\run_physics_scenarios.ps1 -FailFast              # stop at the first failure

param(
    [string]$Configuration = "Debug",
    [string]$Filter = "*",
    [string[]]$Tag = @(),
    [string[]]$ExcludeTag = @("known-fail"),
    [switch]$List,
    [switch]$FailFast,
    [ValidateRange(1, 2147483)]
    [int]$TimeoutSeconds = 120
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot  = $scriptDir
while ($repoRoot -and -not (Test-Path (Join-Path $repoRoot "CMakePresets.json"))) {
    $parent = Split-Path -Parent $repoRoot
    if ($parent -eq $repoRoot) { $repoRoot = $null; break }
    $repoRoot = $parent
}
if (-not $repoRoot) {
    Write-Host "ERROR: Could not locate CMake repository root (CMakePresets.json)"
    exit 1
}
$preset  = "windows-$($Configuration.ToLower())"
$exe     = Join-Path $repoRoot "build\$preset\Physics\PhysicsView.exe"
$scenDir = Join-Path $scriptDir "scenarios"

if (-not $List -and -not (Test-Path $exe)) {
    Write-Host "ERROR: Executable not found: $exe"
    exit 1
}

$allFiles = Get-ChildItem "$scenDir\*.json" | Sort-Object Name
if ($allFiles.Count -eq 0) {
    Write-Host "ERROR: No scenario JSON files found in $scenDir"
    exit 1
}

# Reads the JSON's top-level "tags" array (PowerShell side only -- the C++
# ScenarioRunner ignores unknown top-level keys). Malformed
# JSON or a missing/absent "tags" field both just mean "no tags".
function Get-ScenarioTags([string]$path) {
    try {
        $json = Get-Content -Raw -Encoding UTF8 -Path $path | ConvertFrom-Json -ErrorAction Stop
        if ($json.PSObject.Properties.Name -contains "tags") {
            return @($json.tags)
        }
    } catch {}
    return @()
}

$scenarios = @()
foreach ($f in $allFiles) {
    if ($f.BaseName -notlike $Filter) { continue }

    $tags = Get-ScenarioTags $f.FullName

    if ($Tag.Count -gt 0) {
        $included = $false
        foreach ($t in $Tag) { if ($tags -contains $t) { $included = $true; break } }
        if (-not $included) { continue }
    }

    $excluded = $false
    foreach ($t in $ExcludeTag) { if ($tags -contains $t) { $excluded = $true; break } }
    if ($excluded) { continue }

    $scenarios += [PSCustomObject]@{ File = $f; Tags = $tags }
}

if ($scenarios.Count -eq 0) {
    Write-Host "No scenarios matched (Filter='$Filter', Tag=[$($Tag -join ',')], ExcludeTag=[$($ExcludeTag -join ',')])."
    exit 1
}

if ($List) {
    foreach ($s in $scenarios) {
        $tagStr = if ($s.Tags.Count -gt 0) { " [$($s.Tags -join ',')]" } else { "" }
        Write-Host "$($s.File.BaseName)$tagStr"
    }
    Write-Host ""
    Write-Host "$($scenarios.Count) scenario(s) matched."
    exit 0
}

$passed = 0
$failed = 0
$failedNames = @()
$totalSw = [System.Diagnostics.Stopwatch]::StartNew()

foreach ($s in $scenarios) {
    Write-Host "Running: $($s.File.BaseName)"
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $proc = Start-Process -FilePath $exe `
        -ArgumentList "--run-scenario `"$($s.File.FullName)`"" `
        -PassThru -WindowStyle Hidden -WorkingDirectory $scriptDir -ErrorAction Stop
    $timedOut = $false
    try {
        # Retain the process handle so ExitCode remains available after exit.
        $null = $proc.Handle
        if (-not $proc.WaitForExit($TimeoutSeconds * 1000)) {
            $timedOut = $true
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
            $null = $proc.WaitForExit(5000)
        }
        $exitCode = if ($timedOut) { -1 } else { $proc.ExitCode }
    } finally {
        $proc.Dispose()
    }
    $sw.Stop()
    $elapsed = "{0:N1}s" -f $sw.Elapsed.TotalSeconds

    if (-not $timedOut -and $exitCode -eq 0) {
        Write-Host "  PASSED: $($s.File.BaseName) ($elapsed)"
        $passed++
    } else {
        if ($timedOut) { Write-Host "  TIMEOUT: $($s.File.BaseName) (limit ${TimeoutSeconds}s, $elapsed)" }
        else { Write-Host "  FAILED: $($s.File.BaseName) (exit $exitCode, $elapsed)" }
        $failed++
        $failedNames += $s.File.BaseName
        if ($FailFast) { break }
    }
}

$totalSw.Stop()
$totalElapsed = "{0:N1}s" -f $totalSw.Elapsed.TotalSeconds

Write-Host ""
Write-Host "Results: $passed/$($scenarios.Count) PASSED, $failed FAILED (total $totalElapsed)"
if ($failedNames.Count -gt 0) {
    Write-Host "Failed scenarios:"
    foreach ($n in $failedNames) { Write-Host "  - $n" }
}
exit $failed
