# record_physics_baseline.ps1 - Record the Physics refactoring baseline (docs/todo/PLAN_physics_refactoring.md Phase 0)
#
# Runs representative PhysicsView scenarios several times in Release and records, per scenario:
#   - wall time (median / min / max) of one PhysicsView process, including start-up and Vulkan init
#   - every Get*/Is* response (particle counts, positions, speeds, fuel balance, ...) of the first run
#   - whether all repeats produced identical responses (the deterministic contract), ignoring timing keys
#   - SHA-256 of every screenshot a scenario saves, and whether they matched across repeats
# Start-up cost is measured separately with 00_smoke_startup and reported as "startup", so
# (median - startup) approximates the simulation + rendering time of the scenario.
#
# Usage (from the repository root or Physics\PhysicsView):
#   .\record_physics_baseline.ps1                      # Release, 5 repeats, default scenario list
#   .\record_physics_baseline.ps1 -Repeat 3 -Filter '49_*'
#   .\record_physics_baseline.ps1 -OutFile baseline.json
#
# Release only: Debug is ~10x slower and its timings are meaningless as a baseline.

param(
    [ValidateRange(1, 50)]
    [int]$Repeat = 5,
    [string[]]$Scenarios = @(),
    [string]$Filter = "*",
    [string]$OutFile = "",
    [ValidateRange(1, 3600)]
    [int]$TimeoutSeconds = 300
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot  = $scriptDir
while ($repoRoot -and -not (Test-Path (Join-Path $repoRoot "CMakePresets.json"))) {
    $parent = Split-Path -Parent $repoRoot
    if ($parent -eq $repoRoot) { $repoRoot = $null; break }
    $repoRoot = $parent
}
if (-not $repoRoot) { Write-Host "ERROR: CMakePresets.json not found"; exit 1 }

$exe     = Join-Path $repoRoot "build\windows-release\Physics\PhysicsView.exe"
$scenDir = Join-Path $scriptDir "scenarios"
if (-not (Test-Path $exe)) { Write-Host "ERROR: $exe not found (build windows-release first)"; exit 1 }

if ($Scenarios.Count -eq 0) {
    $Scenarios = @(
        # fluid (DFSPH / PBSPH / WCSPH CPU, CSPH GPU) and the wall/emitter paths
        "10_fluid_dfsph_pool_settle", "11_fluid_pbsph_pool_settle", "12_fluid_wcsph_pool_settle",
        "14_fluid_csph_pool_settle", "15_fluid_pbsph_dam_break", "61_emit_dfsph_faucet",
        # rigid / soft
        "22_rigid_stacking", "23_rigid_billiards", "33_soft_cloth_sphere", "37_soft_jelly_drop",
        # coupling
        "50_couple_rigid_fluid_oneway", "51_couple_rigid_fluid_twoway", "53_couple_soft_fluid",
        # hair
        "06_smoke_hair_dynamics", "09_smoke_hair_lod",
        # cloud
        "56_cloud_form_evaporate_reform",
        # flame, solid combustion coupling, rigid clock
        "49_flame_physical_combustion", "49_flame_combustion_source_off", "49_flame_combustion_resolution",
        "49_flame_object_rigid", "78_flame_cylinder_circulation",
        # rendering: SSFR, glTF + shadows, Flame PBVR, cloud PBVR
        "85_render_ssfr_scene", "86_render_ssfr_anisotropic_kernel", "84_render_shadows",
        "48_flame_pbvr_ensembles", "65_cloud_pbvr_render"
    )
}
$Scenarios = $Scenarios | Where-Object { $_ -like $Filter }

# Responses that depend on wall-clock time or the GPU and so are not part of the determinism check.
$timingKey = '(?i)(gpums|computems|\bms\b|elapsed|fps|frametime)'

function Invoke-Once([string]$name) {
    $json = Join-Path $scenDir "$name.json"
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $exe
    $psi.Arguments = "--run-scenario `"$json`""
    $psi.WorkingDirectory = $repoRoot
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError  = $true
    $psi.UseShellExecute = $false
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $p = [Diagnostics.Process]::Start($psi)
    $errTask = $p.StandardError.ReadToEndAsync()
    $outTask = $p.StandardOutput.ReadToEndAsync()
    if (-not $p.WaitForExit($TimeoutSeconds * 1000)) { $p.Kill(); $sw.Stop(); return @{ ok = $false; ms = $sw.ElapsedMilliseconds; metrics = @(); shots = @{}; note = "timeout" } }
    $p.WaitForExit()
    $sw.Stop()
    $lines = ($outTask.Result -split "`r?`n")
    $passed = [bool]($lines | Where-Object { $_ -match '^\[Scenario\] PASSED' })
    if (-not $passed) {
        $log = Join-Path $env:TEMP ("baseline_fail_{0}_{1:yyyyMMdd_HHmmss_fff}.log" -f $name, (Get-Date))
        Set-Content -Path $log -Value ($outTask.Result + "`n--- exit code $($p.ExitCode) ---`n" + $errTask.Result) -Encoding UTF8
        Write-Host "  FAILED run of $name (exit $($p.ExitCode)); output saved to $log"
    }
    $metrics = New-Object System.Collections.Generic.List[string]
    $shots = @{}
    $cmd = $null
    foreach ($l in $lines) {
        if ($l -match '^\[Scenario\]   > (.*)$') { $cmd = $Matches[1]; continue }
        if ($l -match '^\[Scenario\]   < (.*)$' -and $cmd) {
            $resp = $Matches[1]
            # Mask wall-clock / GPU-time fields inside multi-value responses (e.g. GetFlamePBVRStats: gpuMs=...).
            $resp = [regex]::Replace($resp, '(?i)((?:gpu|compute|cpu|update)?ms\w*)\s*[=:]\s*-?[\d.]+(?:e[-+]?\d+)?', '$1=<t>')
            if ($cmd -match '^(Get|Is)' -and $cmd -notmatch $timingKey) { $metrics.Add("$cmd = $resp") }
            elseif ($cmd -match '^(Get|Is)') { $metrics.Add("$cmd = <timing>") }
            if ($cmd -match '^SaveScreenshot:(.+)$') {
                $path = $Matches[1]
                if (-not [IO.Path]::IsPathRooted($path)) { $path = Join-Path $repoRoot $path }
                if (Test-Path $path) { $shots[$path] = (Get-FileHash $path -Algorithm SHA256).Hash }
            }
            $cmd = $null
        }
    }
    return @{ ok = $passed; ms = $sw.ElapsedMilliseconds; metrics = $metrics.ToArray(); shots = $shots; note = "" }
}

function Get-Median([double[]]$v) {
    $s = $v | Sort-Object
    $n = $s.Count
    if ($n % 2) { return $s[($n - 1) / 2] }
    return ($s[$n / 2 - 1] + $s[$n / 2]) / 2
}

$commit = (git -C $repoRoot rev-parse --short HEAD 2>$null)
$physicsCommit = (git -C (Join-Path $repoRoot "Physics") rev-parse --short HEAD 2>$null)
$gpuLine = ""
$report = [ordered]@{
    date = (Get-Date -Format "yyyy-MM-dd HH:mm:ss")
    phantomCommit = $commit
    physicsCommit = $physicsCommit
    repeat = $Repeat
    startupMsMedian = 0
    scenarios = @()
}

# Start-up cost.
$startup = @()
for ($i = 0; $i -lt $Repeat; $i++) { $startup += (Invoke-Once "00_smoke_startup").ms }
$startupMed = Get-Median $startup
$report.startupMsMedian = $startupMed
Write-Host ("startup (00_smoke_startup) median {0:N0} ms over {1} runs" -f $startupMed, $Repeat)

$allOk = $true
foreach ($name in $Scenarios) {
    if (-not (Test-Path (Join-Path $scenDir "$name.json"))) { Write-Host "SKIP (missing): $name"; continue }
    $times = @(); $first = $null; $deterministic = $true; $shotsSame = $true; $ok = $true; $note = ""
    for ($i = 0; $i -lt $Repeat; $i++) {
        $r = Invoke-Once $name
        $times += $r.ms
        if (-not $r.ok) { $ok = $false; $note = if ($r.note) { $r.note } else { "scenario failed" } }
        if ($null -eq $first) { $first = $r }
        else {
            if (($r.metrics -join "`n") -ne ($first.metrics -join "`n")) { $deterministic = $false }
            foreach ($k in $first.shots.Keys) { if ($r.shots[$k] -ne $first.shots[$k]) { $shotsSame = $false } }
        }
    }
    if (-not $ok) { $allOk = $false }
    $med = Get-Median $times
    $min = ($times | Measure-Object -Minimum).Minimum
    $max = ($times | Measure-Object -Maximum).Maximum
    $spread = if ($med -gt 0) { 100.0 * ($max - $min) / $med } else { 0 }
    Write-Host ("{0,-40} {1,8:N0} ms  [{2:N0}..{3:N0}] spread {4,4:N1}%  ok={5} deterministic={6} shots={7}" -f $name, $med, $min, $max, $spread, $ok, $deterministic, $(if ($first.shots.Count) { $shotsSame } else { "-" }))
    $report.scenarios += [ordered]@{
        name = $name; ok = $ok; note = $note
        medianMs = $med; minMs = $min; maxMs = $max; spreadPercent = [math]::Round($spread, 1)
        simMsApprox = [math]::Max(0, $med - $startupMed)
        deterministic = $deterministic
        screenshots = $first.shots
        screenshotsIdentical = $shotsSame
        metrics = $first.metrics
    }
}

if (-not $OutFile) { $OutFile = Join-Path $scriptDir "baseline_$(Get-Date -Format 'yyyyMMdd').json" }
$report | ConvertTo-Json -Depth 6 | Set-Content -Path $OutFile -Encoding UTF8
Write-Host "Wrote $OutFile"
if (-not $allOk) { exit 1 }
