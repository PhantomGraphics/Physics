# run_flame_pbvr_evaluation.ps1 - Flame GPU PBVR noise vs ensemble count
# (docs/todo/PLAN_flame_sph_pbvr_improvement.md Phase 4 completion criterion).
#
# On one static Flame frame, captures the PBVR image averaged over exactly R
# ensembles (Manual R, target = R) for R in 1,2,4,8 -- $Trials independent
# captures each -- plus a reference averaged over $ReferenceEnsembles
# ensembles, then reports RMSE(image_R, reference) and the log-log slope of
# RMSE against R. Independent Monte-Carlo ensembles predict RMSE ~ 1/sqrt(R),
# i.e. a slope of -0.5. The captures are ACES-tone-mapped 8-bit PNGs: at small
# R a pixel is nearly binary (opaque sub-particle or not) and the bounded
# display mapping compresses those large deviations, flattening the low-R end.
# The pass/fail check therefore uses the tail slope (the largest three R); the
# full-range slope is reported alongside.
#
# Usage: .\Physics\PhysicsView\run_flame_pbvr_evaluation.ps1 [-Configuration Debug|Release] [-Trials 4]
[CmdletBinding()]
param(
    [ValidateSet('Debug','Release')][string]$Configuration = 'Debug',
    [ValidateRange(1,16)][int]$Trials = 4,
    [ValidateRange(64,4096)][int]$ReferenceEnsembles = 1024,
    [string]$OutputDirectory = 'flame-pbvr-evaluation',
    [double]$SlopeTolerance = 0.1
)
$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$exe = Join-Path $root "build/windows-$($Configuration.ToLower())/Physics/PhysicsView.exe"
if (!(Test-Path -LiteralPath $exe)) { throw "Build PhysicsView first: $exe" }
$out = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputDirectory)
if (Test-Path -LiteralPath $out) { throw "Use a new output directory: $out" }
[IO.Directory]::CreateDirectory($out) | Out-Null

# ---- One scenario captures everything (a single process, one scene state) ----
$shot = { param($name) @{ command = "SaveScreenshot:$((Join-Path $out $name) -replace '\\','/')"; expect = 'OK' } }
$frames = { param($n) @{ command = 'GetStatus'; expect = 'OK'; repeat = $n } }
$steps = @(
    @{ command = 'SetFlamePage:true'; expect = 'OK' },
    @{ command = 'SetFlameRunning:false'; expect = 'OK' },
    @{ command = 'FlameReset'; expect = 'OK' },
    @{ command = 'FlameStep:120'; expect = 'OK' },
    @{ command = 'SetUIVisible:false'; expect = 'OK' },
    @{ command = 'SetCameraOrbit:110,0,0.15'; expect = 'OK' },
    @{ command = 'SetFlameRenderParam:pbvrAdaptive,0'; expect = 'OK' },
    @{ command = 'SetFlameRenderParam:pbvrTemporalFrames,0'; expect = 'OK' },
    @{ command = 'SetFlameRenderMode:PBVR'; expect = 'OK' }
)
# Reference: R = 8 per frame until $ReferenceEnsembles have been averaged.
$steps += @{ command = 'SetFlameRenderParam:pbvrEnsembles,8'; expect = 'OK' }
$steps += @{ command = "SetFlameRenderParam:pbvrTargetEnsembles,$ReferenceEnsembles"; expect = 'OK' }
$steps += & $frames ([int]($ReferenceEnsembles / 8) + 8)
$steps += @{ command = 'GetFlamePBVRStat:displayedEnsembles'; expect_range = "$ReferenceEnsembles,$ReferenceEnsembles" }
$steps += & $shot 'reference.png'

$rs = @(1, 2, 4, 8, 16, 32) # R > 8 spans several frames of 8 ensembles
$toggle = 0
foreach ($r in $rs) {
    $steps += @{ command = "SetFlameRenderParam:pbvrEnsembles,$([math]::Min($r, 8))"; expect = 'OK' }
    for ($t = 0; $t -lt $Trials; ++$t) {
        # Restart the history with a negligible shading change (the renderer
        # resets on any shading change); each restart draws fresh ensembles.
        # Target first: a scenario step spans more than one rendered frame, so a
        # reset issued while the old (larger) target is active would keep
        # refining past R before the new target arrives.
        $steps += @{ command = "SetFlameRenderParam:pbvrTargetEnsembles,$r"; expect = 'OK' }
        $toggle = 1 - $toggle
        $steps += @{ command = "SetFlameRenderParam:exposure,$(4.0 + 0.0001 * $toggle)"; expect = 'OK' }
        $steps += & $frames ([int][math]::Ceiling($r / 8) + 4)
        $steps += @{ command = 'GetFlamePBVRStat:displayedEnsembles'; expect_range = "$r,$r" }
        $steps += & $shot ("r{0}_t{1}.png" -f $r, $t)
    }
}
$steps += @{ command = 'SetFlameRenderParam:exposure,4'; expect = 'OK' }
$steps += @{ command = 'SetUIVisible:true'; expect = 'OK' }

$scenario = Join-Path $out 'evaluation.json'
# No BOM: the C++ scenario parser rejects it (same as run_gp_evaluation.ps1).
[IO.File]::WriteAllText($scenario, (@{ name = 'flame_pbvr_evaluation'; steps = $steps } | ConvertTo-Json -Depth 5),
    [Text.UTF8Encoding]::new($false))
Write-Host "Running $scenario ..."
$log = Join-Path $out 'run.log'
$process = Start-Process -FilePath $exe -ArgumentList @('--run-scenario', ('"' + $scenario + '"')) -WorkingDirectory $PSScriptRoot `
    -WindowStyle Hidden -PassThru -RedirectStandardOutput $log -RedirectStandardError (Join-Path $out 'run.err.log')
$null = $process.Handle # cache the handle, or ExitCode reads back empty after exit
$process.WaitForExit()
if ($process.ExitCode -ne 0) { throw "Scenario failed (exit $($process.ExitCode)), see $log" }

# ---- RMSE on [0,1] RGB over the whole frame -------------------------------------
Add-Type -ReferencedAssemblies System.Drawing -TypeDefinition @'
using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
public static class FlameRmse {
    static byte[] Load(string path, out int w, out int h) {
        using (var bmp = new Bitmap(path)) {
            w = bmp.Width; h = bmp.Height;
            var d = bmp.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
            var buf = new byte[d.Stride * h];
            Marshal.Copy(d.Scan0, buf, 0, buf.Length);
            bmp.UnlockBits(d);
            return buf;
        }
    }
    public static double Rmse(string a, string b) {
        int wa, ha, wb, hb;
        var x = Load(a, out wa, out ha);
        var y = Load(b, out wb, out hb);
        if (wa != wb || ha != hb) throw new ArgumentException("size mismatch");
        double sum = 0;
        for (int i = 0; i < x.Length; ++i) { double d = (x[i] - y[i]) / 255.0; sum += d * d; }
        return Math.Sqrt(sum / x.Length);
    }
}
'@

$reference = Join-Path $out 'reference.png'
$rows = foreach ($r in $rs) {
    $vals = for ($t = 0; $t -lt $Trials; ++$t) { [FlameRmse]::Rmse((Join-Path $out ("r{0}_t{1}.png" -f $r, $t)), $reference) }
    $mean = ($vals | Measure-Object -Average).Average
    [pscustomobject]@{ R = $r; RMSE = $mean; RMSEx_sqrtR = $mean * [math]::Sqrt($r); Trials = $Trials }
}
$rows | Export-Csv -LiteralPath (Join-Path $out 'rmse.csv') -NoTypeInformation
$rows | Format-Table -AutoSize | Out-String | Write-Host

# Least-squares slope of log(RMSE) on log(R).
function Get-Slope($subset) {
    $xs = @($subset | ForEach-Object { [math]::Log($_.R) })
    $ys = @($subset | ForEach-Object { [math]::Log($_.RMSE) })
    $mx = ($xs | Measure-Object -Average).Average
    $my = ($ys | Measure-Object -Average).Average
    $num = 0.0; $den = 0.0
    for ($i = 0; $i -lt $xs.Count; ++$i) { $num += ($xs[$i] - $mx) * ($ys[$i] - $my); $den += ($xs[$i] - $mx) * ($xs[$i] - $mx) }
    return $num / $den
}
$slope = Get-Slope $rows
$tailSlope = Get-Slope ($rows | Select-Object -Last 3)
$ok = [math]::Abs($tailSlope + 0.5) -le $SlopeTolerance
Write-Host ("log-log slope: full range {0:N3}, tail (R={1}) {2:N3} (expected -0.5 +- {3})  => {4}" -f `
    $slope, (($rows | Select-Object -Last 3 | ForEach-Object { $_.R }) -join ','), $tailSlope, $SlopeTolerance,
    $(if ($ok) { 'PASS' } else { 'FAIL' }))
@{ slope = $slope; tailSlope = $tailSlope; tolerance = $SlopeTolerance; pass = $ok
   referenceEnsembles = $ReferenceEnsembles; trials = $Trials } |
    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out 'summary.json') -Encoding UTF8
if (!$ok) { exit 1 }
