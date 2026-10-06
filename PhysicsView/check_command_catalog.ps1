# check_command_catalog.ps1 - static consistency check of the PhysicsView command catalog
#
# PhysicsView routes commands through several sub-dispatchers (rigid / soft / flame / cloud) whose
# handlers have side effects, so the in-app "CheckCommandCatalog" probe other viewers use is unsafe
# here. This checks the same two properties from the sources instead:
#   1. every command name a scenario uses is in CommandDispatcher::commandCatalog()
#   2. every catalog name appears as a string literal ("Name" or "Name:...") in a *CommandDispatcher*.cpp outside the
#      catalog block (i.e. some route() really handles it)
# Usage: .\Physics\PhysicsView\check_command_catalog.ps1      (exit code = number of problems)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$mainSrc   = Join-Path $scriptDir "CommandDispatcher.cpp"
$text      = [System.IO.File]::ReadAllText($mainSrc)

$start = $text.IndexOf("commandCatalog() const {")
if ($start -lt 0) { Write-Host "ERROR: commandCatalog() not found"; exit 1 }
$end = $text.IndexOf("`n}", $start)
$block = $text.Substring($start, $end - $start)
$catalog = [regex]::Matches($block, '\{"([A-Za-z0-9_]+)",') | ForEach-Object { $_.Groups[1].Value } | Sort-Object -Unique

# source text without the catalog block, for the "is it routed" check
$routing = $text.Remove($start, $end - $start)
foreach ($f in Get-ChildItem $scriptDir -Filter "*CommandDispatcher*.cpp") {
    if ($f.FullName -ne $mainSrc) { $routing += [System.IO.File]::ReadAllText($f.FullName) }
}

$problems = 0
foreach ($name in $catalog) {
    if ($routing -notmatch ('"' + [regex]::Escape($name) + '[":]')) {
        Write-Host "UNROUTED catalog entry: $name"
        $problems++
    }
}

$used = @{}
foreach ($s in Get-ChildItem (Join-Path $scriptDir "scenarios") -Filter "*.json") {
    $json = [System.IO.File]::ReadAllText($s.FullName) | ConvertFrom-Json
    foreach ($step in $json.steps) {
        if (-not $step.command) { continue }
        $name = ($step.command -split '[:,]')[0]
        if (-not $used.ContainsKey($name)) { $used[$name] = $s.Name }
    }
}
foreach ($name in $used.Keys | Sort-Object) {
    # NoSuchCommand is the deliberate unknown-command probe (command_catalog.json)
    if ($name -eq "NoSuchCommand") { continue }
    if ($catalog -notcontains $name) {
        Write-Host "MISSING from catalog: $name (used by $($used[$name]))"
        $problems++
    }
}

if ($problems -eq 0) { Write-Host "Command catalog OK ($($catalog.Count) entries, $($used.Count) scenario commands)" }
exit $problems
