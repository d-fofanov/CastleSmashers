<#
.SYNOPSIS
  Runs the Unity Test Framework suites in batch mode.

.EXAMPLE
  .\RunTests.ps1                      # EditMode + PlayMode on this project
  .\RunTests.ps1 -Platform EditMode -Filter "Phys.AvbdGpu.Tests.ReferenceTests"
  .\RunTests.ps1 -ProjectPath C:\Personal\Phys\EngineGPUCI   # a mirror project (when the editor has this one open)

.NOTES
  The Unity editor must not have the target project open (batch mode needs the project lock).
#>
param(
    [ValidateSet("EditMode", "PlayMode", "Both")] [string]$Platform = "Both",
    [string]$Filter = "",
    [string]$ProjectPath = $PSScriptRoot,
    [string]$UnityExe = "C:\Program Files\Unity\Hub\Editor\6000.3.12f1\Editor\Unity.exe",
    [string]$ResultsDir = (Join-Path $PSScriptRoot "TestResults"),
    [string]$GraphicsApi = "-force-d3d12"   # the player runs D3D12; "" leaves the editor default (D3D11)
)

New-Item -ItemType Directory -Force $ResultsDir | Out-Null
$platforms = if ($Platform -eq "Both") { @("EditMode", "PlayMode") } else { @($Platform) }
$failed = $false
foreach ($p in $platforms) {
    $results = Join-Path $ResultsDir "$p.xml"
    $log = Join-Path $ResultsDir "$p.log"
    $args = @("-batchmode", "-projectPath", $ProjectPath, "-runTests", "-testPlatform", $p, "-testResults", $results, "-logFile", $log)
    if ($GraphicsApi -ne "") { $args += $GraphicsApi }
    # No -nographics: the GPU solver tests dispatch compute shaders and need a graphics device.
    if ($Filter -ne "") { $args += @("-testFilter", $Filter) }
    elseif ($p -eq "EditMode") { $args += @("-testFilter", "!Phys.AvbdGpu.Tests.PerformanceTests;!Phys.AvbdGpu.Tests.DiagnosticTests") }
    Write-Host "== $p =="
    $proc = Start-Process -FilePath $UnityExe -ArgumentList $args -Wait -PassThru -NoNewWindow
    if (Test-Path $results) {
        [xml]$xml = Get-Content $results
        $run = $xml.'test-run'
        Write-Host ("{0}: total {1} passed {2} failed {3} skipped {4} (exit {5})" -f $p, $run.total, $run.passed, $run.failed, $run.skipped, $proc.ExitCode)
        $xml.SelectNodes("//test-case[@result='Failed']") | ForEach-Object {
            Write-Host ("  FAILED {0}" -f $_.fullname)
            $msg = $_.failure.message.'#cdata-section'
            if ($msg) { Write-Host ("    " + ($msg -split "`n")[0]) }
        }
        if ([int]$run.failed -gt 0 -or $proc.ExitCode -ne 0) { $failed = $true }
    } else {
        Write-Host "${p}: no results file (exit $($proc.ExitCode)); see $log"
        $failed = $true
    }
}
if ($failed) { exit 1 } else { exit 0 }
