<#
.SYNOPSIS
  Records the README video: the Royal citadel undermined by the mining charges of undermining.txt, captured frame by frame from
  the castle player and encoded with ffmpeg at half speed.

.EXAMPLE
  .\Tools\video\record_undermining.ps1 -Build            # build the castle player first (the editor must not have the project open)
  .\Tools\video\record_undermining.ps1                   # record with the player already built
  .\Tools\video\record_undermining.ps1 -Frames 300 -SuperSize 1 -Width 960 -Height 540   # a quick preview

.NOTES
  The player runs one solver step per frame with the clock fixed at the step (Time.captureFramerate), so the run takes as long
  as the captures take. Every step is captured at SuperSize times the window size and downscaled by ffmpeg (the anti-aliasing);
  the movie plays 30 captures per second, half the solver's 60: the x0.5 slow motion. Needs ffmpeg on the PATH.
#>
param(
    [switch]$Build,
    [string]$Exe = "C:\Personal\Phys\build\EngineGPU\Castle.exe",
    [string]$UnityExe = "C:\Program Files\Unity\Hub\Editor\6000.3.12f1\Editor\Unity.exe",
    [string]$Plan = (Join-Path $PSScriptRoot "undermining.txt"),
    [string]$Out = (Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) "docs\undermining"),
    [string]$FramesDir = (Join-Path $env:TEMP "undermining_frames"),
    [int]$Frames = 840,
    [int]$Width = 1920,
    [int]$Height = 1080,
    [int]$SuperSize = 2,
    [string]$Camera = "-avbd-yaw 18 -avbd-pitch 12 -avbd-distance 112 -avbd-dolly -1.2 -avbd-orbit 0.5",
    [int]$Fps = 30,
    [string]$Bitrate = "2700k",   # two-pass H.264: 28 s stay under the 10 MB GitHub allows a video on a free plan
    [int]$PosterFrame = 430
)
$project = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
if ($Build) {
    $log = Join-Path $env:TEMP "undermining_build.log"
    $args = @("-batchmode", "-quit", "-projectPath", $project, "-executeMethod", "Phys.Demo.Editor.BuildDemo.Build", "-buildScene", "Castle", "-buildPath", $Exe, "-logFile", $log)
    $p = Start-Process -FilePath $UnityExe -ArgumentList $args -PassThru -Wait
    if ($p.ExitCode -ne 0) { Write-Error "build failed (exit $($p.ExitCode)); see $log"; exit 1 }
}
if (Test-Path $FramesDir) { Remove-Item -Recurse -Force $FramesDir }
$log = Join-Path $env:TEMP "undermining_record.log"
$all = "-avbd-scene 9 -avbd-nohud -avbd-record `"$FramesDir`" -avbd-supersize $SuperSize -avbd-frames $Frames -avbd-blasts `"$Plan`" " +
       "-screen-width $Width -screen-height $Height -screen-fullscreen 0 -logFile `"$log`" $Camera"
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$p = Start-Process -FilePath $Exe -ArgumentList $all -PassThru -Wait
$n = (Get-ChildItem $FramesDir -Filter *.png -ErrorAction SilentlyContinue | Measure-Object).Count
Write-Host ("recorded {0} frames in {1:F0} s (exit {2})" -f $n, $sw.Elapsed.TotalSeconds, $p.ExitCode)
if ($n -eq 0) { Write-Error "no frames; see $log"; exit 1 }

New-Item -ItemType Directory -Force (Split-Path $Out -Parent) | Out-Null
$frames = Join-Path $FramesDir "frame_%05d.png"
$scale = "scale=${Width}:${Height}:flags=lanczos"
# the movie: every capture at half the solver's rate, downscaled to the window size, two passes at the bitrate
$passLog = Join-Path $env:TEMP "undermining_x264"
& ffmpeg -loglevel error -y -framerate $Fps -i $frames -vf $scale -c:v libx264 -preset veryslow -tune animation -b:v $Bitrate -pass 1 -passlogfile $passLog -an -f null NUL
& ffmpeg -loglevel error -y -framerate $Fps -i $frames -vf $scale -c:v libx264 -preset veryslow -tune animation -b:v $Bitrate -pass 2 -passlogfile $passLog -pix_fmt yuv420p -movflags +faststart "$Out.mp4"
# the poster: one frame mid-fall
& ffmpeg -loglevel error -y -i (Join-Path $FramesDir ("frame_{0:D5}.png" -f $PosterFrame)) -vf $scale -q:v 3 "$Out.jpg"
# the preview: the fall of the towers and the keep as a small GIF (a README renders it without any upload)
$gifStart = 60
$gifFrames = [math]::Min($Frames - $gifStart, 630)
& ffmpeg -loglevel error -y -start_number $gifStart -framerate $Fps -i $frames -frames:v $gifFrames -vf "fps=8,scale=480:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" "$Out.gif"
Get-ChildItem "$Out.*" | ForEach-Object { "{0}  {1:N1} MB" -f $_.Name, ($_.Length / 1MB) }
