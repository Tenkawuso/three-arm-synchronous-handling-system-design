param(
  [ValidateSet('moveit', 'gazebo')]
  [string]$Mode = 'moveit'
)

$ErrorActionPreference = 'Stop'

$dockerBin = 'C:\Program Files\Docker\Docker\resources\bin'
if (Test-Path $dockerBin) {
  $env:PATH = "$dockerBin;" + $env:PATH
}

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
  throw 'Docker command not found. Please install Docker Desktop first.'
}

function Test-DockerDaemon {
  docker info *> $null
  return ($LASTEXITCODE -eq 0)
}

function Ensure-DockerDaemon {
  if (Test-DockerDaemon) {
    return
  }

  $dockerDesktopExe = 'C:\Program Files\Docker\Docker\Docker Desktop.exe'
  if (Test-Path $dockerDesktopExe) {
    Write-Host 'Docker daemon is not ready, starting Docker Desktop...'
    Start-Process -FilePath $dockerDesktopExe | Out-Null
  } else {
    throw 'Docker daemon is not ready and Docker Desktop executable was not found.'
  }

  $ready = $false
  for ($i = 0; $i -lt 90; $i++) {
    Start-Sleep -Seconds 2
    if (Test-DockerDaemon) {
      $ready = $true
      break
    }
  }

  if (-not $ready) {
    throw 'Docker daemon did not become ready in time. Open Docker Desktop manually and wait until Engine is running.'
  }
}

function Wait-ContainerRunning {
  $ready = $false
  for ($i = 0; $i -lt 30; $i++) {
    $state = docker inspect -f "{{.State.Running}}" yjg-noetic 2>$null
    if (($LASTEXITCODE -eq 0) -and ($state.Trim() -eq 'true')) {
      $ready = $true
      break
    }
    Start-Sleep -Seconds 2
  }

  if (-not $ready) {
    throw 'Container yjg-noetic is not running. Check Docker Desktop and run docker compose logs if needed.'
  }
}

$projectRoot = Split-Path $PSScriptRoot -Parent
$composeFile = Join-Path $projectRoot 'docker-compose.noetic.yml'
$vcxsrv = 'C:\Program Files\VcXsrv\vcxsrv.exe'

$displayNumber = 1
$displayHost = "host.docker.internal:$displayNumber.0"
$vcxArgs = ":$displayNumber -multiwindow -clipboard -ac -wgl"

function Test-VcXsrvDisplayRunning {
  param([int]$DisplayNumber)

  $displayRegex = "(^|\s):$DisplayNumber(\.0)?(\s|$)"
  try {
    $procs = Get-CimInstance Win32_Process -Filter "Name = 'vcxsrv.exe'" -ErrorAction SilentlyContinue
    if (-not $procs) {
      return $false
    }
    foreach ($p in $procs) {
      if ($p.CommandLine -and ($p.CommandLine -match $displayRegex)) {
        return $true
      }
    }
    return $false
  } catch {
    return $false
  }
}

if (-not (Test-VcXsrvDisplayRunning -DisplayNumber $displayNumber)) {
  if (Test-Path $vcxsrv) {
    Write-Host "Starting VcXsrv on :$displayNumber ..."
    Start-Process -FilePath $vcxsrv -ArgumentList $vcxArgs -WindowStyle Minimized | Out-Null
    Start-Sleep -Seconds 2
  } else {
    Write-Warning 'VcXsrv not found. RViz/Gazebo windows may not appear. Install VcXsrv if needed.'
  }
} else {
  Write-Host "VcXsrv already running on :$displayNumber"
}

Ensure-DockerDaemon

$env:DISPLAY = $displayHost

Write-Host 'Starting Docker container...'
docker compose -f $composeFile up -d --build
if ($LASTEXITCODE -ne 0) {
  throw 'Failed to start Docker container with docker compose.'
}

Wait-ContainerRunning

Write-Host 'Stopping previous simulation processes (if any)...'
$cleanupCmd = @"
set +e
if command -v pkill >/dev/null 2>&1; then
  pkill -f '[r]oslaunch my_three_arms_moveit_config demo.launch' || true
  pkill -f '[r]oslaunch my_three_arms_moveit_config demo_gazebo.launch' || true
  pkill -f '[m]ove_group' || true
fi
exit 0
"@
docker exec yjg-noetic bash -lc $cleanupCmd *> $null

switch ($Mode) {
  'gazebo' { $launchCmd = 'roslaunch my_three_arms_moveit_config demo_gazebo.launch' }
  default { $launchCmd = 'roslaunch my_three_arms_moveit_config demo.launch use_rviz:=true' }
}

$containerCmd = @"
set -e
source /opt/ros/noetic/setup.bash
export DISPLAY=$displayHost
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export QT_XCB_GL_INTEGRATION=none
cd /workspace/project/catkin_ws
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true
catkin_make
source /workspace/project/catkin_ws/devel/setup.bash
$launchCmd
"@

Write-Host "Launching $Mode simulation..."
$useTty = $true
try {
  if ([Console]::IsInputRedirected -or [Console]::IsOutputRedirected) {
    $useTty = $false
  }
} catch {
  $useTty = $false
}

if ($useTty) {
  docker exec -it yjg-noetic bash -lc $containerCmd
} else {
  docker exec yjg-noetic bash -lc $containerCmd
}

if ($LASTEXITCODE -ne 0) {
  throw 'Failed to launch GUI simulation in container.'
}
