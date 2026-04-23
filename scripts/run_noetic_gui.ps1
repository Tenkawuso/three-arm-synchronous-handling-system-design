$ErrorActionPreference = 'Stop'

param(
  [ValidateSet('moveit', 'gazebo')]
  [string]$Mode = 'moveit'
)

$dockerBin = 'C:\Program Files\Docker\Docker\resources\bin'
if (Test-Path $dockerBin) {
  $env:PATH = "$dockerBin;" + $env:PATH
}

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
  throw 'Docker command not found. Please install Docker Desktop first.'
}

$projectRoot = Split-Path $PSScriptRoot -Parent
$composeFile = Join-Path $projectRoot 'docker-compose.noetic.yml'
$vcxsrv = 'C:\Program Files\VcXsrv\vcxsrv.exe'

if (-not (Get-Process -Name vcxsrv -ErrorAction SilentlyContinue)) {
  if (Test-Path $vcxsrv) {
    Write-Host 'Starting VcXsrv...'
    Start-Process -FilePath $vcxsrv -ArgumentList ':0 -multiwindow -clipboard -ac -nowgl' -WindowStyle Minimized | Out-Null
    Start-Sleep -Seconds 2
  } else {
    Write-Warning 'VcXsrv not found. RViz/Gazebo windows may not appear. Install VcXsrv if needed.'
  }
}

$env:DISPLAY = 'host.docker.internal:0.0'

Write-Host 'Starting Docker container...'
docker compose -f $composeFile up -d --build

Write-Host 'Stopping previous simulation processes (if any)...'
docker exec yjg-noetic bash -lc "pkill -f 'my_three_arms_moveit_config demo.launch' || true; pkill -f 'my_three_arms_moveit_config demo_gazebo.launch' || true; pkill -f move_group || true"

switch ($Mode) {
  'gazebo' { $launchCmd = 'roslaunch my_three_arms_moveit_config demo_gazebo.launch' }
  default { $launchCmd = 'roslaunch my_three_arms_moveit_config demo.launch use_rviz:=true' }
}

$containerCmd = @"
set -e
source /opt/ros/noetic/setup.bash
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
docker exec -it yjg-noetic bash -lc $containerCmd
