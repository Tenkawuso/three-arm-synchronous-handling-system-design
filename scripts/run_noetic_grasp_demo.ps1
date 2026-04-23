param(
  [ValidateSet('moveit', 'gazebo')]
  [string]$Mode = 'moveit',
  [switch]$SkipBuild
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

function Invoke-ContainerBash {
  param([string]$Command)

  docker exec yjg-noetic bash -lc ($Command -replace "`r", "")
}

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

$projectRoot = Split-Path $PSScriptRoot -Parent
$composeFile = Join-Path $projectRoot 'docker-compose.noetic.yml'
$vcxsrv = 'C:\Program Files\VcXsrv\vcxsrv.exe'

$displayNumber = 1
$displayHost = "host.docker.internal:$displayNumber.0"
$vcxArgs = ":$displayNumber -multiwindow -clipboard -ac -wgl"

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

Write-Host 'Stopping previous simulation/demo processes...'
$cleanupCmd = "if command -v pkill >/dev/null 2>&1; then pkill -f '[r]oslaunch my_three_arms_moveit_config demo.launch' || true; pkill -f '[r]oslaunch my_three_arms_moveit_config demo_gazebo.launch' || true; pkill -f '[m]ove_group' || true; pkill -f '[r]viz' || true; pkill -f '[t]hree_arm_coordinator.py' || true; fi"
Invoke-ContainerBash -Command $cleanupCmd *> $null

$containerPrefix = @"
set -e
source /opt/ros/noetic/setup.bash
export DISPLAY=$displayHost
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=0
export QT_XCB_GL_INTEGRATION=none
cd /workspace/project/catkin_ws
mkdir -p /workspace/project/.tmp
"@

if (-not $SkipBuild) {
  Write-Host 'Installing deps and building workspace...'
  $buildCmd = @"
$containerPrefix
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init || true
fi
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || true
catkin_make
"@

  Invoke-ContainerBash -Command $buildCmd
  if ($LASTEXITCODE -ne 0) {
    throw 'Workspace build failed in container.'
  }
} else {
  Write-Host 'Skipping build step (-SkipBuild).'
}

switch ($Mode) {
  'gazebo' { $launchCmd = 'roslaunch my_three_arms_moveit_config demo_gazebo.launch' }
  default { $launchCmd = 'roslaunch my_three_arms_moveit_config demo.launch use_rviz:=true' }
}

Write-Host "Starting $Mode simulation in background..."
$startLaunchCmd = @"
$containerPrefix
source /workspace/project/catkin_ws/devel/setup.bash
nohup $launchCmd > /workspace/project/.tmp/grasp_demo.launch.log 2>&1 &
echo `$! > /workspace/project/.tmp/grasp_demo.launch.pid
"@
Invoke-ContainerBash -Command $startLaunchCmd
if ($LASTEXITCODE -ne 0) {
  throw 'Failed to start simulation launch in background.'
}

$moveGroupReady = $false
for ($i = 0; $i -lt 90; $i++) {
  Invoke-ContainerBash -Command "pgrep -f '[m]ove_group' >/dev/null 2>&1"
  if ($LASTEXITCODE -eq 0) {
    $moveGroupReady = $true
    break
  }
  Start-Sleep -Seconds 1
}

if (-not $moveGroupReady) {
  throw 'move_group did not start in time. Check /workspace/project/.tmp/grasp_demo.launch.log in container.'
}

Write-Host 'Starting coordinator in background...'
$startCoordinatorCmd = @"
$containerPrefix
source /workspace/project/catkin_ws/devel/setup.bash
nohup env PYTHONUNBUFFERED=1 python3 -u /workspace/project/catkin_ws/src/three_arm_demo/scripts/three_arm_coordinator.py > /workspace/project/.tmp/grasp_demo.coord.log 2>&1 &
echo `$! > /workspace/project/.tmp/grasp_demo.coord.pid
"@
Invoke-ContainerBash -Command $startCoordinatorCmd
if ($LASTEXITCODE -ne 0) {
  throw 'Failed to start three_arm_coordinator.py in background.'
}

Start-Sleep -Seconds 4

Write-Host 'Publishing demo targets...'
$publishTargetsCmd = @"
$containerPrefix
source /workspace/project/catkin_ws/devel/setup.bash
python3 - <<'PY'
import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

rospy.init_node('auto_grasp_demo_targets', anonymous=True)
arm_bases = {
    'arm1': (0.30, 0.0, 0.0),
    'arm2': (-0.15, 0.2598076, 0.0),
    'arm3': (-0.15, -0.2598076, 0.0),
}
pair_targets = {
    '/target/P': ('arm1', 'arm2'),
    '/target/Y': ('arm2', 'arm3'),
    '/target/G': ('arm1', 'arm3'),
}
center = (0.0, 0.0, 0.01)
center_pull = 0.45
targets = {
    topic: (
        center[0] + ((arm_bases[a][0] + arm_bases[b][0]) / 2.0 - center[0]) * center_pull,
        center[1] + ((arm_bases[a][1] + arm_bases[b][1]) / 2.0 - center[1]) * center_pull,
        center[2],
    )
    for topic, (a, b) in pair_targets.items()
}
pubs = {topic: rospy.Publisher(topic, PointStamped, queue_size=10) for topic in targets}
marker_pubs = {
    topic: rospy.Publisher(topic.replace('/target/', '/target_marker/'), Marker, queue_size=10, latch=True)
    for topic in targets
}
marker_colors = {
    '/target/P': (0.6, 0.0, 1.0),
    '/target/Y': (1.0, 1.0, 0.0),
    '/target/G': (0.0, 1.0, 0.0),
}
marker_ids = {
    '/target/P': 1,
    '/target/Y': 2,
    '/target/G': 3,
}

rospy.sleep(1.0)
rate = rospy.Rate(5)
for _ in range(80):
    now = rospy.Time.now()
    for topic, (x, y, z) in targets.items():
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = now
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        pubs[topic].publish(msg)

        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = now
        marker.ns = 'grasp_demo'
        marker.id = marker_ids[topic]
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        r, g, b = marker_colors[topic]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker_pubs[topic].publish(marker)
    rate.sleep()

print('demo_targets_published')
PY
"@
Invoke-ContainerBash -Command $publishTargetsCmd
if ($LASTEXITCODE -ne 0) {
  throw 'Failed to publish demo targets.'
}

Write-Host ''
Write-Host 'One-click grasp demo started.'
Write-Host 'RViz should now display pick-and-place actions.'
Write-Host ''
Write-Host 'Useful commands:'
Write-Host '  docker exec -it yjg-noetic bash -lc "tail -f /workspace/project/.tmp/grasp_demo.coord.log"'
Write-Host '  docker exec -it yjg-noetic bash -lc "tail -f /workspace/project/.tmp/grasp_demo.launch.log"'
Write-Host '  docker exec yjg-noetic bash -lc "pkill -f [r]oslaunch; pkill -f [t]hree_arm_coordinator.py"'
