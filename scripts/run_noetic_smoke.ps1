$ErrorActionPreference = 'Stop'

$dockerBin = 'C:\Program Files\Docker\Docker\resources\bin'
if (Test-Path $dockerBin) {
  $env:PATH = "$dockerBin;" + $env:PATH
}

$projectRoot = Split-Path $PSScriptRoot -Parent
$composeFile = Join-Path $projectRoot 'docker-compose.noetic.yml'
$vcxsrv = 'C:\Program Files\VcXsrv\vcxsrv.exe'

if (-not (Get-Process -Name vcxsrv -ErrorAction SilentlyContinue)) {
  if (Test-Path $vcxsrv) {
    Start-Process -FilePath $vcxsrv -ArgumentList ':0 -multiwindow -clipboard -ac -nowgl' -WindowStyle Minimized | Out-Null
    Start-Sleep -Seconds 2
  }
}

$env:DISPLAY = 'host.docker.internal:0.0'

docker compose -f $composeFile up -d --build

docker exec yjg-noetic bash -lc 'chmod +x /workspace/project/scripts/build_ws_in_container.sh /workspace/project/scripts/smoke_test_noetic.sh && /workspace/project/scripts/build_ws_in_container.sh && /workspace/project/scripts/smoke_test_noetic.sh'
