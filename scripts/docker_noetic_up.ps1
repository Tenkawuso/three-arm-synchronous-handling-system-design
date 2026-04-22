$env:PATH = "C:\Program Files\Docker\Docker\resources\bin;" + $env:PATH
$env:DISPLAY = "host.docker.internal:0.0"
docker compose -f docker-compose.noetic.yml up -d --build
