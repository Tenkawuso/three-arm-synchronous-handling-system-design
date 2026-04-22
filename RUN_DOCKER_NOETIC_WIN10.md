# Win10 + Ubuntu22.04 + Docker(Noetic) 仿真步骤

## 1) 安装 Docker Desktop

你刚才的自动安装失败是因为需要管理员确认。请手动安装一次 Docker Desktop（安装过程中同意 WSL2 集成）。

安装完成后，确认：

```powershell
docker --version
docker compose version
```

## 2) 一键运行

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_noetic_smoke.ps1
```

这会自动完成：

1. 启动 VcXsrv
2. 启动/重建 Docker 容器
3. 编译工作区
4. 跑一轮三臂仿真验证

## 3) 结束

```powershell
docker compose -f .\docker-compose.noetic.yml down
```
