# noetix-m1-yaocao

Noetix M1 双臂控制器。仓库只保留正式的 Modbus 控制核心、UDP 协议、
运行配置、systemd 服务文件和必要的部署诊断工具。

## 构建

```bash
cmake -S . -B build-servo-test
cmake --build build-servo-test -j
```

## 运行

生产环境由 `systemd/startup.service` 调用 `startup.sh`。控制端口为 UDP
8890，主臂状态流端口为 UDP 8888。

`tools/send_dual_arm_udp.py` 用于协议健康检查，
`tools/restart_robot_udp_service.sh` 用于远程重启并验证服务。
