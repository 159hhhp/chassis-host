# TCP 文本协议

> 返回 [README](../README.md)。指令门控、CON 链路保活与失效保护机制见
> [protocol-serial.md](protocol-serial.md)；MATLAB 客户端示例见 `samples/`。

一行一命令（`\n` 结尾，容忍 `\r\n`，`#` 开头为注释行）。应答行以
`ok` / `err` 开头；遥测为 JSON 行。示例：

```
$ nc 192.168.1.10 9000
v 0.2 0 0.5
ok twist vx=0.200 vy=0.000 wz=0.500（已下发，10Hz 重发中）
watch on
{"t":"odom","vx":0.195,"vy":0.000,"wz":0.481,"x":0.112,"y":0.000,"yaw":0.049}
{"t":"status","uptime":37,"imu_ok":1,"en":1,"mode":1,"con":1}
ping
pong rtt=3.0ms
stop
ok stop
```

| 命令 | 说明 | 下发帧 |
|---|---|---|
| `v <vx> [vy] [wz]` | 车体速度 m/s、rad/s（登记后由宿主 10Hz 重发） | CMD_TWIST(0x86) |
| `rpm <r1> [r2] [r3]` | 三电机输出轴目标转速 rpm | CMD_VEL(0x81) |
| `stop` | 刹车 + 退出闭环，并停止重发 | CMD_STOP(0x82) |
| `en <0\|1>` | 电机使能。**1**=使能+自动建链；**0**=失能+拆链（粘性，重连后仍失能） | CMD_EN(0x83) |
| `pid <kp> <ki> <kd>` | 三电机共用 PID | CMD_PID(0x84) |
| `sign <s1> <s2> <s3>` | 编码器方向校正 ±1 | CMD_SIGN(0x85) |
| `ping` | 测板端 RTT（下位机原样回显） | PING(0x8F) |
| `watch [on\|off]` | 订阅/退订 JSON 遥测流 | — |
| `flash <name.hex>` | 从固件白名单目录远程烧录；仅回环连接可执行 | USB1 ROM Bootloader |
| `flash_status` | 查询升级状态、阶段与子进程存活状态 | — |
| `flash_abort [force]` | 烧前可直接取消；擦写中须显式 `force` | — |
| `stat` `con` `help` `quit` | 统计 / 链路状态 / 帮助 / 断开 | — |

遥测 JSON（`watch` 订阅者）：`odom` 默认 10Hz（`--odom-hz`）；
`enc` / `imu` / `telem` / `status` 默认 1Hz（`--slow-hz`）。
`enc` 行含 `ts` 字段（板侧编码器采样时刻，1ms 时基，新固件 22B 帧）。
`con`（链路状态）与 `serial`（设备 up/down）事件推送给**所有**连接。

多连接：均可 `watch` 观测；速度指令来源（最后一条 `v/rpm` 的连接）断开时
宿主自动 `CMD_STOP` 停车。多控制端并发下发时后到者覆盖，需自行约定。

## USB1 远程固件升级

### 部署

车上同时保留两根 USB 线：USB3 是 `--serial` 协议口，USB1 是
`--flash-dev` 烧录口。先按板载 CH9102 的序列号解析设备；树莓派以 root
运行同一脚本时还会安装 udev 固定名规则：

```bash
scripts/setup-c30d-ports.sh

# 真机服务用户需有串口权限；禁用会主动探测串口控制线的 ModemManager
sudo usermod -aG dialout <服务用户>
sudo systemctl disable --now ModemManager

./build/host/chassis_host \
    --serial /dev/c30d-proto \
    --flash-dev /dev/c30d-flash \
    --flash-script /opt/c30d/host_test/flash.py \
    --firmware-dir /opt/c30d/firmware \
    --flash-tool auto
```

固件目录只接收安全文件名，并要求同名发布三件套：`.hex`、`.bin`、
`.manifest.json`。清单由 C30D_Chassis 的 `host_test/make_manifest.py`
生成；`flash.py` 复核板型、F407 ID、版本、摘要、大小、主 Flash 地址范围，
使用 `stm32flash` 时还会证明 BIN 与 HEX 数据一致。

默认只监听 `127.0.0.1:9000`。即使显式开放 `0.0.0.0:9000`，`flash` 和
`flash_abort` 仍拒绝非回环连接；推荐 SSH 登录车端，再运行
`nc 127.0.0.1 9000`。当前文本协议没有用户认证，不应把控制端口直接暴露到
不可信网络。

### 状态与安全语义

1. `quiescing`：立即清空速度重发缓存，下发 STOP 和 EN0，并等待板侧
   STATUS 回报 `en=0`；应用已损坏、无法回报时，3 秒后允许复位进 BL 走恢复；
2. `flashing`：重置 CON 状态，暂停无意义的重连和静默告警；烧录工具在独立
   进程组中运行，输出按行回显，运动和参数命令统一返回 `busy`；
3. `recovering`：擦写校验成功、应用复位后重建 CON；
4. `none`：任务结束。速度缓存不恢复，板端保持 `en=0`，必须人工执行新
   `en 1` 和新速度指令。

发起连接断开不终止已经开始的烧录。`flash_abort` 在擦写开始前可直接取消；
擦写中普通取消会被拒绝，只有 `flash_abort force` 杀整个进程组，此时应用
可能不完整，但系统 ROM Bootloader 仍可通过下一次完整 `flash` 恢复。错误
镜像、缺清单、端口占用和工具缺失都返回明确失败信息，失败后电机保持失能。
