# 测试

> 返回 [README](../README.md)。一键运行：

```bash
bash scripts/run_selftest.sh     # 构建 + 全部测试
```

| 测试 | 内容 |
|---|---|
| `test_framecodec` | CRC 向量（`"123456789"→0x4B37`）、全帧型组/解往返、坏帧重同步、伪帧头、逐字节喂入、CON 载荷校验（88 断言） |
| `test_consm` | 虚拟时钟 8 场景：建链/握手耗尽/ping 丢失断链/优雅拆链/发送失败/板重上电重入/对端拆链/重连（44 断言） |
| `test_otaflasher` | 子进程输出按 `\r/\n` 切行、退出码、重复启动拒绝、进程组强杀、快速退出后复用与 Channel 延迟销毁（28 断言） |
| `log_bench` | 多线程日志吞吐基准 |
| `tools/selftest.py` | 端到端三进程：`mock_board`（pty 模拟下位机，含 CON 服务端/3s 看门狗/PI 电机模型/正逆运动学 ODOM）+ `chassis_host` + 脚本化 TCP 客户端，覆盖建链/自动使能/遥测/速度指令/ping/统计/断连停车/杀模拟器→串口恢复→重连/粘性失能/优雅退出与日志落盘（21 断言） |

`tools/mock_board.cc` 亦可单独用于人工联调：

```bash
./build/host/mock_board --pty --link /tmp/mock_tty     # 终端 1：打印 pty 路径
./build/host/chassis_host --serial /tmp/mock_tty       # 终端 2
python3 tools/client.py --port 9000                    # 终端 3：交互操控
```

USB1 真板 OTA 的交付验收还应覆盖：正常烧录后 CON 重建且 STATUS `en=0`、
擦写中忙态门控、缺清单/错 BIN 在进 BL 前拒绝、烧录口被占用、非回环连接
拒绝 `flash`/`flash_abort`、强制中止后再次完整烧录可恢复。测试前架空车轮或
确认电机无速度指令，结束后不发送 `en 1`。
