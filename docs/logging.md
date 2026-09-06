# 双缓冲异步日志库

> 返回 [README](../README.md)。`log/` 是一个独立于 muduo、可单独复用的
> C++17 日志库（设计学习自陈硕 muduo 的 AsyncLogging，代码为本项目自行
> 实现），四层结构：

| 模块 | 职责 |
|---|---|
| `Logger` | 前端：TRACE~FATAL 六级流式宏（低于当前级别零开销短路），行格式 `YYYYMMDD HH:MM:SS.uuuuuu tid LEVEL 内容 - 文件:行`；秒级时间字符串线程内缓存 |
| `LogStream` | 栈上 4KB FixedBuffer + `operator<<`；整数倒序查表转换（不走 snprintf）；溢出安全截断 |
| `AsyncLogging` | **双缓冲核心**：`currentBuffer_`/`nextBuffer_` 两个 4MB 前台缓冲 + 待写队列。append 热路径 = 一次加锁 + 内存拷贝（缓冲够则返回，不唤醒不分配）；写满 → 移入队列、nextBuffer 补位；极端积压才临时 new。后台线程至多等 3s 或被唤醒，整批写入 LogFile；待写超 25 块丢弃多余并告警；写完的缓冲回收复用，稳态零动态分配 |
| `LogFile`/`FileUtil` | 64KB 用户态缓冲 + `fwrite_unlocked`；按大小/跨天滚动，周期 flush；文件名 `basename.YYYYmmdd-HHMMSS.host.pid.log` |

`main()` 用 `Logger::setOutput()` 把前端输出接到 `AsyncLogging::append`，
业务线程因此**永不阻塞在磁盘 I/O**。`--log-dir ""` 可退化为 stdout 同步
输出。实测吞吐（WSL2，4 线程，前端 append 速率）：

```
log_bench: 4 threads x 50000 lines = 200000 lines in 0.083s -> 2416442 lines/s
```
