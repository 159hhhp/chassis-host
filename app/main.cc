/// @file main.cc
/// @brief chassis_host 入口：解析 CLI → 初始化日志（双缓冲异步接管）→
///        运行单 Reactor 事件循环（TCP + 串口同 loop）。

#include <getopt.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <muduo/net/EventLoop.h>

#include "ChassisHost.h"
#include "log/AsyncLogging.h"
#include "log/Logger.h"

namespace {

std::atomic<bool> g_quit{false};  // 信号处理只置位，EventLoop 周期检查后优雅退出
mhost::AsyncLogging* g_asyncLog = nullptr;

/// Logger 全局输出钩子 → 双缓冲异步日志（函数指针接口，故用全局指针）
void asyncOutput(const char* msg, int len) {
  if (g_asyncLog != nullptr) {
    g_asyncLog->append(msg, len);
  }
}

void onSignal(int) { g_quit.store(true); }

void usage(const char* prog) {
  ::fprintf(
      stderr,
      "用法: %s [选项]\n"
      "  --serial <dev>       串口设备（默认 /dev/ttyUSB0，树莓派 USB3 口 CH9102）\n"
      "  --baud <n>           波特率（默认 115200，8N1，与固件 USART3 一致）\n"
      "  --listen <ip:port>   TCP 监听（默认 0.0.0.0:9000）\n"
      "  --log-dir <dir>      日志目录（默认 log；传 \"\" 用 stdout 同步输出）\n"
      "  --log-level <lv>     trace/debug/info/warn/error/fatal（默认 info）\n"
      "  --repeat-hz <hz>     速度指令重发频率（默认 10，喂板侧 500ms 超时）\n"
      "  --odom-hz <hz>       odom JSON 推送频率（默认 10）\n"
      "  --slow-hz <hz>       enc/imu/telem/status JSON 推送频率（默认 1）\n"
      "  --con-retry-ms <ms>  CON 断链重连节流（默认 2000）\n"
      "  --rx-timeout-ms <ms> 板帧静默告警阈值（默认 2000）\n"
      "  --en-resend-ms <ms>  STATUS en=0 补发 CMD_EN 限频（默认 1000）\n"
      "  -h, --help           本说明\n",
      prog);
}

}  // namespace

int main(int argc, char* argv[]) {
  mhost::app::HostOptions opts;
  std::string logDir = "log";
  std::string logLevel = "info";
  double repeatHz = opts.repeatHz, odomHz = opts.odomHz, slowHz = opts.slowHz;

  static const struct option longopts[] = {
      {"serial", required_argument, nullptr, 's'},
      {"baud", required_argument, nullptr, 'b'},
      {"listen", required_argument, nullptr, 'l'},
      {"log-dir", required_argument, nullptr, 'd'},
      {"log-level", required_argument, nullptr, 'v'},
      {"repeat-hz", required_argument, nullptr, 'r'},
      {"odom-hz", required_argument, nullptr, 'o'},
      {"slow-hz", required_argument, nullptr, 'w'},
      {"con-retry-ms", required_argument, nullptr, 'c'},
      {"rx-timeout-ms", required_argument, nullptr, 'x'},
      {"en-resend-ms", required_argument, nullptr, 'e'},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, 0, nullptr, 0},
  };
  int ch;
  while ((ch = ::getopt_long(argc, argv, "s:b:l:d:v:r:o:w:c:x:e:h", longopts, nullptr)) != -1) {
    switch (ch) {
      case 's': opts.serialDev = optarg; break;
      case 'b': opts.serialBaud = ::atoi(optarg); break;
      case 'l': opts.listenAddr = optarg; break;
      case 'd': logDir = optarg; break;
      case 'v': logLevel = optarg; break;
      case 'r': repeatHz = ::atof(optarg); break;
      case 'o': odomHz = ::atof(optarg); break;
      case 'w': slowHz = ::atof(optarg); break;
      case 'c': opts.conRetryMs = static_cast<uint64_t>(::atoll(optarg)); break;
      case 'x': opts.rxTimeoutMs = static_cast<uint64_t>(::atoll(optarg)); break;
      case 'e': opts.enResendMs = static_cast<uint64_t>(::atoll(optarg)); break;
      case 'h': usage(argv[0]); return 0;
      default: usage(argv[0]); return 1;
    }
  }
  if (repeatHz <= 0.0 || odomHz <= 0.0 || slowHz <= 0.0) {
    ::fprintf(stderr, "repeat-hz/odom-hz/slow-hz 必须为正数\n");
    return 1;
  }
  opts.repeatHz = repeatHz;
  opts.odomHz = odomHz;
  opts.slowHz = slowHz;

  // ---- 日志初始化 ----
  mhost::Logger::setLogLevel(mhost::Logger::INFO);
  mhost::Logger::LogLevel lv = mhost::Logger::levelFromString(logLevel.c_str());
  if (lv == mhost::Logger::NUM_LOG_LEVELS) {
    ::fprintf(stderr, "非法日志级别: %s\n", logLevel.c_str());
    return 1;
  }
  mhost::Logger::setLogLevel(lv);

  std::unique_ptr<mhost::AsyncLogging> asyncLog;
  if (!logDir.empty()) {
    ::mkdir(logDir.c_str(), 0755);  // 已存在则忽略
    asyncLog = std::make_unique<mhost::AsyncLogging>(logDir + "/chassis_host",
                                                     32 * 1024 * 1024, 3);
    asyncLog->start();
    g_asyncLog = asyncLog.get();
    mhost::Logger::setOutput(&asyncOutput);
  }
  // logDir 为空时保持默认 stdout 同步输出（调试用）

  // ---- 信号 ----
  ::signal(SIGPIPE, SIG_IGN);  // 对端 RST 时走 write 错误路径而非杀进程
  ::signal(SIGINT, onSignal);
  ::signal(SIGTERM, onSignal);

  // ---- 运行 ----
  muduo::net::EventLoop loop;
  mhost::app::ChassisHost host(&loop, opts);
  host.start();
  loop.runEvery(0.2, [&loop] {
    if (g_quit.load()) loop.quit();
  });

  LOG_INFO << "chassis_host 启动";
  loop.loop();
  LOG_INFO << "chassis_host 退出";

  if (asyncLog) {
    asyncLog->stop();
  }
  return 0;
}
