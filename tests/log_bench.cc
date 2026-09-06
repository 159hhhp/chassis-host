/// @file log_bench.cc
/// @brief 双缓冲异步日志吞吐基准：N 线程并发打日志，统计总吞吐与丢弃情况
///
/// 用法: log_bench [线程数=4] [每线程条数=200000]
/// 输出写入 /tmp/mhost_log_bench.*.log（滚动阈值 1GB，基准期间不触发滚动）。

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "log/AsyncLogging.h"
#include "log/Logger.h"

namespace {

mhost::AsyncLogging* g_log = nullptr;  // Logger 输出钩子（函数指针接口，需无捕获）

/// 简易栅栏：等所有工作线程就位后同时开跑
class StartGate {
 public:
  void wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return ready_; });
  }
  void open() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = true;
    }
    cv_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool ready_ = false;
};

}  // namespace

int main(int argc, char* argv[]) {
  int numThreads = argc > 1 ? ::atoi(argv[1]) : 4;
  long long linesPerThread = argc > 2 ? ::atoll(argv[2]) : 200000;
  if (numThreads <= 0) numThreads = 4;
  if (linesPerThread <= 0) linesPerThread = 200000;

  mhost::AsyncLogging log("/tmp/mhost_log_bench", 1ULL << 30);
  log.start();
  mhost::Logger::setLogLevel(mhost::Logger::INFO);
  g_log = &log;
  mhost::Logger::setOutput([](const char* msg, int len) {
    if (g_log != nullptr) g_log->append(msg, len);
  });

  StartGate gate;
  std::atomic<long long> total{0};
  std::vector<std::thread> threads;
  threads.reserve(static_cast<size_t>(numThreads));
  for (int t = 0; t < numThreads; ++t) {
    threads.emplace_back([&, t] {
      gate.wait();
      for (long long i = 0; i < linesPerThread; ++i) {
        // 典型业务行：时间戳 + tid + 级别 + 若干数值字段，约 90 字节
        LOG_INFO << "worker " << t << " iter " << i << " vx=" << (i % 400) * 0.001
                 << " rpm=" << i % 350 << " duty=" << (i % 1000) * 0.1
                 << " con=READY frames=" << i;
      }
      total += linesPerThread;
    });
  }

  auto t0 = std::chrono::steady_clock::now();
  gate.open();
  for (auto& th : threads) th.join();
  auto t1 = std::chrono::steady_clock::now();
  double secs = std::chrono::duration<double>(t1 - t0).count();

  std::printf(
      "log_bench: %d threads x %lld lines = %lld lines in %.3fs -> %.0f lines/s "
      "(%.1f MB/s, 按 90B/行)\n",
      numThreads, linesPerThread, total.load(), secs,
      static_cast<double>(total.load()) / secs,
      static_cast<double>(total.load()) * 90.0 / secs / (1024.0 * 1024.0));

  // 先把输出切到 stderr 再停后端，避免 stop 后仍向已停止的 AsyncLogging 写入
  mhost::Logger::setOutput([](const char* msg, int len) {
    ::fwrite(msg, 1, static_cast<size_t>(len), stderr);
  });
  log.stop();
  return 0;
}
