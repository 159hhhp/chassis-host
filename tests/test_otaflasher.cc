/// @file test_otaflasher.cc
/// @brief 烧录子进程单测：行切分/退出码/进程组强杀。
///
/// 子进程用系统工具（printf/sleep）代替 flash.py，行为等价：
/// 按行输出、指定码退出、TERM/KILL 可终止。断言经事件回调在
/// EventLoop 线程内收集，主线程在全部回调到齐后退出事件循环。

#include <string>
#include <vector>

#include "ota/OtaFlasher.h"
#include "test_util.h"

using mhost::ota::OtaFlasher;

namespace {

struct Collector {
  std::vector<std::string> lines;
  bool exited = false;
  bool ok = false;
  int status = -1;

  OtaFlasher::LineCb lineFn() {
    return [this](const std::string& l) { lines.push_back(l); };
  }
  OtaFlasher::ExitCb exitFn(muduo::net::EventLoop* loop) {
    return [this, loop](bool o, int s) {
      exited = true;
      ok = o;
      status = s;
      loop->quit();
    };
  }
};

void testLineSplitAndExitCode() {
  muduo::net::EventLoop loop;
  OtaFlasher f(&loop);
  Collector c;
  // 每个 \r 与 \n 都独立终止一行（烧录工具用 \r 刷新进度条，必须逐条上抛）
  CHECK(f.start({"printf", "STAGE: check\nhalf\rline\r\n"}, c.lineFn(), c.exitFn(&loop)));
  loop.runAfter(2.0, [&loop] { loop.quit(); });  // 看门狗：异常时防挂死
  loop.loop();
  CHECK(c.exited);
  CHECK(c.ok);
  CHECK_EQ(c.lines.size(), 3u);
  CHECK(c.lines[0] == "STAGE: check");
  CHECK(c.lines[1] == "half");
  CHECK(c.lines[2] == "line");
}

void testNonZeroExit() {
  muduo::net::EventLoop loop;
  OtaFlasher f(&loop);
  Collector c;
  CHECK(f.start({"sh", "-c", "echo FAIL: 5 test; exit 5"}, c.lineFn(), c.exitFn(&loop)));
  loop.runAfter(2.0, [&loop] { loop.quit(); });
  loop.loop();
  CHECK(c.exited);
  CHECK(!c.ok);
  CHECK_EQ(c.lines.size(), 1u);
  CHECK(c.lines[0] == "FAIL: 5 test");
}

void testKillGroup() {
  muduo::net::EventLoop loop;
  OtaFlasher f(&loop);
  Collector c;
  // sh 子 shell 里再挂一个 sleep 孙进程：杀的是进程组，孙进程一并带走
  CHECK(f.start({"sh", "-c", "sleep 30 & echo spawned; wait"}, c.lineFn(), c.exitFn(&loop)));
  loop.runAfter(0.3, [&f] { f.killGroup(); });
  loop.runAfter(3.0, [&loop] { loop.quit(); });
  loop.loop();
  CHECK(c.exited);
  CHECK(!c.ok);
  CHECK_EQ(c.lines.size(), 1u);  // "spawned" 已收到，随后被杀
}

void testStartTwiceRejected() {
  muduo::net::EventLoop loop;
  OtaFlasher f(&loop);
  Collector c;
  CHECK(f.start({"sleep", "5"}, c.lineFn(), c.exitFn(&loop)));
  CHECK(!f.start({"echo", "dup"}, c.lineFn(), c.exitFn(&loop)));
  f.killGroup();
  loop.runAfter(1.5, [&loop] { loop.quit(); });
  loop.loop();
  CHECK(c.exited);  // 被杀子进程也触发了退出回调
  CHECK(!c.ok);
}

void testReusableAfterFastExit() {
  muduo::net::EventLoop loop;
  OtaFlasher f(&loop);
  std::vector<std::string> lines;
  int exits = 0;
  OtaFlasher::LineCb onLine = [&lines](const std::string& line) { lines.push_back(line); };
  OtaFlasher::ExitCb onSecondExit = [&loop, &exits](bool ok, int) {
    CHECK(ok);
    ++exits;
    loop.quit();
  };
  OtaFlasher::ExitCb onFirstExit = [&f, &exits, onLine, onSecondExit](bool ok, int) {
    CHECK(ok);
    ++exits;
    CHECK(f.start({"printf", "second\\n"}, onLine, onSecondExit));
  };
  CHECK(f.start({"printf", "first"}, onLine, onFirstExit));  // EOF 也须上抛末尾半行
  loop.runAfter(2.0, [&loop] { loop.quit(); });
  loop.loop();
  CHECK_EQ(exits, 2);
  CHECK_EQ(lines.size(), 2u);
  CHECK(lines[0] == "first");
  CHECK(lines[1] == "second");
}

}  // namespace

int main() {
  testLineSplitAndExitCode();
  testNonZeroExit();
  testKillGroup();
  testStartTwiceRejected();
  testReusableAfterFastExit();
  return testSummary("test_otaflasher");
}
