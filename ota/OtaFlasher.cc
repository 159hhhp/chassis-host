#include "ota/OtaFlasher.h"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include "log/Logger.h"

extern char** environ;

namespace mhost {
namespace ota {

namespace {

/// 管道读端转非阻塞（epoll LT 要求；写端给子进程保持阻塞语义无妨）
void setNonBlocking(int fd) {
  int fl = ::fcntl(fd, F_GETFL, 0);
  ::fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

}  // namespace

OtaFlasher::OtaFlasher(muduo::net::EventLoop* loop) : loop_(loop) {}

OtaFlasher::~OtaFlasher() {
  if (pid_ > 0) {
    // 主进程退出路径：杀组 + 有限等待 + 兜底 KILL + 阻塞收尸
    ::kill(-pid_, SIGTERM);
    for (int i = 0; i < 20; ++i) {  // 最多等 1s
      if (::waitpid(pid_, nullptr, WNOHANG) == pid_) {
        pid_ = 0;
        break;
      }
      ::usleep(50 * 1000);
    }
    if (pid_ > 0) {
      ::kill(-pid_, SIGKILL);
      ::waitpid(pid_, nullptr, 0);
      pid_ = 0;
    }
  }
  if (channel_) {
    channel_->disableAll();
    channel_->remove();
    channel_.reset();
  }
  if (pipeFd_ >= 0) {
    ::close(pipeFd_);
    pipeFd_ = -1;
  }
}

bool OtaFlasher::start(const std::vector<std::string>& argv, LineCb onLine,
                       ExitCb onExit) {
  if (pid_ > 0) {
    LOG_WARN << "OtaFlasher: 已有子进程在运行，拒绝重复启动";
    return false;
  }
  if (argv.empty()) return false;

  int fds[2];
  if (::pipe(fds) != 0) {
    LOG_SYSERR << "OtaFlasher: pipe";
    return false;
  }
  setNonBlocking(fds[0]);

  std::vector<char*> cargv;
  cargv.reserve(argv.size() + 1);
  for (const auto& a : argv) cargv.push_back(const_cast<char*>(a.c_str()));
  cargv.push_back(nullptr);

  posix_spawn_file_actions_t fa;
  ::posix_spawn_file_actions_init(&fa);
  ::posix_spawn_file_actions_adddup2(&fa, fds[1], STDOUT_FILENO);
  ::posix_spawn_file_actions_adddup2(&fa, fds[1], STDERR_FILENO);
  ::posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  ::posix_spawn_file_actions_addclose(&fa, fds[0]);
  ::posix_spawn_file_actions_addclose(&fa, fds[1]);

  posix_spawnattr_t attr;
  ::posix_spawnattr_init(&attr);
  // 独立进程组：组信号一刀切带走 flash.py 与其子进程（CubeProgrammer 等）；
  // 信号掩码清空 + 默认处置，避免继承主进程的屏蔽位
  ::posix_spawnattr_setpgroup(&attr, 0);
  sigset_t empty;
  ::sigemptyset(&empty);
  ::posix_spawnattr_setsigmask(&attr, &empty);
  ::posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK);

  pid_t pid = -1;
  int rc = ::posix_spawnp(&pid, cargv[0], &fa, &attr, cargv.data(), environ);
  ::posix_spawnattr_destroy(&attr);
  ::posix_spawn_file_actions_destroy(&fa);
  ::close(fds[1]);  // 父进程必须关写端，否则子进程退出后读端不会 EOF
  if (rc != 0) {
    LOG_ERROR << "OtaFlasher: posix_spawnp(" << cargv[0] << ") 失败: " << ::strerror(rc);
    ::close(fds[0]);
    return false;
  }

  pid_ = pid;
  pipeFd_ = fds[0];
  lineBuf_.clear();
  lineCb_ = std::move(onLine);
  exitCb_ = std::move(onExit);
  exited_ = false;
  detachScheduled_ = false;

  channel_ = std::make_unique<muduo::net::Channel>(loop_, pipeFd_);
  channel_->setReadCallback([this](muduo::Timestamp) { onReadable(); });
  channel_->setCloseCallback([this] { scheduleDetachPipe(); });
  channel_->setErrorCallback([this] { scheduleDetachPipe(); });
  channel_->enableReading();

  LOG_INFO << "OtaFlasher: 子进程已启动 pid=" << pid_ << " exec=" << argv[0]
           << " argc=" << argv.size();
  return true;
}

void OtaFlasher::killGroup() {
  if (pid_ <= 0 || exited_) return;
  if (::kill(-pid_, SIGTERM) != 0 && errno == ESRCH) return;  // 已退出
  LOG_WARN << "OtaFlasher: SIGTERM -> 进程组 " << pid_;
  // 0.5s 未退升级 KILL（回调自查存活，无需外部撤销）
  loop_->runAfter(0.5, [this] {
    if (pid_ > 0 && !exited_ && ::kill(-pid_, 0) == 0) {
      LOG_WARN << "OtaFlasher: TERM 未生效，SIGKILL -> 进程组 " << pid_;
      ::kill(-pid_, SIGKILL);
    }
  });
}

void OtaFlasher::onReadable() {
  if (pipeFd_ < 0) return;
  char buf[4096];
  for (;;) {
    ssize_t n = ::read(pipeFd_, buf, sizeof buf);
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        char c = buf[i];
        if (c == '\n' || c == '\r') {
          if (!lineBuf_.empty()) {
            if (lineCb_) lineCb_(lineBuf_);
            lineBuf_.clear();
          }
        } else {
          lineBuf_ += c;
          if (lineBuf_.size() > 8192) {  // 单行超限：疑为二进制垃圾，丢弃防撑爆
            LOG_WARN << "OtaFlasher: 子进程单行超 8KB，截断";
            lineBuf_.clear();
          }
        }
      }
      continue;
    }
    if (n == 0) {
      scheduleDetachPipe();  // 子进程关闭了标准输出（通常即将/已经退出）
      return;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) return;
    if (errno == EINTR) continue;
    LOG_SYSERR << "OtaFlasher: read";
    scheduleDetachPipe();
    return;
  }
}

void OtaFlasher::scheduleDetachPipe() {
  if (pipeFd_ < 0 || detachScheduled_) return;
  detachScheduled_ = true;
  if (!lineBuf_.empty()) {
    if (lineCb_) lineCb_(lineBuf_);
    lineBuf_.clear();
  }
  if (channel_) channel_->disableAll();
  // Channel 正在执行 handleEvent 时销毁自身会形成悬空 this；排到本轮事件
  // 回调之后再 remove/reset，保证 Muduo 的事件栈已经退干净。
  loop_->queueInLoop([this] { detachPipe(); });
}

void OtaFlasher::detachPipe() {
  if (pipeFd_ < 0) return;
  if (channel_) {
    channel_->disableAll();
    channel_->remove();
    channel_.reset();
  }
  ::close(pipeFd_);
  pipeFd_ = -1;
  detachScheduled_ = false;
  pollReap(0);
}

void OtaFlasher::pollReap(int tries) {
  if (pid_ <= 0 || exited_) return;
  int status = 0;
  pid_t r = ::waitpid(pid_, &status, WNOHANG);
  if (r == pid_) {
    finish(status);
    return;
  }
  if (r == 0 || errno == EINTR) {
    // EOF 先于退出（半途关 stdout）：50ms 后再查；10s 仍活着则阻塞等待兜底
    if (tries < 200) {
      loop_->runAfter(0.05, [this, next = tries + 1] { pollReap(next); });
    } else {
      LOG_WARN << "OtaFlasher: 回收超时，阻塞 waitpid 兜底";
      ::kill(-pid_, SIGKILL);
      if (::waitpid(pid_, &status, 0) == pid_) {
        finish(status);
      } else {
        finish(-1);
      }
    }
    return;
  }
  LOG_SYSERR << "OtaFlasher: waitpid";
  finish(-1);
}

void OtaFlasher::finish(int exitStatus) {
  exited_ = true;
  int code = exitStatus;
  pid_ = 0;
  bool ok = (WIFEXITED(exitStatus) && WEXITSTATUS(exitStatus) == 0);
  LOG_INFO << "OtaFlasher: 子进程退出 ok=" << ok << " status=" << exitStatus;
  if (exitCb_) {
    ExitCb cb = std::move(exitCb_);
    exitCb_ = nullptr;
    cb(ok, code);
  }
}

}  // namespace ota
}  // namespace mhost
