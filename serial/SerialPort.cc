#include "SerialPort.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "log/Logger.h"

namespace mhost {
namespace serial {

namespace {

/// 波特率数值 → termios 常量（覆盖工程全部使用档位），未知返回 false
bool toSpeedConstant(int baud, speed_t* out) {
  switch (baud) {
    case 9600: *out = B9600; return true;
    case 19200: *out = B19200; return true;
    case 38400: *out = B38400; return true;
    case 57600: *out = B57600; return true;
    case 115200: *out = B115200; return true;
    case 230400: *out = B230400; return true;
    case 460800: *out = B460800; return true;
    case 921600: *out = B921600; return true;
    default: return false;
  }
}

/// 原始 8N1 配置：无流控、无回环、VMIN=0/VTIME=0（配合非阻塞 fd）
bool configureTty(int fd, int baud) {
  speed_t speed;
  if (!toSpeedConstant(baud, &speed)) {
    LOG_ERROR << "SerialPort: 不支持的波特率 " << baud;
    return false;
  }
  struct termios tty;
  if (::tcgetattr(fd, &tty) != 0) {
    LOG_SYSERR << "SerialPort: tcgetattr";
    return false;
  }
  ::cfmakeraw(&tty);  // 无加工：关回显/规范化/字符映射
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);   // 1 停止位
  tty.c_cflag &= ~static_cast<tcflag_t>(PARENB);   // 无校验（8N1，与固件 USART3 一致）
  tty.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);  // 无硬件流控（CH9102 未接 DTR/RTS）
  // VMIN 必须为 1：O_NONBLOCK + VMIN=0 时 tty 无数据 read() 返回 0（而非
  // EAGAIN），与 EOF 无法区分（pty 场景必现）；VMIN=1 下返回 -1/EAGAIN。
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;
  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
    LOG_SYSERR << "SerialPort: cfset*speed";
    return false;
  }
  if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
    LOG_SYSERR << "SerialPort: tcsetattr";
    return false;
  }
  ::tcflush(fd, TCIOFLUSH);
  return true;
}

}  // namespace

SerialPort::SerialPort(muduo::net::EventLoop* loop) : loop_(loop) {}

SerialPort::~SerialPort() {
  // 要求在 loop 结束后析构（ChassisHost 生命周期覆盖 loop.loop()）
  if (fd_ >= 0) {
    if (channel_) {
      channel_->disableAll();
      channel_->remove();
    }
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::openDevice(const std::string& dev, int baud) {
  if (fd_ >= 0) return true;  // 幂等：重开定时器周期调用时直接返回

  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    // 设备暂不存在（未插线 / usbipd 未 attach）不算错误，重开定时器会再试
    LOG_DEBUG << "SerialPort: open " << dev << " 失败: " << ::strerror(errno);
    return false;
  }
  if (!configureTty(fd, baud)) {
    ::close(fd);
    return false;
  }

  fd_ = fd;
  dev_ = dev;
  outputBuffer_.retrieveAll();
  // Channel 创建/挂载必须回到 loop 线程（openDevice 可能来自任意线程）
  loop_->runInLoop([this] { attachInLoop(); });
  return true;
}

void SerialPort::attachInLoop() {
  if (fd_ < 0) return;  // 排队期间已被关闭

  channel_ = std::make_unique<muduo::net::Channel>(loop_, fd_);
  channel_->setReadCallback([this](muduo::Timestamp t) { handleRead(t); });
  channel_->setWriteCallback([this] { handleWrite(); });
  channel_->setCloseCallback([this] { closeInLoop("hup"); });
  channel_->setErrorCallback([this] { handleError(); });
  channel_->doNotLogHup();  // tty 掉线是预期内的可自愈事件，不打 muduo WARN
  channel_->enableReading();

  LOG_INFO << "SerialPort: " << dev_ << " 已挂入 epoll（fd=" << fd_ << "）";
  if (stateCb_) stateCb_(true, "opened");
}

void SerialPort::closeDevice() {
  loop_->runInLoop([this] { closeInLoop("closed"); });
}

void SerialPort::closeInLoop(const std::string& reason) {
  if (fd_ < 0) return;
  if (channel_) {
    channel_->disableAll();
    channel_->remove();  // 从 epoll 与 Poller 的 channels_ 表中移除
    channel_.reset();
  }
  ::close(fd_);
  fd_ = -1;
  LOG_WARN << "SerialPort: " << dev_ << " 已断开（" << reason << "）";
  if (stateCb_) stateCb_(false, reason);
}

void SerialPort::handleRead(muduo::Timestamp /*now*/) {
  if (fd_ < 0) return;
  char buf[4096];
  // LT 触发：一次唤醒内循环读到 EAGAIN 排空，避免残留数据反复唤醒
  for (;;) {
    ssize_t n = ::read(fd_, buf, sizeof buf);
    if (n > 0) {
      if (dataCb_) dataCb_(buf, static_cast<size_t>(n));
      continue;
    }
    if (n == 0) {
      // VMIN=1 下正常不会返回 0；到达此处说明对端确已关闭（pty 主端退出等）
      closeInLoop("eof");  // usbipd detach / 设备拔出时可能出现
      return;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return;  // 已排空
    }
    if (errno == EINTR) {
      continue;
    }
    LOG_SYSERR << "SerialPort: read";
    closeInLoop("read-error");
    return;
  }
}

void SerialPort::handleWrite() {
  if (fd_ < 0) return;
  if (!channel_->isWriting()) {
    LOG_TRACE << "SerialPort: 下行已写完，POLLOUT 已撤（无害）";
    return;
  }
  ssize_t n = ::write(fd_, outputBuffer_.peek(), outputBuffer_.readableBytes());
  if (n >= 0) {
    outputBuffer_.retrieve(static_cast<size_t>(n));
    if (outputBuffer_.readableBytes() == 0) {
      channel_->disableWriting();  // 全部写出，撤 POLLOUT
    }
  } else {
    LOG_SYSERR << "SerialPort: write";
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      closeInLoop("write-error");
    }
  }
}

void SerialPort::handleError() {
  // tty 设备消失通常报 POLLERR（伴随/不伴随 POLLHUP）
  LOG_SYSERR << "SerialPort: 设备错误";
  closeInLoop("err");
}

void SerialPort::send(const void* data, size_t len) {
  if (fd_ < 0) return;
  loop_->runInLoop([this, data, len] { sendInLoop(static_cast<const char*>(data), len); });
}

void SerialPort::sendInLoop(const char* data, size_t len) {
  if (fd_ < 0) return;
  size_t remaining = len;
  ssize_t nwrote = 0;

  // 缓冲为空且未挂 POLLOUT：先尝试直写（快速路径）
  if (!channel_->isWriting() && outputBuffer_.readableBytes() == 0) {
    nwrote = ::write(fd_, data, len);
    if (nwrote >= 0) {
      remaining = len - static_cast<size_t>(nwrote);
    } else {
      nwrote = 0;
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        LOG_SYSERR << "SerialPort: 直写失败";
        if (errno == EPIPE || errno == ECONNRESET || errno == EBADF || errno == ENODEV ||
            errno == EIO) {
          closeInLoop("write-fault");
          return;
        }
      }
    }
  }

  if (remaining > 0) {
    outputBuffer_.append(data + (len - remaining), remaining);
    if (!channel_->isWriting()) {
      channel_->enableWriting();  // 挂 POLLOUT，内核缓冲腾出后续传
    }
    if (outputBuffer_.readableBytes() > kTxOverflowLimit) {
      LOG_ERROR << "SerialPort: 发送缓冲溢出 " << outputBuffer_.readableBytes()
                << "B，判定链路半死，主动断开";
      closeInLoop("tx-overflow");
    }
  }
}

}  // namespace serial
}  // namespace mhost
