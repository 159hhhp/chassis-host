#pragma once
/// @file SerialPort.h
/// @brief 串口设备：把 tty fd 用 muduo Channel 挂进 EventLoop 的 epoll，
///        与 TcpServer 共享同一个 Reactor（无桥接线程）。
///
/// 可行性依据：Channel/Poller 与 socket 解耦，EPollPoller 是纯
/// epoll_wait/epoll_ctl 转发；裸 fd 挂 Channel 的范本见 ref/muduo 的
/// examples/cdns/Resolver.cc 与 muduo/net/tests/Channel_test.cc，
/// 实现要点与坑见本工程 docs/architecture.md。
///
/// 线程约束：所有 Channel 操作必须在 loop 线程执行，跨线程接口
/// （send/openDevice/closeDevice）内部经 loop->runInLoop 转发。

#include <functional>
#include <memory>
#include <string>

#include <muduo/net/Buffer.h>
#include <muduo/net/Channel.h>
#include <muduo/net/EventLoop.h>

namespace mhost {
namespace serial {

class SerialPort {
 public:
  /// tty 有数据可读（已尽量排空一次唤醒内的全部字节）
  using DataCallback = std::function<void(const char* data, size_t len)>;
  /// 设备 up/down 事件（down 携带原因，如 "hup" / "err: No such device"）
  using StateCallback = std::function<void(bool up, const std::string& reason)>;

  explicit SerialPort(muduo::net::EventLoop* loop);
  ~SerialPort();

  /// 打开并配置 tty，成功后把 Channel 挂入 loop（幂等：已打开直接返回 true）
  bool openDevice(const std::string& dev, int baud);
  /// 拆除 Channel 并关闭 fd（可在任意时刻调用，幂等）
  void closeDevice();

  bool isOpen() const { return fd_ >= 0; }
  const std::string& device() const { return dev_; }

  /// 发送（线程安全；内部转 loop 线程执行）
  void send(const void* data, size_t len);

  void setDataCallback(DataCallback cb) { dataCb_ = std::move(cb); }
  void setStateCallback(StateCallback cb) { stateCb_ = std::move(cb); }

  /// 发送缓冲硬上限：tty 长时间写不出（设备半死）时主动断开，防止内存堆积
  static const size_t kTxOverflowLimit = 1 * 1024 * 1024;

 private:
  void attachInLoop();                  ///< loop 线程内创建 Channel 并挂 epoll
  void closeInLoop(const std::string& reason);
  void handleRead(muduo::Timestamp now);
  void handleWrite();
  void handleError();
  void sendInLoop(const char* data, size_t len);

  muduo::net::EventLoop* loop_;
  int fd_ = -1;
  std::string dev_;
  std::unique_ptr<muduo::net::Channel> channel_;
  muduo::net::Buffer outputBuffer_;     ///< 未写出的剩余字节（POLLOUT 续传）

  DataCallback dataCb_;
  StateCallback stateCb_;
};

}  // namespace serial
}  // namespace mhost
