#pragma once
/// @file ChassisHost.h
/// @brief 轻量级底盘上位机组装层：TcpServer（TCP 文本指令）+ SerialPort
///        （串口帧协议）共处一个 EventLoop，向下喂板侧 CON 看门狗与速度
///        指令超时，向上提供 JSON 遥测。
///
/// 架构图、TCP 文本协议、看门狗层级表以本工程 docs/（architecture /
/// protocol-tcp / protocol-serial）为准（单一事实来源，注释里不复制）。

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <muduo/net/EventLoop.h>
#include <muduo/net/TcpConnection.h>
#include <muduo/net/TcpServer.h>

#include "link/ConSm.h"
#include "ota/OtaFlasher.h"
#include "proto/FrameCodec.h"
#include "serial/SerialPort.h"

namespace mhost {
namespace app {

struct HostOptions {
  std::string serialDev = "/dev/ttyUSB0";  ///< 串口设备（树莓派 USB3 口 CH9102）
  int serialBaud = 115200;                 ///< 与固件 USART3 一致（8N1）
  std::string listenAddr = "127.0.0.1:9000"; ///< TCP 监听（默认仅本机；开放
                                           ///  局域网须显式 --listen 0.0.0.0:9000）
  double repeatHz = 10.0;                  ///< 速度指令重发节拍（喂板侧 500ms 超时）
  double odomHz = 10.0;                    ///< odom JSON 推送频率（watch 订阅者）
  double slowHz = 1.0;                     ///< enc/imu/telem/status JSON 推送频率
  uint64_t conRetryMs = 2000;              ///< CON 断链后自动重连节流
  uint64_t rxTimeoutMs = 2000;             ///< 板帧静默告警阈值
  uint64_t enResendMs = 1000;              ///< STATUS en=0 补发 CMD_EN 的限频
  // ---- OTA 远程烧录（USB1 口 + 出厂 bootloader，见 docs/protocol-tcp.md）----
  std::string flashDev;                    ///< USB1 烧录口设备（空 = 禁用 flash）
  std::string firmwareDir = "firmware";    ///< 固件白名单目录（flash 只收其中文件名）
  std::string flashScript;                 ///< flash.py 绝对路径（C30D_Chassis/host_test）
  std::string flashPython = "python3";     ///< 解释器（-u 逐行输出）
  std::string flashTool = "auto";          ///< 透传 flash.py --tool（cubeprog/stm32flash）
  uint64_t quiesceTimeoutMs = 3000;        ///< QUIESCING 等 STATUS en=0 上限
  uint64_t recoverTimeoutMs = 15000;       ///< 烧录后等 CON 重建上限
};

class ChassisHost {
 public:
  ChassisHost(muduo::net::EventLoop* loop, const HostOptions& opts);
  ~ChassisHost();

  void start();

 private:
  // ---- TCP 侧 ----
  void onConnection(const muduo::net::TcpConnectionPtr& conn);
  void onMessage(const muduo::net::TcpConnectionPtr& conn, muduo::net::Buffer* buf,
                 muduo::Timestamp time);
  void handleLine(const muduo::net::TcpConnectionPtr& conn, std::string line);
  void handleCommand(const muduo::net::TcpConnectionPtr& conn,
                     const std::vector<std::string>& tok);

  // ---- 各命令实现（tok[0] 为命令字）----
  void cmdTwist(const muduo::net::TcpConnectionPtr& conn, const std::vector<std::string>& tok);
  void cmdRpm(const muduo::net::TcpConnectionPtr& conn, const std::vector<std::string>& tok);
  void cmdStop(const muduo::net::TcpConnectionPtr& conn);
  void cmdEnable(const muduo::net::TcpConnectionPtr& conn, const std::vector<std::string>& tok);
  void cmdPid(const muduo::net::TcpConnectionPtr& conn, const std::vector<std::string>& tok);
  void cmdSign(const muduo::net::TcpConnectionPtr& conn, const std::vector<std::string>& tok);
  void cmdPing(const muduo::net::TcpConnectionPtr& conn);
  void cmdWatch(const muduo::net::TcpConnectionPtr& conn, const std::string& arg);
  void cmdStat(const muduo::net::TcpConnectionPtr& conn);
  void cmdCon(const muduo::net::TcpConnectionPtr& conn);
  void cmdHelp(const muduo::net::TcpConnectionPtr& conn);

  // ---- OTA 远程烧录命令与状态机（流程见 docs/protocol-tcp.md）----
  void cmdFlash(const muduo::net::TcpConnectionPtr& conn,
                const std::vector<std::string>& tok);
  void cmdFlashAbort(const muduo::net::TcpConnectionPtr& conn,
                     const std::vector<std::string>& tok);
  void cmdFlashStatus(const muduo::net::TcpConnectionPtr& conn);
  void otaBeginFlasher();                       ///< QUIESCING 完成/超时 → 启动子进程
  void otaOnChildLine(const std::string& line); ///< flash.py 输出解析与转发
  void otaOnChildExit(bool ok, int exitStatus);
  void otaFinish(const std::string& resultLine); ///< 统一收尾（回 IDLE，保持失能）
  void otaSendProgress(const std::string& line); ///< 进度发发起者（已断则落日志）
  const char* otaStateName() const;
  bool otaBusy() const { return otaState_ != OtaState::kNone; }
  static bool isLoopbackConn(const muduo::net::TcpConnectionPtr& conn);

  // ---- 串口/协议侧 ----
  void onSerialData(const char* data, size_t len);
  void onSerialState(bool up, const std::string& reason);
  void onFrame(const proto::Frame& f);
  bool sendFrameToBoard(uint8_t type, const std::vector<uint8_t>& payload);
  bool sendConFrame(proto::ConMsgType msgType);
  void onCommState(link::ConSm::CommState comm);

  // ---- 周期任务 ----
  void onConTick();        ///< 100ms：ConSm deadline 驱动
  void onRepeatTimer();    ///< repeatHz：速度指令重发
  void onOdomTimer();      ///< odomHz：odom JSON
  void onSlowTimer();      ///< slowHz：enc/imu/telem/status JSON
  void onWatchdogTimer();  ///< 500ms：rx 静默检查 + CON 自动重连

  // ---- 辅助 ----
  void broadcastAll(const std::string& line);      ///< 全部 TCP 连接（安全事件）
  void sendToWatchers(const std::string& line);    ///< 仅 watch 订阅者（遥测）
  static bool parseDouble(const std::string& s, double* out);
  static std::vector<std::string> splitTokens(const std::string& line);
  static uint64_t steadyMs();  ///< 单调毫秒（全部时序基准，不受系统对时影响）

  muduo::net::EventLoop* loop_;
  HostOptions opts_;
  std::unique_ptr<muduo::net::TcpServer> server_;  ///< TCP 服务端（构造于监听地址解析后）
  std::set<muduo::net::TcpConnectionPtr> conns_;     ///< 全部 TCP 连接（loop 线程独占）
  std::set<muduo::net::TcpConnectionPtr> watchers_;  ///< 订阅遥测的连接

  serial::SerialPort serial_;
  proto::FrameParser parser_;
  link::ConSm conSm_;
  uint8_t txSeq_ = 0;

  // ---- 当前速度指令（重发状态）----
  enum class CmdMode : uint8_t { kNone, kTwist, kRpm };
  CmdMode cmdMode_ = CmdMode::kNone;
  float twist_[3] = {0.0f, 0.0f, 0.0f};
  float rpm_[3] = {0.0f, 0.0f, 0.0f};
  muduo::net::TcpConnectionPtr speedConn_;  ///< 最近下发速度指令的连接

  // ---- 遥测缓存（最新一帧）----
  proto::EncData enc_{};
  proto::ImuData imu_{};
  proto::TelemetryData telem_{};
  proto::StatusData status_{};
  proto::OdomData odom_{};
  bool haveEnc_ = false;
  bool haveImu_ = false;
  bool haveTelem_ = false;
  bool haveOdom_ = false;

  // ---- PING RTT 测量 ----
  uint32_t lastPingId_ = 0;
  uint64_t lastPingSentMs_ = 0;
  muduo::net::TcpConnectionPtr pingConn_;

  // ---- 看门狗/自愈状态 ----
  uint64_t lastFrameMs_ = 0;       ///< 最近收到板帧时刻（0=从未）
  bool rxSilenceWarned_ = false;
  uint64_t lastRetryMs_ = 0;       ///< 最近一次 CON 重连尝试
  uint64_t lastEnSentMs_ = 0;      ///< 最近一次 CMD_EN 发送（补发限频）
  bool userDisabled_ = false;      ///< 用户手动失能（粘性：压制 READY 自愈补发）

  // ---- OTA 远程烧录状态机 ----
  // 流程：kQuiescing（停速度重发+STOP/EN0，等 STATUS en=0 确认，超时兜底
  // 继续进 BL——复位本身就是硬停车）→ kFlashing（ConSm 强制 reset，子进程
  // 烧录，此阶段抑制 rx 静默告警与 CON 重连）→ kRecovering（等 CON 重建）
  // → kNone（保持失能，人工 en 1 才恢复运动）。
  enum class OtaState : uint8_t { kNone, kQuiescing, kFlashing, kRecovering };
  OtaState otaState_ = OtaState::kNone;
  muduo::net::TcpConnectionPtr otaConn_;  ///< 发起者（断开不中止任务，仅停止回显）
  std::string otaFile_;                   ///< 本次烧录文件名（白名单内 basename）
  std::string otaStage_;                  ///< flash.py 最近 STAGE
  std::string otaLastLine_;               ///< 最近一行子进程输出（诊断）
  uint64_t otaPhaseStartMs_ = 0;          ///< 当前阶段起始（超时基准）
  int otaCliLineCount_ = 0;               ///< cli| 进度透传限频计数
  std::unique_ptr<ota::OtaFlasher> flasher_;

  // ---- 统计 ----
  uint64_t txFrames_ = 0;
  muduo::Timestamp startTime_;

  std::string statSummary() const;
};

}  // namespace app
}  // namespace mhost
