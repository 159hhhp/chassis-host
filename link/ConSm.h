#pragma once
/// @file ConSm.h
/// @brief 链路连接（CON）客户端状态机（四态）
///
/// 移植自 C30D_Chassis/host_test/con.py（设计源自 uc-platform DBAL
/// Connection / AlCon.cpp），与板侧 App/con.c（服务端两态）配对；
/// 时序常量为类内 constexpr，改协议须与 con.py 同步。
/// 状态转移只收集动作（Deferred），flush() 集中执行副作用，发送失败
/// → REJECT；断链后的自动重连节流由上层 ChassisHost 实现。
/// 单线程驱动（EventLoop 定时器），无锁；nowMs 由调用方注入以便
/// 虚拟时钟单测。

#include <cstdint>
#include <functional>

#include "proto/FrameDefs.h"

namespace mhost {
namespace link {

class ConSm {
 public:
  enum State : int8_t {
    ST_DISCONNECTED = 0,
    ST_CONNECTING = 1,
    ST_CONNECTED = 2,
    ST_DISCONNECTING = 3,
  };

  /// 对外通信态（对应固件 con_comm_state_t，仅变化时上抛）
  enum CommState : int8_t { COMM_NOT_READY = 0, COMM_READY = 1 };

  // ---- 时序常量（ms），与 con.py / 板侧一致 ----
  static constexpr uint64_t kPingTimeMs = 1000;        ///< 标准保活间隔
  static constexpr uint64_t kRepeatPingTimeMs = 300;   ///< 未应答重发间隔
  static constexpr int kMaxPingRepeat = 2;             ///< 重发次数上限
  static constexpr uint64_t kConRspTimeoutMs = 400;    ///< 握手应答超时
  static constexpr int kConRetryMax = 5;               ///< 握手重传上限
  static constexpr uint64_t kSelfDisableMs = 3000;     ///< 客户端预留（对齐 DBAL）

  /// 发送一帧 CON（载荷由本状态机构造），false = 传输失败
  using SendFn = std::function<bool(proto::ConMsgType)>;
  /// 通信态变化回调（仅 NOT_READY→READY / READY→NOT_READY 边沿触发）
  using CommCb = std::function<void(CommState)>;

  ConSm(SendFn sendFn, CommCb commCb);

  /// 请求建链。0=已受理或已连接；-1=握手中（忙）
  int connect(uint64_t nowMs);
  /// 请求拆链（发 DISABLE_REQUEST 并等确认）。0=已受理或本就断开；-1=忙
  int disconnect(uint64_t nowMs);
  /// 强制回 DISCONNECTED：清空在途握手/保活，不发任何帧（区别于优雅
  /// disconnect）。用于 OTA 烧录等"对端即将复位消失"的场景；通信态若
  /// 原为 READY 会补一次 NOT_READY 边沿回调。
  void reset();
  /// 喂入一帧 CON 消息（调用方先经 proto::unpackCon 校验）
  void onFrame(proto::ConMsgType msgType, uint64_t nowMs);
  /// 周期驱动 deadline（建议 50~100ms 一拍）
  void tick(uint64_t nowMs);

  State state() const { return state_; }
  CommState comm() const { return comm_; }

  static const char* stateName(State s);
  static const char* commName(CommState c) { return c == COMM_READY ? "READY" : "NOT_READY"; }

 private:
  enum Event : int8_t {
    EV_ENABLE,          ///< 应用请求建链
    EV_ENABLE_REQUEST,  ///< 收到对端建链请求（客户端角色仅告警容错）
    EV_DISABLE,         ///< 应用请求拆链
    EV_DISABLE_REQUEST, ///< 收到对端拆链请求
    EV_ACCEPT,          ///< 收到 ENABLE_RESPONSE
    EV_REJECT,          ///< 重传耗尽 / 发送失败 / 收到 DISABLE_RESPONSE
  };

  /// 状态转移收集的副作用，flush() 集中执行（对齐 AlCon.cpp Deferred）
  struct Deferred {
    bool send = false;
    proto::ConMsgType sendType{};
    bool cancelRsp = false;
    bool cancelPing = false;
    bool armRsp = false;
    bool armPing = false;
    bool fireComm = false;
    CommState newComm = COMM_NOT_READY;
    Event followUp = EV_ENABLE;  ///< 后续事件；约定用 EV_ENABLE 表示“无”（flush 前不会触发转移）
    bool hasFollowUp = false;
  };

  void process(Event ev, uint64_t nowMs);
  void sm(Event ev, Deferred& d);
  void flush(Deferred& d, uint64_t nowMs);
  void toConnected(Deferred& d);
  void toDisconnected(Deferred& d);
  bool pingArmed() const;
  void schedulePingStd(uint64_t nowMs);

  SendFn sendFn_;
  CommCb commCb_;

  State state_ = ST_DISCONNECTED;
  CommState comm_ = COMM_NOT_READY;

  uint64_t rspDeadline_ = 0;      ///< 握手应答/重传 deadline（0=未武装）
  uint64_t pingDeadline_ = 0;     ///< 保活 deadline
  int pingRep_ = 0;               ///< 未应答 ping 已重发次数
  int reqRep_ = 0;                ///< 在途 REQUEST 已重传次数
  proto::ConMsgType curReq_ = proto::CON_ENABLE_REQUEST;  ///< 在途 REQUEST 类型
};

}  // namespace link
}  // namespace mhost
