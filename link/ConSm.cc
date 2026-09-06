#include "ConSm.h"

#include "log/Logger.h"

namespace mhost {
namespace link {

const char* ConSm::stateName(State s) {
  switch (s) {
    case ST_DISCONNECTED: return "DISCONNECTED";
    case ST_CONNECTING: return "CONNECTING";
    case ST_CONNECTED: return "CONNECTED";
    case ST_DISCONNECTING: return "DISCONNECTING";
  }
  return "?";
}

ConSm::ConSm(SendFn sendFn, CommCb commCb)
    : sendFn_(std::move(sendFn)), commCb_(std::move(commCb)) {}

// ---------------- 应用接口 ----------------

int ConSm::connect(uint64_t nowMs) {
  if (state_ == ST_CONNECTED) return 0;
  if (state_ == ST_CONNECTING) return -1;
  process(EV_ENABLE, nowMs);
  return 0;
}

int ConSm::disconnect(uint64_t nowMs) {
  if (state_ == ST_CONNECTED) {
    pingRep_ = 0;
    process(EV_DISABLE, nowMs);
    return 0;
  }
  if (state_ == ST_DISCONNECTED) return 0;
  return -1;  // CONNECTING / DISCONNECTING：忙
}

void ConSm::onFrame(proto::ConMsgType msgType, uint64_t nowMs) {
  switch (msgType) {
    case proto::CON_ENABLE_REQUEST:
      // 客户端角色不预期服务端的建链请求（DBAL 对齐），仍按事件处理保持容错
      process(EV_ENABLE_REQUEST, nowMs);
      break;
    case proto::CON_ENABLE_RESPONSE:
      process(EV_ACCEPT, nowMs);
      break;
    case proto::CON_DISABLE_REQUEST:
      process(EV_DISABLE_REQUEST, nowMs);
      break;
    case proto::CON_DISABLE_RESPONSE:
      // 对端拒绝（DBAL 语义：DISABLE_RESPONSE 到达即视为连接请求被拒）
      process(EV_REJECT, nowMs);
      break;
    case proto::CON_PING_REQUEST:
      LOG_WARN << "CON 收到 PING_REQUEST（客户端角色不预期）";
      break;
    case proto::CON_PING_RESPONSE:
      schedulePingStd(nowMs);
      break;
  }
}

// ---------------- 定时器驱动（对齐 con.py tick） ----------------

void ConSm::tick(uint64_t nowMs) {
  // 握手应答超时：重传或 REJECT
  if (0 < rspDeadline_ && rspDeadline_ <= nowMs) {
    rspDeadline_ = 0;
    bool retrying = (reqRep_ < kConRetryMax &&
                     (state_ == ST_CONNECTING || state_ == ST_DISCONNECTING));
    if (retrying) {
      ++reqRep_;
      if (sendFn_(curReq_)) {
        rspDeadline_ = nowMs + kConRspTimeoutMs;
      } else {
        process(EV_REJECT, nowMs);
      }
    } else {
      process(EV_REJECT, nowMs);
    }
  }

  // 保活超时：重发或 REJECT
  if (0 < pingDeadline_ && pingDeadline_ <= nowMs) {
    pingDeadline_ = 0;
    if (pingRep_ <= kMaxPingRepeat) {
      ++pingRep_;
      if (comm_ == COMM_READY) {
        if (!sendFn_(proto::CON_PING_REQUEST)) {
          process(EV_REJECT, nowMs);
          return;
        }
        if (pingArmed()) {
          pingDeadline_ = nowMs + kRepeatPingTimeMs;  // 未应答 → 300ms 后重发
        }
      }
    } else {
      process(EV_REJECT, nowMs);
    }
  }
}

// ---------------- 状态转移表（对齐 AlCon.cpp connectionSm） ----------------

void ConSm::sm(Event ev, Deferred& d) {
  LOG_DEBUG << "CON sm [state=" << stateName(state_) << " event=" << static_cast<int>(ev) << "]";

  switch (state_) {
    case ST_DISCONNECTED:
      if (ev == EV_ENABLE) {
        state_ = ST_CONNECTING;
        curReq_ = proto::CON_ENABLE_REQUEST;
        d.send = true;
        d.sendType = proto::CON_ENABLE_REQUEST;
      } else if (ev == EV_ENABLE_REQUEST) {
        state_ = ST_CONNECTING;
        curReq_ = proto::CON_ENABLE_RESPONSE;
        d.send = true;
        d.sendType = proto::CON_ENABLE_RESPONSE;
      } else if (ev == EV_REJECT) {
        // 无等待者，结果由 commCb 体现
      }
      break;

    case ST_CONNECTING:
      if (ev == EV_ACCEPT) {
        state_ = ST_CONNECTED;
        toConnected(d);
      } else if (ev == EV_REJECT) {
        state_ = ST_DISCONNECTED;
        d.cancelRsp = true;
        reqRep_ = 0;
      } else if (ev == EV_ENABLE_REQUEST) {
        state_ = ST_CONNECTED;
        curReq_ = proto::CON_ENABLE_RESPONSE;
        d.send = true;
        d.sendType = proto::CON_ENABLE_RESPONSE;
        toConnected(d);
      } else if (ev == EV_DISABLE) {
        state_ = ST_DISCONNECTING;
        curReq_ = proto::CON_DISABLE_REQUEST;
        d.send = true;
        d.sendType = proto::CON_DISABLE_REQUEST;
      }
      break;

    case ST_CONNECTED:
      if (ev == EV_ENABLE_REQUEST) {
        // 板子重上电自愈路径：重新应答并回到 READY
        curReq_ = proto::CON_ENABLE_RESPONSE;
        d.send = true;
        d.sendType = proto::CON_ENABLE_RESPONSE;
        if (comm_ != COMM_NOT_READY) {
          comm_ = COMM_NOT_READY;
          d.fireComm = true;
          d.newComm = COMM_NOT_READY;
        }
        toConnected(d);
      } else if (ev == EV_ACCEPT) {
        // 容忍重复应答
      } else if (ev == EV_DISABLE) {
        state_ = ST_DISCONNECTING;
        curReq_ = proto::CON_DISABLE_REQUEST;
        d.send = true;
        d.sendType = proto::CON_DISABLE_REQUEST;
      } else if (ev == EV_DISABLE_REQUEST) {
        // 对端请求拆链 → 断开并回执。有意偏离 con.py：原版此分支不置
        // DISCONNECTED（滞留 CONNECTED），导致之后 connect() 误报”已连接”。
        state_ = ST_DISCONNECTED;
        toDisconnected(d);
        curReq_ = proto::CON_DISABLE_RESPONSE;
        d.send = true;
        d.sendType = proto::CON_DISABLE_RESPONSE;
      } else if (ev == EV_REJECT) {
        state_ = ST_DISCONNECTED;
        d.cancelRsp = true;
        d.cancelPing = true;
        if (comm_ != COMM_NOT_READY) {
          comm_ = COMM_NOT_READY;
          d.fireComm = true;
          d.newComm = COMM_NOT_READY;
        }
      }
      break;

    case ST_DISCONNECTING:
      if (ev == EV_REJECT) {
        state_ = ST_DISCONNECTED;
        toDisconnected(d);
      } else {
        LOG_WARN << "CON sm: DISCONNECTING 态不预期事件 " << static_cast<int>(ev);
      }
      break;
  }
}

// ---------------- 动作（对齐 actionToConnected 等） ----------------

void ConSm::toConnected(Deferred& d) {
  reqRep_ = 0;
  d.cancelRsp = true;
  comm_ = COMM_READY;
  d.fireComm = true;
  d.newComm = COMM_READY;
  d.armPing = true;
}

void ConSm::toDisconnected(Deferred& d) {
  d.cancelRsp = true;
  d.cancelPing = true;
  if (comm_ != COMM_NOT_READY) {
    comm_ = COMM_NOT_READY;
    d.fireComm = true;
    d.newComm = COMM_NOT_READY;
  }
  pingRep_ = 0;
  reqRep_ = 0;
}

// ---------------- 定时器装载与副作用集中执行 ----------------

bool ConSm::pingArmed() const {
  return comm_ == COMM_READY && (state_ == ST_CONNECTED || state_ == ST_DISCONNECTING);
}

void ConSm::schedulePingStd(uint64_t nowMs) {
  pingRep_ = 0;
  if (pingArmed()) {
    pingDeadline_ = nowMs + kPingTimeMs;
  } else {
    pingDeadline_ = 0;
  }
}

void ConSm::process(Event ev, uint64_t nowMs) {
  Deferred d;
  sm(ev, d);
  flush(d, nowMs);
  if (d.hasFollowUp) {
    process(d.followUp, nowMs);
  }
}

void ConSm::flush(Deferred& d, uint64_t nowMs) {
  if (d.cancelRsp) rspDeadline_ = 0;
  if (d.cancelPing) pingDeadline_ = 0;

  if (d.send) {
    bool ok = false;
    if (sendFn_) ok = sendFn_(d.sendType);
    if (ok) {
      if (d.sendType == proto::CON_ENABLE_RESPONSE || d.sendType == proto::CON_DISABLE_RESPONSE) {
        d.hasFollowUp = true;
        d.followUp = EV_ACCEPT;  // 应答发出即视为对方接受（DBAL flush 语义）
      } else {
        d.armRsp = true;  // REQUEST：装载应答超时
      }
    } else {
      d.hasFollowUp = true;
      d.followUp = EV_REJECT;
    }
  }

  if (d.armRsp) rspDeadline_ = nowMs + kConRspTimeoutMs;
  if (d.armPing) schedulePingStd(nowMs);

  if (d.fireComm && commCb_) commCb_(d.newComm);
}

}  // namespace link
}  // namespace mhost
