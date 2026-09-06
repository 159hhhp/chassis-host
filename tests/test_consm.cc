/// @file test_consm.cc
/// @brief CON 客户端四态机单测：虚拟时钟逐场景校验时序（对齐 host_test/con.py）
///
/// 场景覆盖：建链成功 / 握手重传耗尽拒绝 / ping 丢失重发耗尽断链 /
/// 优雅拆链 / 发送失败即拒 / 板侧重上电（CONNECTED 收 ENABLE_REQUEST）/
/// 对端 DISABLE_RESPONSE / CONNECTING 期 connect() 忙。

#include <vector>

#include "link/ConSm.h"
#include "proto/FrameDefs.h"
#include "test_util.h"

using mhost::link::ConSm;
using MT = mhost::proto::ConMsgType;

namespace {

/// 测试夹具：10ms 步进虚拟时钟 + 发送/通信态记录
struct Harness {
  uint64_t now = 1000;
  std::vector<MT> sent;
  std::vector<ConSm::CommState> commEvents;
  bool sendOk = true;
  ConSm sm;

  Harness()
      : sm([this](MT mt) {
          if (sendOk) sent.push_back(mt);
          return sendOk;
        },
        [this](ConSm::CommState c) { commEvents.push_back(c); }) {}

  /// 推进虚拟时钟到 t（含端点），每 10ms 调一次 tick
  void tickTo(uint64_t t) {
    while (now < t) {
      now += 10;
      sm.tick(now);
    }
  }
  int count(MT mt) const {
    int n = 0;
    for (MT m : sent) {
      if (m == mt) ++n;
    }
    return n;
  }
};

void testConnectSuccess() {
  Harness h;
  CHECK_EQ(h.sm.connect(h.now), 0);
  CHECK_EQ(h.count(MT::CON_ENABLE_REQUEST), 1);
  CHECK(h.sm.state() == ConSm::ST_CONNECTING);
  CHECK_EQ(h.sm.connect(h.now), -1);  // 握手中：忙
  CHECK_EQ(h.count(MT::CON_ENABLE_REQUEST), 1);

  h.now += 100;
  h.sm.onFrame(MT::CON_ENABLE_RESPONSE, h.now);
  CHECK(h.sm.state() == ConSm::ST_CONNECTED);
  CHECK(h.sm.comm() == ConSm::COMM_READY);
  CHECK_EQ(h.commEvents.size(), 1u);
  CHECK(h.commEvents[0] == ConSm::COMM_READY);

  // 1s 后第一个 PING；应答后回到标准周期
  h.tickTo(h.now + 1000);
  CHECK_EQ(h.count(MT::CON_PING_REQUEST), 1);
  h.now += 50;
  h.sm.onFrame(MT::CON_PING_RESPONSE, h.now);
  h.tickTo(h.now + 1000);
  CHECK_EQ(h.count(MT::CON_PING_REQUEST), 2);
  h.now += 30;
  h.sm.onFrame(MT::CON_PING_RESPONSE, h.now);
  h.tickTo(h.now + 950);
  CHECK_EQ(h.count(MT::CON_PING_REQUEST), 2);  // 未到期
}

void testHandshakeRetryExhausted() {
  Harness h;
  h.sm.connect(h.now);  // t=1000
  // 5 次重传（400ms 一次）后耗尽 → REJECT → DISCONNECTED
  h.tickTo(1000 + 400 * 6 + 50);
  CHECK_EQ(h.count(MT::CON_ENABLE_REQUEST), 6);  // 首发 + 5 次重传
  CHECK(h.sm.state() == ConSm::ST_DISCONNECTED);
  CHECK(h.sm.comm() == ConSm::COMM_NOT_READY);
  CHECK(h.commEvents.empty());  // 从未 READY，无边沿事件
}

void testPingLossReject() {
  Harness h;
  h.sm.connect(h.now);   // t=1000
  h.now += 100;          // t=1100 建链成功
  h.sm.onFrame(MT::CON_ENABLE_RESPONSE, h.now);
  // PING@2100 / 2400 / 2700（300ms 重发 ×2），3000ms 无应答 → 断链
  h.tickTo(3250);
  CHECK_EQ(h.count(MT::CON_PING_REQUEST), 3);
  CHECK(h.sm.state() == ConSm::ST_DISCONNECTED);
  CHECK_EQ(h.commEvents.size(), 2u);
  CHECK(h.commEvents[0] == ConSm::COMM_READY);
  CHECK(h.commEvents[1] == ConSm::COMM_NOT_READY);
}

void testGracefulDisconnect() {
  Harness h;
  h.sm.connect(h.now);
  h.now += 100;
  h.sm.onFrame(MT::CON_ENABLE_RESPONSE, h.now);
  CHECK_EQ(h.sm.disconnect(h.now), 0);
  CHECK(h.sm.state() == ConSm::ST_DISCONNECTING);
  CHECK_EQ(h.count(MT::CON_DISABLE_REQUEST), 1);
  h.now += 80;
  h.sm.onFrame(MT::CON_DISABLE_RESPONSE, h.now);  // 对端确认 → REJECT 事件 → 断开
  CHECK(h.sm.state() == ConSm::ST_DISCONNECTED);
  CHECK(h.sm.comm() == ConSm::COMM_NOT_READY);
  CHECK_EQ(h.commEvents.size(), 2u);
  CHECK_EQ(h.sm.disconnect(h.now), 0);  // 已断开：幂等
}

void testSendFailureReject() {
  Harness h;
  h.sendOk = false;
  h.sm.connect(h.now);
  CHECK(h.sm.state() == ConSm::ST_DISCONNECTED);
  CHECK(h.commEvents.empty());
  CHECK(h.sent.empty());
}

void testBoardResetImplicit() {
  Harness h;
  h.sm.connect(h.now);
  h.now += 100;
  h.sm.onFrame(MT::CON_ENABLE_RESPONSE, h.now);
  CHECK(h.sm.comm() == ConSm::COMM_READY);

  // CONNECTED 态收到 ENABLE_REQUEST（板重上电重入建链）：重发应答并保持
  // READY。con.py 语义：to_connected 会覆盖 Deferred 里的 NOT_READY 事件，
  // 对观察者只表现为一次 READY（无 NOT_READY 抖动）。
  h.now += 2000;
  h.sm.onFrame(MT::CON_ENABLE_REQUEST, h.now);
  CHECK_EQ(h.count(MT::CON_ENABLE_RESPONSE), 1);
  CHECK(h.sm.state() == ConSm::ST_CONNECTED);
  CHECK(h.sm.comm() == ConSm::COMM_READY);
  CHECK_EQ(h.commEvents.size(), 2u);  // 建链 READY + 重入建链 READY（无 NOT_READY 抖动）
  CHECK(h.commEvents[0] == ConSm::COMM_READY);
  CHECK(h.commEvents[1] == ConSm::COMM_READY);
}

void testPeerDisableResponse() {
  Harness h;
  h.sm.connect(h.now);
  h.now += 100;
  h.sm.onFrame(MT::CON_ENABLE_RESPONSE, h.now);
  h.now += 500;
  h.sm.onFrame(MT::CON_DISABLE_RESPONSE, h.now);  // 对端拒绝/拆链确认
  CHECK(h.sm.state() == ConSm::ST_DISCONNECTED);
  CHECK(h.sm.comm() == ConSm::COMM_NOT_READY);
}

void testReconnectAfterReject() {
  Harness h;
  h.sm.connect(h.now);
  h.tickTo(1000 + 400 * 6 + 50);  // 握手耗尽
  CHECK(h.sm.state() == ConSm::ST_DISCONNECTED);
  // 重新 connect → 再次进入 CONNECTING 并重发请求
  CHECK_EQ(h.sm.connect(h.now), 0);
  CHECK(h.sm.state() == ConSm::ST_CONNECTING);
  CHECK_EQ(h.count(MT::CON_ENABLE_REQUEST), 7);
}

}  // namespace

int main() {
  testConnectSuccess();
  testHandshakeRetryExhausted();
  testPingLossReject();
  testGracefulDisconnect();
  testSendFailureReject();
  testBoardResetImplicit();
  testPeerDisableResponse();
  testReconnectAfterReject();
  return testSummary("test_consm");
}
