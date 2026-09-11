#include "ChassisHost.h"

#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include "log/Logger.h"

namespace mhost {
namespace app {

namespace {

constexpr size_t kMaxLineLen = 512;  ///< TCP 单行上限（超长直接丢弃防滥用）

bool isSafeFirmwareName(const std::string& name) {
  if (name.size() < 5 || name.compare(name.size() - 4, 4, ".hex") != 0) return false;
  for (unsigned char c : name) {
    if (!std::isalnum(c) && c != '.' && c != '_' && c != '-') return false;
  }
  return name.find("..") == std::string::npos;
}

/// 监听地址 "ip:port" 拆分（muduo InetAddress 需要分开传入）
bool splitListenAddr(const std::string& addr, std::string* ip, uint16_t* port) {
  size_t colon = addr.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 == addr.size()) return false;
  *ip = addr.substr(0, colon);
  int p = ::atoi(addr.c_str() + colon + 1);
  if (p <= 0 || p > 65535) return false;
  *port = static_cast<uint16_t>(p);
  return true;
}

/// 变参格式化追加
void appendf(std::string* s, const char* fmt, ...) {
  char buf[512];
  va_list args;
  va_start(args, fmt);
  int n = ::vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  if (n > 0) s->append(buf, static_cast<size_t>(n) < sizeof buf ? static_cast<size_t>(n) : sizeof buf - 1);
}

std::string jnum(double v, const char* fmt) {
  char buf[64];
  ::snprintf(buf, sizeof buf, fmt, v);
  return buf;
}

/// 三元素浮点数组 → JSON 数组
std::string jArr3(const float (&v)[3], const char* fmt) {
  std::string s = "[";
  for (int i = 0; i < 3; ++i) {
    if (i != 0) s += ",";
    s += jnum(v[i], fmt);
  }
  s += "]";
  return s;
}

/// 三元素 int32 数组 → JSON 数组（编码器累计计数）
std::string jArr3i(const int32_t (&v)[3]) {
  std::string s = "[";
  for (int i = 0; i < 3; ++i) {
    if (i != 0) s += ",";
    s += std::to_string(v[i]);
  }
  s += "]";
  return s;
}

}  // namespace

ChassisHost::ChassisHost(muduo::net::EventLoop* loop, const HostOptions& opts)
    : loop_(loop),
      opts_(opts),
      serial_(loop),
      conSm_([this](proto::ConMsgType mt) { return sendConFrame(mt); },
             [this](link::ConSm::CommState comm) { onCommState(comm); }),
      startTime_(muduo::Timestamp::now()) {
  std::string ip;
  uint16_t port = 0;
  if (!splitListenAddr(opts.listenAddr, &ip, &port)) {
    LOG_FATAL << "监听地址非法: '" << opts.listenAddr << "'（应为 ip:port）";
  }
  server_ = std::make_unique<muduo::net::TcpServer>(
      loop, muduo::net::InetAddress(ip, port), "chassis_host");
  server_->setConnectionCallback(
      std::bind(&ChassisHost::onConnection, this, std::placeholders::_1));
  server_->setMessageCallback(
      std::bind(&ChassisHost::onMessage, this, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  server_->setThreadNum(0);  // 单 Reactor：TCP 与串口同处一个 epoll，零锁
}

ChassisHost::~ChassisHost() {
  // loop.loop() 已返回（同线程）。forceClose 同步把连接置为 kDisconnected，
  // 保证 TcpConnection 析构断言通过
  for (const auto& conn : conns_) {
    conn->forceClose();
  }
  conns_.clear();
  watchers_.clear();
}

void ChassisHost::start() {
  serial_.setDataCallback(
      [this](const char* data, size_t len) { onSerialData(data, len); });
  serial_.setStateCallback(
      [this](bool up, const std::string& reason) { onSerialState(up, reason); });

  if (!serial_.openDevice(opts_.serialDev, opts_.serialBaud)) {
    LOG_WARN << "串口暂不可用（" << opts_.serialDev
             << "），watchdog 定时器会持续重试";
  }

  loop_->runEvery(0.1, [this] { onConTick(); });
  loop_->runEvery(1.0 / opts_.repeatHz, [this] { onRepeatTimer(); });
  loop_->runEvery(1.0 / opts_.odomHz, [this] { onOdomTimer(); });
  loop_->runEvery(1.0 / opts_.slowHz, [this] { onSlowTimer(); });
  loop_->runEvery(0.5, [this] { onWatchdogTimer(); });
  loop_->runEvery(5.0, [this] { LOG_INFO << "统计: " << statSummary(); });

  server_->start();
  LOG_INFO << "ChassisHost 启动: 串口=" << opts_.serialDev << "@" << opts_.serialBaud
           << " TCP=" << opts_.listenAddr << " repeatHz=" << opts_.repeatHz;
}

// ================= TCP 侧 =================

void ChassisHost::onConnection(const muduo::net::TcpConnectionPtr& conn) {
  if (conn->connected()) {
    conns_.insert(conn);
    LOG_INFO << "TCP 连接建立: " << conn->peerAddress().toIpPort() << "（当前 "
             << conns_.size() << " 个）";
    conn->send("C30D chassis host。输入 help 查看命令。\n");
    conn->send(std::string("{\"t\":\"con\",\"state\":\"") +
               link::ConSm::commName(conSm_.comm()) + "\"}\n");
  } else {
    conns_.erase(conn);
    watchers_.erase(conn);
    LOG_INFO << "TCP 连接断开: " << conn->peerAddress().toIpPort() << "（剩余 "
             << conns_.size() << " 个）";
    if (speedConn_ == conn) {
      // 看门狗延展：控制端掉线 → 停止重发并立即刹车
      LOG_WARN << "速度指令来源已断开，自动下发 CMD_STOP";
      cmdMode_ = CmdMode::kNone;
      speedConn_.reset();
      sendFrameToBoard(proto::PT_CMD_STOP, {});
    }
    if (pingConn_ == conn) pingConn_.reset();
    if (otaConn_ == conn) {
      // 升级发起端掉线不中止任务（擦写已开始就让它烧完），进度转日志
      otaConn_.reset();
      if (otaBusy()) LOG_WARN << "OTA 发起端已断开，任务继续（进度见日志）";
    }
  }
}

void ChassisHost::onMessage(const muduo::net::TcpConnectionPtr& conn, muduo::net::Buffer* buf,
                            muduo::Timestamp /*time*/) {
  // 文本行协议：以 \n 分行（容忍 \r\n），无行尾的残包留在缓冲
  while (buf->readableBytes() > 0) {
    const char* eol = buf->findEOL();
    if (eol == nullptr) {
      if (buf->readableBytes() > kMaxLineLen) {
        LOG_WARN << "TCP 半行超限，丢弃 " << buf->readableBytes() << "B";
        buf->retrieveAll();
      }
      break;
    }
    size_t len = static_cast<size_t>(eol - buf->peek());
    if (len <= kMaxLineLen) {
      handleLine(conn, std::string(buf->peek(), len));
    } else {
      LOG_WARN << "TCP 行超长 " << len << "B，丢弃";
    }
    buf->retrieveUntil(eol + 1);
  }
}

void ChassisHost::handleLine(const muduo::net::TcpConnectionPtr& conn, std::string line) {
  // 去掉 \r 与首尾空白
  while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
  size_t start = line.find_first_not_of(" \t");
  if (start == std::string::npos) return;
  line.erase(0, start);
  if (line[0] == '#') return;  // 脚本注释行

  handleCommand(conn, splitTokens(line));
}

void ChassisHost::handleCommand(const muduo::net::TcpConnectionPtr& conn,
                                const std::vector<std::string>& tok) {
  if (tok.empty()) return;
  const std::string& cmd = tok[0];

  // 升级忙态门控：烧录期间只放行查询/取消类命令（stop 也放行，无害）
  if (otaBusy() && cmd != "flash_status" && cmd != "flash_abort" && cmd != "stat" &&
      cmd != "con" && cmd != "help" && cmd != "quit" && cmd != "exit" &&
      cmd != "watch" && cmd != "stop") {
    conn->send(std::string("err busy: 固件升级进行中（") + otaStateName() +
               "），flash_status 查看 / flash_abort 取消\n");
    return;
  }

  if (cmd == "v") {
    cmdTwist(conn, tok);
  } else if (cmd == "rpm") {
    cmdRpm(conn, tok);
  } else if (cmd == "stop") {
    cmdStop(conn);
  } else if (cmd == "en") {
    cmdEnable(conn, tok);
  } else if (cmd == "pid") {
    cmdPid(conn, tok);
  } else if (cmd == "sign") {
    cmdSign(conn, tok);
  } else if (cmd == "ping") {
    cmdPing(conn);
  } else if (cmd == "watch") {
    cmdWatch(conn, tok.size() > 1 ? tok[1] : "");
  } else if (cmd == "stat") {
    cmdStat(conn);
  } else if (cmd == "con") {
    cmdCon(conn);
  } else if (cmd == "flash") {
    cmdFlash(conn, tok);
  } else if (cmd == "flash_status") {
    cmdFlashStatus(conn);
  } else if (cmd == "flash_abort") {
    cmdFlashAbort(conn, tok);
  } else if (cmd == "help") {
    cmdHelp(conn);
  } else if (cmd == "quit" || cmd == "exit") {
    conn->shutdown();
  } else {
    conn->send("err 未知命令 '" + cmd + "'，输入 help 查看命令\n");
  }
}

// ================= 命令实现 =================

void ChassisHost::cmdTwist(const muduo::net::TcpConnectionPtr& conn,
                           const std::vector<std::string>& tok) {
  double v[3] = {0.0, 0.0, 0.0};
  if (tok.size() < 2 || tok.size() > 4) {
    conn->send("err 用法: v <vx> [vy] [wz]\n");
    return;
  }
  for (int i = 1; i < static_cast<int>(tok.size()); ++i) {
    if (!parseDouble(tok[i], &v[i - 1])) {
      conn->send("err 参数非法: " + tok[i] + "\n");
      return;
    }
  }
  for (int i = 0; i < 3; ++i) twist_[i] = static_cast<float>(v[i]);
  cmdMode_ = CmdMode::kTwist;
  speedConn_ = conn;
  LOG_INFO << "twist 指令: vx=" << twist_[0] << " vy=" << twist_[1] << " wz=" << twist_[2];
  std::string rep = "ok twist vx=" + jnum(twist_[0], "%.3f") +
                    " vy=" + jnum(twist_[1], "%.3f") + " wz=" + jnum(twist_[2], "%.3f");
  rep += conSm_.comm() == link::ConSm::COMM_READY ? "（已下发，10Hz 重发中）\n"
                                                  : "（链路未就绪，建链后生效）\n";
  conn->send(rep);
}

void ChassisHost::cmdRpm(const muduo::net::TcpConnectionPtr& conn,
                         const std::vector<std::string>& tok) {
  double v[3] = {0.0, 0.0, 0.0};
  if (tok.size() < 2 || tok.size() > 4) {
    conn->send("err 用法: rpm <r1> [r2] [r3]\n");
    return;
  }
  for (int i = 1; i < static_cast<int>(tok.size()); ++i) {
    if (!parseDouble(tok[i], &v[i - 1])) {
      conn->send("err 参数非法: " + tok[i] + "\n");
      return;
    }
  }
  for (int i = 0; i < 3; ++i) rpm_[i] = static_cast<float>(v[i]);
  cmdMode_ = CmdMode::kRpm;
  speedConn_ = conn;
  LOG_INFO << "rpm 指令: " << rpm_[0] << " " << rpm_[1] << " " << rpm_[2];
  conn->send("ok rpm " + jArr3(rpm_, "%.1f") + "（已登记，10Hz 重发中）\n");
}

void ChassisHost::cmdStop(const muduo::net::TcpConnectionPtr& conn) {
  cmdMode_ = CmdMode::kNone;
  speedConn_.reset();
  if (sendFrameToBoard(proto::PT_CMD_STOP, {})) {
    LOG_INFO << "CMD_STOP 已下发（刹车 + 退出闭环）";
    conn->send("ok stop\n");
  } else {
    conn->send("err stop: 串口未打开\n");
  }
}

void ChassisHost::cmdEnable(const muduo::net::TcpConnectionPtr& conn,
                            const std::vector<std::string>& tok) {
  if (tok.size() != 2 || (tok[1] != "0" && tok[1] != "1")) {
    conn->send("err 用法: en <0|1>\n");
    return;
  }
  uint64_t now = steadyMs();
  if (tok[1] == "1") {
    userDisabled_ = false;
    lastEnSentMs_ = now;
    sendFrameToBoard(proto::PT_CMD_EN, proto::packCmdEn(true));
    if (conSm_.state() == link::ConSm::ST_DISCONNECTED) {
      conSm_.connect(now);  // 断链状态下使能 → 同时发起建链
    }
    conn->send("ok en=1（并自动建链）\n");
  } else {
    // 失能：置粘性标志（CONNECTING 期 disconnect 会忙返回，READY 自愈补发
    // 依此标志让位），一并拆链并停止速度重发
    userDisabled_ = true;
    cmdMode_ = CmdMode::kNone;
    sendFrameToBoard(proto::PT_CMD_EN, proto::packCmdEn(false));
    conSm_.disconnect(now);
    conn->send("ok en=0（已拆链，重连后仍保持失能）\n");
  }
}

void ChassisHost::cmdPid(const muduo::net::TcpConnectionPtr& conn,
                         const std::vector<std::string>& tok) {
  double kp, ki, kd;
  if (tok.size() != 4 || !parseDouble(tok[1], &kp) || !parseDouble(tok[2], &ki) ||
      !parseDouble(tok[3], &kd)) {
    conn->send("err 用法: pid <kp> <ki> <kd>\n");
    return;
  }
  if (sendFrameToBoard(proto::PT_CMD_PID, proto::packCmdPid(
      static_cast<float>(kp), static_cast<float>(ki), static_cast<float>(kd)))) {
    conn->send("ok pid " + jnum(kp, "%.4g") + " " + jnum(ki, "%.4g") + " " + jnum(kd, "%.4g") + "\n");
  } else {
    conn->send("err pid: 串口未打开\n");
  }
}

void ChassisHost::cmdSign(const muduo::net::TcpConnectionPtr& conn,
                          const std::vector<std::string>& tok) {
  if (tok.size() != 4) {
    conn->send("err 用法: sign <s1> <s2> <s3>（±1）\n");
    return;
  }
  int8_t s[3];
  for (int i = 0; i < 3; ++i) {
    double v;
    if (!parseDouble(tok[i + 1], &v)) {
      conn->send("err 参数非法: " + tok[i + 1] + "\n");
      return;
    }
    s[i] = v >= 0 ? 1 : -1;
  }
  if (sendFrameToBoard(proto::PT_CMD_SIGN, proto::packCmdSign(s))) {
    conn->send("ok sign " + std::to_string(s[0]) + " " + std::to_string(s[1]) + " " +
               std::to_string(s[2]) + "\n");
  } else {
    conn->send("err sign: 串口未打开\n");
  }
}

void ChassisHost::cmdPing(const muduo::net::TcpConnectionPtr& conn) {
  if (!serial_.isOpen()) {
    conn->send("err ping: 串口未打开\n");
    return;
  }
  uint64_t now = steadyMs();
  lastPingId_ = static_cast<uint32_t>(now & 0xFFFFFFFFu);
  lastPingSentMs_ = now;
  pingConn_ = conn;
  sendFrameToBoard(proto::PT_CMD_PING, proto::packPing(lastPingId_));
}

void ChassisHost::cmdWatch(const muduo::net::TcpConnectionPtr& conn, const std::string& arg) {
  bool on;
  if (arg == "on") {
    on = true;
  } else if (arg == "off") {
    on = false;
  } else {
    on = watchers_.count(conn) == 0;  // 不带参数 = 取反
  }
  if (on) {
    watchers_.insert(conn);
  } else {
    watchers_.erase(conn);
  }
  LOG_DEBUG << "watch " << (on ? "on" : "off") << "（当前 " << watchers_.size() << " 订阅者）";
  conn->send(std::string("ok watch=") + (on ? "on" : "off") + "\n");
}

std::string ChassisHost::statSummary() const {
  const auto& st = parser_.stats();
  std::string cmd = "none";
  if (cmdMode_ == CmdMode::kTwist) {
    cmd = "twist(" + jnum(twist_[0], "%.3f") + "," + jnum(twist_[1], "%.3f") + "," +
          jnum(twist_[2], "%.3f") + ")";
  } else if (cmdMode_ == CmdMode::kRpm) {
    cmd = "rpm(" + jnum(rpm_[0], "%.1f") + "," + jnum(rpm_[1], "%.1f") + "," +
          jnum(rpm_[2], "%.1f") + ")";
  }
  std::string s;
  appendf(&s, "serial=%s con=%s/%s cmd=%s ota=%s tx=%llu frames=%llu crc=%llu drop=%llu uptime=%.1fs",
          serial_.isOpen() ? "up" : "down",
          link::ConSm::stateName(conSm_.state()), link::ConSm::commName(conSm_.comm()),
          cmd.c_str(), otaStateName(),
          static_cast<unsigned long long>(txFrames_),
          static_cast<unsigned long long>(st.frames),
          static_cast<unsigned long long>(st.crcErr),
          static_cast<unsigned long long>(st.drop),
          muduo::timeDifference(muduo::Timestamp::now(), startTime_));
  return s;
}

void ChassisHost::cmdStat(const muduo::net::TcpConnectionPtr& conn) {
  conn->send("ok " + statSummary() + "\n");
}

void ChassisHost::cmdCon(const muduo::net::TcpConnectionPtr& conn) {
  std::string s = "ok con state=";
  s += link::ConSm::stateName(conSm_.state());
  s += " comm=";
  s += link::ConSm::commName(conSm_.comm());
  appendf(&s, " lastFrame=");
  if (lastFrameMs_ == 0) {
    s += "never";
  } else {
    appendf(&s, "%llums-ago", steadyMs() - lastFrameMs_);
  }
  s += "\n";
  conn->send(s);
}

void ChassisHost::cmdHelp(const muduo::net::TcpConnectionPtr& conn) {
  conn->send(
      "ok 命令列表:\n"
      "  v <vx> [vy] [wz]    车体速度 m/s、rad/s（CMD_TWIST，宿主 10Hz 重发）\n"
      "  rpm <r1> [r2] [r3]  三电机输出轴转速 rpm（CMD_VEL）\n"
      "  stop                刹车并停止重发（CMD_STOP）\n"
      "  en <0|1>            电机使能（1=使能+自动建链；0=失能+拆链）\n"
      "  pid <kp> <ki> <kd>  设置 PID（CMD_PID）\n"
      "  sign <s1> <s2> <s3> 编码器方向校正 ±1（CMD_SIGN）\n"
      "  ping                测板端 RTT\n"
      "  watch [on|off]      订阅/退订 JSON 遥测流\n"
      "  flash <name.hex>    远程固件烧录（仅本机连接；文件须在固件目录内\n"
      "                      且配对 .manifest.json；完成后保持失能）\n"
      "  flash_status        烧录状态/进度\n"
      "  flash_abort [force] 取消烧录（擦写中需 force：杀进程组，板留在 BL）\n"
      "  stat | con | quit   统计 / CON 状态 / 断开\n");
}

// ================= OTA 远程烧录 =================

const char* ChassisHost::otaStateName() const {
  switch (otaState_) {
    case OtaState::kNone: return "none";
    case OtaState::kQuiescing: return "quiescing";
    case OtaState::kFlashing: return "flashing";
    case OtaState::kRecovering: return "recovering";
  }
  return "?";
}

bool ChassisHost::isLoopbackConn(const muduo::net::TcpConnectionPtr& conn) {
  const std::string ip = conn->peerAddress().toIp();
  return ip == "127.0.0.1" || ip == "::1";
}

void ChassisHost::cmdFlash(const muduo::net::TcpConnectionPtr& conn,
                           const std::vector<std::string>& tok) {
  if (tok.size() != 2) {
    conn->send("err 用法: flash <文件名.hex>（仅文件名，文件位于固件目录）\n");
    return;
  }
  if (!isLoopbackConn(conn)) {
    conn->send("err flash 仅接受本机回环连接（SSH 到车端后 nc 127.0.0.1 "
               "9000）\n");
    return;
  }
  if (otaBusy()) {
    conn->send(std::string("err flash: 已有任务进行中（") + otaStateName() +
               "），flash_status 查看\n");
    return;
  }
  if (opts_.flashDev.empty() || opts_.flashScript.empty()) {
    conn->send("err flash: 未配置烧录链路（启动参数 --flash-dev / "
               "--flash-script，见 docs/protocol-tcp.md）\n");
    return;
  }
  const std::string& name = tok[1];
  if (!isSafeFirmwareName(name)) {
    conn->send("err flash: 文件名非法（仅收固件目录内的 .hex 文件名）\n");
    return;
  }
  std::string path = opts_.firmwareDir + "/" + name;
  if (::access(path.c_str(), R_OK) != 0) {
    conn->send("err flash: 文件不存在 " + path + "\n");
    return;
  }

  // 进入 QUIESCING：清速度重发 → 粘性失能 → STOP/EN0，等板端 STATUS 确认
  otaFile_ = name;
  otaConn_ = conn;
  otaStage_ = "-";
  otaLastLine_.clear();
  otaCliLineCount_ = 0;
  otaState_ = OtaState::kQuiescing;
  otaPhaseStartMs_ = steadyMs();
  cmdMode_ = CmdMode::kNone;
  speedConn_.reset();
  userDisabled_ = true;  // 升级全程+完成后保持失能，人工 en 1 才恢复
  sendFrameToBoard(proto::PT_CMD_STOP, {});
  sendFrameToBoard(proto::PT_CMD_EN, proto::packCmdEn(false));
  conn->send("ok flash: 停车确认中（等 STATUS en=0，超时 " +
             std::to_string(opts_.quiesceTimeoutMs) + "ms）\n");
  broadcastAll("{\"t\":\"ota\",\"state\":\"quiescing\",\"file\":\"" + name + "\"}");
  LOG_INFO << "OTA 开始: " << name << "（QUIESCING）";
}

void ChassisHost::otaBeginFlasher() {
  otaState_ = OtaState::kFlashing;
  otaPhaseStartMs_ = steadyMs();
  // 板子即将复位进 bootloader：强制清 CON 现场（优雅 disconnect 会因无应答
  // 空转 2s），此后 kFlashing 期间看门狗暂停重连与静默告警
  conSm_.reset();

  if (!flasher_) flasher_ = std::make_unique<ota::OtaFlasher>(loop_);
  std::vector<std::string> argv = {
      opts_.flashPython, "-u", opts_.flashScript,
      "--flash", opts_.flashDev, opts_.firmwareDir + "/" + otaFile_,
      "--tool", opts_.flashTool};
  if (!flasher_->start(
          argv,
          [this](const std::string& line) { otaOnChildLine(line); },
          [this](bool ok, int status) { otaOnChildExit(ok, status); })) {
    otaFinish("err flash: 子进程启动失败（检查 --flash-script / 解释器）");
    return;
  }
  otaSendProgress("ok flash: 烧录子进程已启动（" + otaFile_ + "）");
  broadcastAll("{\"t\":\"ota\",\"state\":\"flashing\"}");
}

void ChassisHost::otaOnChildLine(const std::string& line) {
  otaLastLine_ = line;
  if (line.rfind("STAGE: ", 0) == 0) {
    otaStage_ = line.substr(7);
    otaCliLineCount_ = 0;
    otaSendProgress("flash [" + otaStage_ + "]");
    return;
  }
  if (line.rfind("OK: ", 0) == 0) {
    otaSendProgress("flash ok: " + line.substr(4));
    return;
  }
  if (line.rfind("FAIL: ", 0) == 0) {
    otaSendProgress("flash fail: " + line.substr(6));
    return;
  }
  if (line.rfind("cli| ", 0) == 0) {
    // 后端原始输出透传；stm32flash 逐块进度行很密，20 行放行 1 行
    if (line.find("Wrote and verified address") != std::string::npos ||
        line.find('%') != std::string::npos) {
      if (++otaCliLineCount_ % 20 != 0) return;
    }
  }
  otaSendProgress("  " + line);
}

void ChassisHost::otaOnChildExit(bool ok, int exitStatus) {
  if (otaState_ != OtaState::kFlashing) return;  // abort 已收尾
  if (ok) {
    otaSendProgress("ok flash: 擦写校验通过，复位运行中，等待 CON 重建…");
    otaState_ = OtaState::kRecovering;
    otaPhaseStartMs_ = steadyMs();
    lastRetryMs_ = 0;  // 让看门狗下一拍立即发起 CON 重连
    broadcastAll("{\"t\":\"ota\",\"state\":\"recovering\"}");
    return;
  }
  std::string reason = "status=" + std::to_string(exitStatus);
  if (WIFEXITED(exitStatus)) {
    reason = "exit=" + std::to_string(WEXITSTATUS(exitStatus));
  } else if (WIFSIGNALED(exitStatus)) {
    reason = "signal=" + std::to_string(WTERMSIG(exitStatus));
  }
  if (WIFEXITED(exitStatus) && WEXITSTATUS(exitStatus) == 6) {
    otaFinish("err flash: 烧录口被占用或不存在（" + reason +
              "），未进入 bootloader；电机保持失能。" +
              (otaLastLine_.empty() ? "" : "最后输出: " + otaLastLine_));
  } else if (otaStage_ == "check") {
    otaFinish("err flash: 镜像预检失败（" + reason +
              "），未进入 bootloader；电机保持失能。" +
              (otaLastLine_.empty() ? "" : "最后输出: " + otaLastLine_));
  } else {
    otaFinish("err flash: 烧录失败（" + reason +
              "）。板子可能停在 bootloader，可直接重试 flash。" +
              (otaLastLine_.empty() ? "" : "最后输出: " + otaLastLine_));
  }
}

void ChassisHost::otaFinish(const std::string& resultLine) {
  otaSendProgress(resultLine);
  broadcastAll("{\"t\":\"ota\",\"state\":\"done\"}");
  LOG_INFO << "OTA 结束（" << resultLine << "）";
  otaState_ = OtaState::kNone;
  otaConn_.reset();
  otaStage_ = "-";
  otaLastLine_.clear();
  // userDisabled_ 有意保持 true：升级后不自动使能、不恢复旧速度（安全约定）
}

void ChassisHost::otaSendProgress(const std::string& line) {
  if (otaConn_ && otaConn_->connected()) {
    otaConn_->send(line + "\n");
  } else {
    LOG_INFO << "OTA（发起端已断开）: " << line;
  }
}

void ChassisHost::cmdFlashStatus(const muduo::net::TcpConnectionPtr& conn) {
  std::string s = "ok flash_status state=";
  s += otaStateName();
  if (otaBusy()) {
    appendf(&s, " file=%s stage=%s child=%s",
            otaFile_.c_str(), otaStage_.c_str(),
            (flasher_ && flasher_->running()) ? "alive" : "none");
  }
  s += "\n";
  conn->send(s);
}

void ChassisHost::cmdFlashAbort(const muduo::net::TcpConnectionPtr& conn,
                                const std::vector<std::string>& tok) {
  if (!isLoopbackConn(conn)) {
    conn->send("err flash_abort 仅接受本机回环连接\n");
    return;
  }
  if (tok.size() > 2 || (tok.size() == 2 && tok[1] != "force")) {
    conn->send("err 用法: flash_abort [force]\n");
    return;
  }
  if (otaState_ == OtaState::kNone) {
    conn->send("err flash_abort: 无进行中的烧录任务\n");
  } else if (otaState_ == OtaState::kQuiescing) {
    otaFinish("ok flash_abort: 擦除尚未开始，已取消；电机保持失能");
  } else if (otaState_ == OtaState::kFlashing) {
    if (tok.size() == 2 && tok[1] == "force") {
      conn->send("ok flash_abort force: 已向进程组发 TERM（0.5s 后 KILL），"
                 "退出后自动收尾；板子留在 bootloader，可重试 flash\n");
      flasher_->killGroup();  // 子进程退出经 otaOnChildExit(ok=false) 走失败收尾
    } else {
      conn->send("err flash_abort: 擦写进行中，中止会留下半份固件"
                 "（bootloader 无恙，重烧可恢复）。确认请用: flash_abort force\n");
    }
  } else {  // kRecovering
    otaFinish("ok flash_abort: 已退出恢复等待，CON 由看门狗继续自动重建");
  }
}

// ================= 串口/协议侧 =================

void ChassisHost::onSerialData(const char* data, size_t len) {
  std::vector<proto::Frame> frames = parser_.feed(reinterpret_cast<const uint8_t*>(data), len);
  for (const auto& f : frames) {
    onFrame(f);
  }
}

void ChassisHost::onSerialState(bool up, const std::string& reason) {
  std::string js = "{\"t\":\"serial\",\"up\":";
  js += up ? "true" : "false";
  js += ",\"reason\":\"" + reason + "\"}";
  broadcastAll(js);
  if (up) lastRetryMs_ = 0;  // 设备恢复后尽快建链
}

void ChassisHost::onFrame(const proto::Frame& f) {
  lastFrameMs_ = steadyMs();
  rxSilenceWarned_ = false;
  uint64_t now = lastFrameMs_;

  switch (f.type) {
    case proto::PT_ENC_DATA:
      haveEnc_ = parseEnc(f, enc_);
      break;
    case proto::PT_IMU_DATA:
      haveImu_ = parseImu(f, imu_);
      break;
    case proto::PT_TELEMETRY:
      haveTelem_ = parseTelemetry(f, telem_);
      break;
    case proto::PT_ODOM_DATA:
      haveOdom_ = parseOdom(f, odom_);
      break;
    case proto::PT_STATUS: {
      if (!parseStatus(f, status_)) break;
      // 板重上电自愈：READY 但 en=0 → 限频补发 CMD_EN（与 bridge 行为一致；
      // 用户手动失能时让位）
      if (!userDisabled_ && conSm_.comm() == link::ConSm::COMM_READY && status_.en == 0 &&
          now - lastEnSentMs_ > opts_.enResendMs) {
        LOG_WARN << "观测到 en=0（板侧重上电？），补发 CMD_EN";
        lastEnSentMs_ = now;
        sendFrameToBoard(proto::PT_CMD_EN, proto::packCmdEn(true));
      }
      // OTA QUIESCING：板端亲口确认已失能 → 启动烧录子进程
      if (otaState_ == OtaState::kQuiescing && status_.en == 0) {
        LOG_INFO << "OTA: 板端已确认 en=0（停车确认完成）";
        otaBeginFlasher();
      }
      break;
    }
    case proto::PT_CON: {
      proto::ConPayload c;
      if (!unpackCon(f, c)) {
        LOG_WARN << "CON 载荷非法: len=" << f.len;
        break;
      }
      if (c.sender == proto::kConSenderHost) break;  // 忽略己方回显
      conSm_.onFrame(static_cast<proto::ConMsgType>(c.msgType), now);
      break;
    }
    case proto::PT_CMD_PING: {
      uint32_t tMs = 0;
      if (parsePing(f, tMs) && pingConn_ && tMs == lastPingId_) {
        double rtt = static_cast<double>(steadyMs() - lastPingSentMs_);
        pingConn_->send("pong rtt=" + jnum(rtt, "%.1f") + "ms\n");
        LOG_DEBUG << "PING RTT=" << rtt << "ms";
        pingConn_.reset();
      }
      break;
    }
    default:
      LOG_DEBUG << "未知帧类型 0x" << Fmt("%02x", f.type);
      break;
  }
}

bool ChassisHost::sendFrameToBoard(uint8_t type, const std::vector<uint8_t>& payload) {
  if (!serial_.isOpen()) {
    LOG_DEBUG << "串口未打开，丢弃 " << proto::typeName(type) << " 帧";
    return false;
  }
  std::vector<uint8_t> frame = proto::packFrame(type, payload, &txSeq_);
  serial_.send(frame.data(), frame.size());
  ++txFrames_;
  LOG_TRACE << "TX " << proto::typeName(type) << " " << payload.size() << "B";
  return true;
}

bool ChassisHost::sendConFrame(proto::ConMsgType msgType) {
  return sendFrameToBoard(proto::PT_CON, proto::packCon(msgType));
}

void ChassisHost::onCommState(link::ConSm::CommState comm) {
  LOG_INFO << "CON 通信态: " << link::ConSm::commName(comm);
  broadcastAll(std::string("{\"t\":\"con\",\"state\":\"") +
               link::ConSm::commName(comm) + "\"}");
  if (comm == link::ConSm::COMM_READY) {
    if (userDisabled_) {
      // 用户手动失能：重连后显式保持失能，而非自愈使能
      lastEnSentMs_ = steadyMs();
      sendFrameToBoard(proto::PT_CMD_EN, proto::packCmdEn(false));
    } else {
      // 建链成功即自动使能电机（与 chassis_bridge 行为一致）
      lastEnSentMs_ = steadyMs();
      sendFrameToBoard(proto::PT_CMD_EN, proto::packCmdEn(true));
    }
    if (otaState_ == OtaState::kRecovering) {
      // 烧录后固件回归、CON 重建成功：升级闭环（电机保持失能）
      otaFinish("ok flash complete: CON 已重建，电机保持失能（en 1 人工恢复）");
    }
  }
}

// ================= 周期任务 =================

void ChassisHost::onConTick() { conSm_.tick(steadyMs()); }

void ChassisHost::onRepeatTimer() {
  if (cmdMode_ == CmdMode::kNone || userDisabled_) return;
  if (conSm_.comm() != link::ConSm::COMM_READY) return;  // 门控：未 READY 不下发
  if (cmdMode_ == CmdMode::kTwist) {
    sendFrameToBoard(proto::PT_CMD_TWIST, proto::packCmdTwist(twist_[0], twist_[1], twist_[2]));
  } else {
    sendFrameToBoard(proto::PT_CMD_VEL, proto::packCmdVel(rpm_));
  }
}

void ChassisHost::onOdomTimer() {
  if (!haveOdom_ || watchers_.empty()) return;
  std::string js = "{\"t\":\"odom\"";
  appendf(&js, ",\"vx\":%.3f,\"vy\":%.3f,\"wz\":%.3f", odom_.vx, odom_.vy, odom_.wz);
  appendf(&js, ",\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f}", odom_.x, odom_.y, odom_.yaw);
  sendToWatchers(js);
}

void ChassisHost::onSlowTimer() {
  if (watchers_.empty()) return;
  if (haveEnc_) {
    std::string js = "{\"t\":\"enc\",\"count\":" + jArr3i(enc_.count) +
                     ",\"rpm\":" + jArr3(enc_.rpm, "%.1f");
    if (enc_.hasTs) js += ",\"ts\":" + std::to_string(enc_.tMs);
    sendToWatchers(js + "}");
  }
  if (haveImu_) {
    sendToWatchers("{\"t\":\"imu\",\"accel_g\":" + jArr3(imu_.accelG, "%.3f") +
                   ",\"gyro_dps\":" + jArr3(imu_.gyroDps, "%.2f") +
                   ",\"rpy_deg\":" + jArr3(imu_.rpyDeg, "%.2f") + "}");
  }
  if (haveTelem_) {
    sendToWatchers("{\"t\":\"telem\",\"tgt_rpm\":" + jArr3(telem_.tgtRpm, "%.1f") +
                   ",\"rpm\":" + jArr3(telem_.rpm, "%.1f") +
                   ",\"out_pct\":" + jArr3(telem_.outPct, "%.1f") + "}");
  }
  char buf[160];
  ::snprintf(buf, sizeof buf,
             "{\"t\":\"status\",\"uptime\":%u,\"imu_ok\":%u,\"en\":%u,\"mode\":%u,\"con\":%u}",
             status_.uptime, status_.imuOk, status_.en, status_.mode, status_.con);
  sendToWatchers(buf);
}

void ChassisHost::onWatchdogTimer() {
  uint64_t now = steadyMs();

  // 0) 串口设备恢复：未打开时周期重试（热插拔 / usbipd 重枚举自愈）
  if (!serial_.isOpen()) {
    serial_.openDevice(opts_.serialDev, opts_.serialBaud);  // 幂等，失败仅 DEBUG
  }

  // 0.5) OTA 阶段超时驱动
  if (otaState_ == OtaState::kQuiescing && now - otaPhaseStartMs_ > opts_.quiesceTimeoutMs) {
    // 未确认停车（应用离线/串口断）：复位进 BL 本身就是硬停车兜底，
    // 且"应用跑飞"时恰恰需要这条不在线也允许烧的恢复路径
    LOG_WARN << "OTA: 停车确认超时（应用离线？），复位进 BL 兜底，继续烧录";
    otaBeginFlasher();
  } else if (otaState_ == OtaState::kRecovering &&
             now - otaPhaseStartMs_ > opts_.recoverTimeoutMs) {
    otaFinish("err flash: CON 重建超时（固件未起来？）。升级状态复位，"
              "看门狗会继续自动重连；必要时重试 flash");
  }

  // 1) 板帧静默：串口开着却长时间收不到任何帧（板子死机/线缆半插）。
  //    升级期间板子在 bootloader/复位中，静默是预期，不告警
  if (otaState_ == OtaState::kNone && serial_.isOpen() && lastFrameMs_ != 0 &&
      now - lastFrameMs_ > opts_.rxTimeoutMs) {
    if (!rxSilenceWarned_) {
      rxSilenceWarned_ = true;
      LOG_WARN << "板帧静默超 " << opts_.rxTimeoutMs << "ms（CON ping 无应答将自动断链）";
      broadcastAll("{\"t\":\"board\",\"note\":\"rx-silence\"}");
    }
  }

  // 2) CON 自动重连（节流）：DISCONNECTED 且串口可用 → 再次建链。
  //    kFlashing 期间板子在 bootloader（USART3 无应答），重连只产生噪音
  if (otaState_ != OtaState::kFlashing && serial_.isOpen() &&
      conSm_.state() == link::ConSm::ST_DISCONNECTED &&
      now - lastRetryMs_ >= opts_.conRetryMs) {
    lastRetryMs_ = now;
    LOG_DEBUG << "CON 重连尝试";
    conSm_.connect(now);
  }
}

// ================= 辅助 =================

void ChassisHost::broadcastAll(const std::string& line) {
  for (const auto& conn : conns_) {
    conn->send(line + "\n");
  }
}

void ChassisHost::sendToWatchers(const std::string& line) {
  for (const auto& conn : watchers_) {
    conn->send(line + "\n");
  }
}

bool ChassisHost::parseDouble(const std::string& s, double* out) {
  if (s.empty()) return false;
  char* end = nullptr;
  double v = ::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') return false;
  *out = v;
  return true;
}

std::vector<std::string> ChassisHost::splitTokens(const std::string& line) {
  std::vector<std::string> tok;
  std::istringstream iss(line);
  std::string w;
  while (iss >> w) tok.push_back(w);
  return tok;
}

uint64_t ChassisHost::steadyMs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace app
}  // namespace mhost
