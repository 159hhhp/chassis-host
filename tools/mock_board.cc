/// @file mock_board.cc
/// @brief C30D 固件模拟器（无硬件联调用），移植自
///        C30D_Chassis/host_test/mock_board.py，行为与固件对齐：
///        20ms 节拍 ENC/IMU/ODOM + 1Hz STATUS；CMD_VEL/CMD_TWIST PI 闭环
///        与 500ms 超时；CMD_EN/STOP/PID/PING（原样回显）；CON 服务端
///        两态 + 3s 宿主活动看门狗（ping 隐式重建链）。
///
/// 用法:
///   mock_board --pty [--link /tmp/mhost_mock_tty]
///     创建伪终端打印从设备路径；--link 同时建立指向它的符号链接，
///     mock 重启后重建链接，宿主以固定路径打开即可测试设备消失自愈。
///   mock_board --port /dev/ttyUSB0 [--baud 115200]
///     直接使用真实串口（裸机半联调）。

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <time.h>

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <vector>

#include "proto/FrameCodec.h"
#include "log/Logger.h"

using namespace mhost;
using mhost::proto::FrameParser;

namespace {

std::atomic<bool> g_quit{false};
void onSignal(int) { g_quit.store(true); }

/// 三全向轮运动学（与固件 App/kinematics.c 同参数同约定）
///   逆解: v_i = -vx·sinθi + vy·cosθi + wz·R ；rpm = v·60/(2πr)
///   正解: 克莱姆法则解 3×3 方程组（安装角 90/210/330°+π 偏移）
class Kinematics {
 public:
  Kinematics() : wheelR_(0.030f), robotR_(0.055f), offset_(3.14159265f) {
    const float deg[3] = {90.0f, 210.0f, 330.0f};
    for (int i = 0; i < 3; ++i) {
      float a = deg[i] * 0.01745329f + offset_;
      sin_[i] = ::sinf(a);
      cos_[i] = ::cosf(a);
    }
  }
  float msToRpm(float v) const { return v * 60.0f / (6.2831853f * wheelR_); }
  float rpmToMs(float rpm) const { return rpm / 60.0f * 6.2831853f * wheelR_; }
  void twistToWheels(float vx, float vy, float wz, float rpm[3]) const {
    for (int i = 0; i < 3; ++i) {
      rpm[i] = msToRpm(-vx * sin_[i] + vy * cos_[i] + wz * robotR_);
    }
  }
  void wheelsToTwist(const float rpm[3], float* vx, float* vy, float* wz) const {
    float v[3] = {rpmToMs(rpm[0]), rpmToMs(rpm[1]), rpmToMs(rpm[2])};
    float a[3][3] = {{-sin_[0], cos_[0], robotR_},
                     {-sin_[1], cos_[1], robotR_},
                     {-sin_[2], cos_[2], robotR_}};
    float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (::fabsf(det) < 1e-9f) {
      *vx = *vy = *wz = 0.0f;
      return;
    }
    float x[3];
    for (int j = 0; j < 3; ++j) {
      float m[3][3];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          m[r][c] = (c == j) ? v[r] : a[r][c];
        }
      }
      x[j] = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
              m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
              m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det;
    }
    *vx = x[0];
    *vy = x[1];
    *wz = x[2];
  }

 private:
  float wheelR_, robotR_, offset_;
  float sin_[3], cos_[3];
};

/// 单电机近似模型：PI + 一阶惯性（参数与 mock_board.py 一致）
struct MotorSim {
  float kp = 0.5f;          // %/rpm
  float ki = 2.0f;          // %/(rpm·s)
  const float kRpmPerPct = 3.2f;
  const float tauS = 0.25f;
  float target = 0.0f;
  float speed = 0.0f;       // rpm
  int32_t count = 0;        // 60000 计数/输出轴转
  float integral = 0.0f;

  void tick(float dt) {
    float err = target - speed;
    float out = kp * err + integral;
    if (out > 100.0f) out = 100.0f;
    if (out < -100.0f) out = -100.0f;
    if ((out > -100.0f && out < 100.0f) || err * out <= 0) {
      integral += ki * err * dt;
      if (integral > 100.0f) integral = 100.0f;
      if (integral < -100.0f) integral = -100.0f;
    }
    float steady = out * kRpmPerPct;
    float k = 1.0f - ::expf(-dt / tauS);
    speed += (steady - speed) * k;
    count += static_cast<int32_t>(speed * 60000.0f / 60.0f * dt);
  }
};

/// 固件模拟器：feed(宿主→板) / pump(板→宿主)
class MockBoard {
 public:
  void feed(const uint8_t* data, size_t len) {
    for (const auto& f : parser_.feed(data, len)) {
      handle(f);
    }
  }

  /// 推进仿真并返回待发字节（20ms 节拍）
  std::vector<uint8_t> pump() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    double now = static_cast<double>(ts.tv_sec) + ts.tv_nsec * 1e-9;
    double dt = now - lastTx_;
    if (dt < 0.02) return {};

    // 速度指令 500ms 超时（speed_ctrl 一致）
    if (mode_ == 1 && now - lastCmd_ > 0.5) {
      for (auto& m : motors_) m.target = 0.0f;
    }
    // CON 宿主活动看门狗（con.c 一致：3s）
    if (conReady_ && lastCon_ > 0 && now - lastCon_ > 3.0) {
      conReady_ = false;
      LOG_WARN << "[mock] CON 看门狗超时（3s 无宿主 CON 帧）→ 断链";
    }

    // 电机以 5ms 步长推进
    int steps = static_cast<int>(dt / 0.005 + 0.5);
    if (steps < 1) steps = 1;
    float sub = static_cast<float>(dt / steps);
    for (int s = 0; s < steps; ++s) {
      for (auto& m : motors_) m.tick(sub);
    }

    lastTx_ = now;
    ++txCount_;

    // ENC 帧
    std::vector<uint8_t> enc;
    for (int i = 0; i < 3; ++i) {
      uint32_t c;
      ::memcpy(&c, &motors_[i].count, 4);
      for (int b = 0; b < 4; ++b) enc.push_back((c >> (8 * b)) & 0xFF);
    }
    for (int i = 0; i < 3; ++i) {
      int16_t r = static_cast<int16_t>(::lroundf(motors_[i].speed * 10.0f));
      enc.push_back(static_cast<uint8_t>(r & 0xFF));
      enc.push_back(static_cast<uint8_t>((r >> 8) & 0xFF));
    }
    send(proto::PT_ENC_DATA, enc);

    // IMU 帧（gz 以双轮差速近似）
    float gz = (motors_[1].speed - motors_[0].speed) * 0.01f;
    yaw_ += gz * 0.02f;
    std::vector<uint8_t> imu;
    int16_t imuRaw[9] = {0, 0, 1000, 0, 0, static_cast<int16_t>(::lroundf(gz * 100)), 0, 0,
                         static_cast<int16_t>(::lroundf(yaw_ * 100))};
    for (int16_t v : imuRaw) {
      imu.push_back(static_cast<uint8_t>(v & 0xFF));
      imu.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    }
    send(proto::PT_IMU_DATA, imu);

    // ODOM 帧（50Hz）：正解三轮实测转速 → 车体速度，积分得世界系位姿
    float rpmNow[3] = {motors_[0].speed, motors_[1].speed, motors_[2].speed};
    float vx, vy, wz;
    kin_.wheelsToTwist(rpmNow, &vx, &vy, &wz);
    x_ += vx * 0.02f;
    y_ += vy * 0.02f;
    yawOdom_ += wz * 0.02f;
    std::vector<uint8_t> odom;
    const float odomVals[6] = {vx, vy, wz, x_, y_, yawOdom_};
    for (float v : odomVals) {
      uint32_t bits;
      ::memcpy(&bits, &v, 4);
      for (int b = 0; b < 4; ++b) odom.push_back((bits >> (8 * b)) & 0xFF);
    }
    send(proto::PT_ODOM_DATA, odom);

    // STATUS 帧 1Hz（6B，末字节 con）
    if (txCount_ % 50 == 0) {
      send(proto::PT_STATUS,
           {static_cast<uint8_t>(txCount_ / 50 % 256), 0, 1,
            static_cast<uint8_t>(driverEn_ ? 1 : 0), static_cast<uint8_t>(mode_),
            static_cast<uint8_t>(conReady_ ? 1 : 0)});
    }

    std::vector<uint8_t> out = txBuf_;
    txBuf_.clear();
    return out;
  }

  bool conReady() const { return conReady_; }

 private:
  void handle(const proto::Frame& f) {
    switch (f.type) {
      case proto::PT_CMD_VEL: {
        if (f.len != 6) return;
        for (int i = 0; i < 3; ++i) {
          int16_t raw = static_cast<int16_t>(f.payload[2 * i] | (f.payload[2 * i + 1] << 8));
          float t = raw / 10.0f;
          if (t > 350.0f) t = 350.0f;
          if (t < -350.0f) t = -350.0f;
          if (t * motors_[i].speed < 0) motors_[i].integral = 0.0f;
          motors_[i].target = t;
        }
        mode_ = 1;
        lastCmd_ = monoNow();
        break;
      }
      case proto::PT_CMD_STOP:
        mode_ = 0;
        for (auto& m : motors_) {
          m.target = 0.0f;
          m.speed = 0.0f;
          m.integral = 0.0f;
        }
        LOG_INFO << "[mock] CMD_STOP";
        break;
      case proto::PT_CMD_EN:
        if (f.len != 1) return;
        driverEn_ = f.payload[0] == 1;
        if (!driverEn_) {
          mode_ = 0;
          for (auto& m : motors_) m.target = 0.0f;
        }
        LOG_INFO << "[mock] CMD_EN en=" << (driverEn_ ? 1 : 0);
        break;
      case proto::PT_CMD_PID:
        if (f.len != 12) return;
        ::memcpy(&kp_, f.payload.data(), 4);
        ::memcpy(&ki_, f.payload.data() + 4, 4);
        for (auto& m : motors_) {
          m.kp = kp_;
          m.ki = ki_;
        }
        LOG_INFO << "[mock] CMD_PID kp=" << kp_ << " ki=" << ki_;
        break;
      case proto::PT_CMD_SIGN:
        break;  // 模拟器不做方向翻转
      case proto::PT_CMD_PING:
        send(proto::PT_CMD_PING,
             std::vector<uint8_t>(f.payload.begin(), f.payload.begin() + f.len));
        break;
      case proto::PT_CMD_TWIST: {
        // 逆解到三轮目标转速（与固件 chassis_ctrl 路径等价），钳位 ±350rpm
        if (f.len != 12) return;
        float vx, vy, wz;
        ::memcpy(&vx, f.payload.data(), 4);
        ::memcpy(&vy, f.payload.data() + 4, 4);
        ::memcpy(&wz, f.payload.data() + 8, 4);
        float rpm[3];
        kin_.twistToWheels(vx, vy, wz, rpm);
        for (int i = 0; i < 3; ++i) {
          float t = rpm[i];
          if (t > 350.0f) t = 350.0f;
          if (t < -350.0f) t = -350.0f;
          if (t * motors_[i].speed < 0) motors_[i].integral = 0.0f;
          motors_[i].target = t;
        }
        mode_ = 1;
        lastCmd_ = monoNow();
        LOG_INFO << "[mock] CMD_TWIST vx=" << vx << " wz=" << wz;
        break;
      }
      case proto::PT_CON: {
        proto::ConPayload c;
        if (!proto::unpackCon(f, c) || c.sender != proto::kConSenderHost) return;
        double now = monoNow();
        if (c.msgType == proto::CON_ENABLE_REQUEST) {
          sendCon(proto::CON_ENABLE_RESPONSE);
          conReady_ = true;
          lastCon_ = now;
        } else if (c.msgType == proto::CON_DISABLE_REQUEST) {
          sendCon(proto::CON_DISABLE_RESPONSE);
          conReady_ = false;
        } else if (c.msgType == proto::CON_PING_REQUEST) {
          sendCon(proto::CON_PING_RESPONSE);
          conReady_ = true;  // 隐式重建链（con.c 一致）
          lastCon_ = now;
        }
        break;
      }
      default:
        break;
    }
  }

  void sendCon(proto::ConMsgType mt) {
    send(proto::PT_CON, proto::packCon(mt, proto::kConSenderBoard));
  }

  void send(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame = proto::packFrame(type, payload, &seq_);
    txBuf_.insert(txBuf_.end(), frame.begin(), frame.end());
  }

  static double monoNow() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + ts.tv_nsec * 1e-9;
  }

  FrameParser parser_;
  MotorSim motors_[3];
  Kinematics kin_;
  float kp_ = 0.5f, ki_ = 2.0f;
  bool driverEn_ = false;
  int mode_ = 0;
  float yaw_ = 0.0f;
  float x_ = 0.0f, y_ = 0.0f, yawOdom_ = 0.0f;
  bool conReady_ = false;
  uint8_t seq_ = 0;
  uint64_t txCount_ = 0;
  double lastCmd_ = 0.0;
  double lastTx_ = 0.0;
  double lastCon_ = 0.0;
  std::vector<uint8_t> txBuf_;
};

}  // namespace

int main(int argc, char* argv[]) {
  std::string port, linkPath;
  int baud = 115200;
  bool usePty = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--pty") {
      usePty = true;
    } else if (a == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (a == "--baud" && i + 1 < argc) {
      baud = ::atoi(argv[++i]);
    } else if (a == "--link" && i + 1 < argc) {
      linkPath = argv[++i];
    } else {
      ::fprintf(stderr, "用法: mock_board --pty [--link PATH] | --port DEV [--baud N]\n");
      return 1;
    }
  }
  ::signal(SIGINT, onSignal);
  ::signal(SIGTERM, onSignal);
  ::signal(SIGPIPE, SIG_IGN);

  int fd = -1;
  if (usePty) {
    // 打开伪终端主设备（不经 termios——宿主侧自会配置从设备）
    fd = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (fd < 0 || ::grantpt(fd) != 0 || ::unlockpt(fd) != 0) {
      ::perror("posix_openpt");
      return 1;
    }
    std::string slave = ::ptsname(fd);
    if (!linkPath.empty()) {
      ::unlink(linkPath.c_str());
      if (::symlink(slave.c_str(), linkPath.c_str()) != 0) {
        ::perror("symlink");
        return 1;
      }
      ::printf("[mock] pty: %s (link: %s)\n", slave.c_str(), linkPath.c_str());
    } else {
      ::printf("[mock] pty: %s\n", slave.c_str());
    }
  } else if (!port.empty()) {
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      ::perror("open serial");
      return 1;
    }
    // 与宿主一致的原始 8N1 配置
    struct termios tty;
    ::tcgetattr(fd, &tty);
    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, B115200);
    ::cfsetospeed(&tty, B115200);
    ::tcsetattr(fd, TCSANOW, &tty);
    (void)baud;
    ::printf("[mock] serial: %s\n", port.c_str());
  } else {
    ::fprintf(stderr, "用法: mock_board --pty [--link PATH] | --port DEV [--baud N]\n");
    return 1;
  }
  ::fflush(stdout);

  MockBoard board;
  struct pollfd pfd = {fd, POLLIN, 0};
  uint8_t buf[4096];

  while (!g_quit.load()) {
    std::vector<uint8_t> out = board.pump();
    if (!out.empty() && ::write(fd, out.data(), out.size()) < 0) {
      // 宿主未打开从设备时 pty 写入会 EAGAIN/EIO，忽略
    }
    int r = ::poll(&pfd, 1, 5);
    if (r > 0 && (pfd.revents & POLLIN)) {
      ssize_t n = ::read(fd, buf, sizeof buf);
      if (n > 0) {
        board.feed(buf, static_cast<size_t>(n));
      }
    }
    if (pfd.revents & (POLLHUP | POLLERR)) {
      // pty 从端（宿主侧）未打开：主端持续 HUP，睡一拍避免 poll 自旋；
      // 宿主重开后 HUP 自动消失
      struct timespec ts = {0, 20 * 1000 * 1000};
      ::nanosleep(&ts, nullptr);
    }
  }

  ::printf("[mock] 退出\n");
  if (!linkPath.empty()) ::unlink(linkPath.c_str());
  ::close(fd);
  return 0;
}
