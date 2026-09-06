#pragma once
/// @file FrameCodec.h
/// @brief 帧编解码：CRC16-MODBUS、组帧、流式解帧状态机、全部载荷编解码
///
/// 解帧语义与 host_test/protocol.py FrameParser 一致：
///  - 坏帧（CRC 错）整帧丢弃，统计 crc_err，从下一字节继续找帧头；
///  - len > 32 的伪帧头丢弃，统计 drop；
///  - 流尾孤立的 0xAA 保留（可能是下一帧的帧头首字节）。
/// 目标平台均为小端（x86-64 / 树莓派 ARM64），整数按显式字节序、
/// float 按 IEEE754 小端内存布局编解码。

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "FrameDefs.h"

namespace mhost {
namespace proto {

/// CRC16-MODBUS（多项式 0xA001，初值 0xFFFF）
uint16_t crc16Modbus(const uint8_t* data, size_t len);

/// 组帧；seq 传发送侧自增计数（模 256），payload 不得超过 kMaxPayload
std::vector<uint8_t> packFrame(uint8_t type, const uint8_t* payload, size_t len, uint8_t* seq);
inline std::vector<uint8_t> packFrame(uint8_t type, const std::vector<uint8_t>& payload,
                                      uint8_t* seq) {
  return packFrame(type, payload.data(), payload.size(), seq);
}

/// 解出的完整帧
struct Frame {
  uint8_t type = 0;
  uint8_t seq = 0;
  std::array<uint8_t, kMaxPayload> payload{};
  size_t len = 0;
};

/// 流式解帧状态机（无锁，单线程喂入）
class FrameParser {
 public:
  struct Stats {
    uint64_t frames = 0;   ///< 好帧
    uint64_t crcErr = 0;   ///< CRC 错帧
    uint64_t drop = 0;     ///< 丢弃的字节数 / 伪帧头数
  };

  /// 喂入字节流，返回本次解析出的完整帧
  std::vector<Frame> feed(const uint8_t* data, size_t len);
  std::vector<Frame> feed(const std::vector<uint8_t>& data) {
    return feed(data.data(), data.size());
  }

  const Stats& stats() const { return stats_; }

 private:
  std::vector<uint8_t> buf_;  ///< 未完成帧的字节累积
  Stats stats_;
};

// ---- 遥测载荷（板→宿主） ----

struct EncData {
  int32_t count[3];  ///< 累计计数（60000 计数/输出轴转）
  float rpm[3];      ///< 输出轴转速（由 int16 ×0.1rpm 换算）
  uint32_t tMs = 0;  ///< 板侧编码器采样时刻（1ms 时基）
  bool hasTs = false;  ///< 22B 帧带时间戳；旧固件 18B 帧无
};

struct ImuData {
  float accelG[3];   ///< 加速度（g）
  float gyroDps[3];  ///< 角速度（dps）
  float rpyDeg[3];   ///< roll/pitch/yaw（deg）
};

struct StatusData {
  uint8_t uptime;  ///< STATUS 计数（1Hz +1，模 256）
  uint8_t rsv;
  uint8_t imuOk;
  uint8_t en;    ///< 电机使能
  uint8_t mode;  ///< 0=开环 1=闭环
  uint8_t con;   ///< 板侧 CON 状态（0=NOT_READY 1=READY）
  bool hasCon;   ///< 兼容旧固件 5B 帧
};

struct TelemetryData {
  float tgtRpm[3];
  float rpm[3];
  float outPct[3];  ///< PID 输出占空比（%）
};

struct OdomData {
  float vx, vy, wz;  ///< 车体系速度（m/s、rad/s）
  float x, y, yaw;   ///< 世界系累计位姿（m、rad）
};

/// 解析失败（长度不符）返回 false，出参不动
bool parseEnc(const Frame& f, EncData& out);
bool parseImu(const Frame& f, ImuData& out);
bool parseStatus(const Frame& f, StatusData& out);
bool parseTelemetry(const Frame& f, TelemetryData& out);
bool parseOdom(const Frame& f, OdomData& out);

// ---- SYSID 采集帧（板→宿主；测量主通道在 host_test/tune_session.py，
//      此处仅保证编解码完整与日志可读） ----

struct SysidSample {
  uint16_t idx = 0;
  uint32_t tMs = 0;
  float ref = 0;   ///< 激励参考：开环为 duty%，闭环为 rpm 目标
  float rpm = 0;
  float out = 0;   ///< 实际输出 duty%
};

struct SysidMeta {
  uint8_t mode = 0, motor = 0, decim = 1, status = 0;  ///< status: 0闲/1跑/2完
  float kp = 0, ki = 0, kd = 0;
  float amp = 0, bias = 0;
  uint16_t n = 0;
  uint32_t tStartMs = 0;
};

bool parseSysidMeta(const Frame& f, SysidMeta& out);
/// 解析 SYSID_DATA（每帧最多 2 样本），追加到 out
bool parseSysidData(const Frame& f, std::vector<SysidSample>& out);

// ---- CON 载荷 ----

struct ConPayload {
  uint8_t sender;   ///< kConSenderBoard / kConSenderHost
  uint8_t msgType;  ///< ConMsgType
};

/// 解析 CON 载荷，校验 proto_type/ver 与 msg_type 合法性
bool unpackCon(const Frame& f, ConPayload& out);
/// 组 CON 载荷（宿主侧 sender 默认 0x02）
std::vector<uint8_t> packCon(uint8_t msgType, uint8_t sender = kConSenderHost);

// ---- 指令载荷（宿主→板） ----

/// 三电机目标转速（输出轴 rpm；×10 取整 int16，钳位 ±3276.7）
std::vector<uint8_t> packCmdVel(const float rpm[3]);
std::vector<uint8_t> packCmdEn(bool en);
std::vector<uint8_t> packCmdPid(float kp, float ki, float kd);
/// 编码器方向校正（非 0 视为 +1）
std::vector<uint8_t> packCmdSign(const int8_t sign[3]);
std::vector<uint8_t> packCmdTwist(float vx, float vy, float wz);
std::vector<uint8_t> packPing(uint32_t tMs);

/// 解 PING 回显载荷（板原样返回），失败返回 false
bool parsePing(const Frame& f, uint32_t& tMs);

}  // namespace proto
}  // namespace mhost
