#include "FrameCodec.h"

#include <cmath>
#include <cstring>

namespace mhost {
namespace proto {

const char* typeName(uint8_t type) {
  switch (type) {
    case PT_ENC_DATA: return "ENC";
    case PT_IMU_DATA: return "IMU";
    case PT_STATUS: return "STATUS";
    case PT_TELEMETRY: return "TELEM";
    case PT_ODOM_DATA: return "ODOM";
    case PT_CON: return "CON";
    case PT_SYSID_DATA: return "SYSID_DATA";
    case PT_SYSID_META: return "SYSID_META";
    case PT_CMD_VEL: return "CMD_VEL";
    case PT_CMD_STOP: return "CMD_STOP";
    case PT_CMD_EN: return "CMD_EN";
    case PT_CMD_PID: return "CMD_PID";
    case PT_CMD_SIGN: return "CMD_SIGN";
    case PT_CMD_TWIST: return "CMD_TWIST";
    case PT_CMD_SYSID: return "CMD_SYSID";
    case PT_CMD_SYSID_FREQS: return "CMD_SYSID_FREQS";
    case PT_CMD_PING: return "PING";
    default: return "?";
  }
}

// ---- CRC ----

uint16_t crc16Modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 1) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

// ---- 小端编解码辅助 ----

namespace {

void putU16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}

void putI16(std::vector<uint8_t>& v, int16_t x) {
  putU16(v, static_cast<uint16_t>(x));
}

void putF32(std::vector<uint8_t>& v, float x) {
  uint32_t bits;
  ::memcpy(&bits, &x, sizeof bits);  // IEEE754 小端布局直拷
  v.push_back(static_cast<uint8_t>(bits & 0xFF));
  v.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>((bits >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((bits >> 24) & 0xFF));
}

uint16_t getU16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

int16_t getI16(const uint8_t* p) {
  return static_cast<int16_t>(getU16(p));
}

uint32_t getU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int32_t getI32(const uint8_t* p) { return static_cast<int32_t>(getU32(p)); }

float getF32(const uint8_t* p) {
  uint32_t bits = getU32(p);
  float x;
  ::memcpy(&x, &bits, sizeof x);
  return x;
}

}  // namespace

// ---- 组帧 ----

std::vector<uint8_t> packFrame(uint8_t type, const uint8_t* payload, size_t len, uint8_t* seq) {
  std::vector<uint8_t> out;
  out.reserve(kFrameOverhead + len);
  out.push_back(kHead0);
  out.push_back(kHead1);
  out.push_back(type);
  out.push_back(static_cast<uint8_t>(len));
  out.push_back(seq != nullptr ? (*seq)++ : 0);
  out.insert(out.end(), payload, payload + len);
  // CRC 覆盖 type+len+seq+payload（out[2] 起）
  uint16_t crc = crc16Modbus(out.data() + 2, 3 + len);
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return out;
}

// ---- 解帧状态机（语义对齐 protocol.py FrameParser） ----

std::vector<Frame> FrameParser::feed(const uint8_t* data, size_t len) {
  std::vector<Frame> out;
  buf_.insert(buf_.end(), data, data + len);

  for (;;) {
    // 1) 找帧头
    size_t idx = buf_.size();
    for (size_t i = 0; i + 1 < buf_.size(); ++i) {
      if (buf_[i] == kHead0 && buf_[i + 1] == kHead1) {
        idx = i;
        break;
      }
    }
    if (idx == buf_.size()) {
      // 无帧头：保留流尾孤立的 0xAA（可能是下一帧的首字节）
      size_t keep = (!buf_.empty() && buf_.back() == kHead0) ? 1 : 0;
      if (buf_.size() > keep) {
        stats_.drop += buf_.size() - keep;
        buf_.erase(buf_.begin(), buf_.end() - static_cast<long>(keep));
      }
      return out;
    }
    if (idx > 0) {
      stats_.drop += idx;
      buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(idx));
    }

    // 2) type+len+seq 是否到齐
    if (buf_.size() < 5) return out;
    uint8_t type = buf_[2];
    size_t plen = buf_[3];

    // 3) 长度合法性：>32 为伪帧头，跳过这 2 字节继续找
    if (plen > kMaxPayload) {
      stats_.drop += 1;
      buf_.erase(buf_.begin(), buf_.begin() + 2);
      continue;
    }

    // 4) 整帧是否到齐
    size_t flen = kFrameOverhead + plen;
    if (buf_.size() < flen) return out;

    Frame f;
    f.type = type;
    f.seq = buf_[4];
    f.len = plen;
    ::memcpy(f.payload.data(), buf_.data() + 5, plen);

    // 5) CRC（覆盖 type+len+seq+payload）
    uint16_t crcRx = static_cast<uint16_t>(buf_[5 + plen] | (buf_[6 + plen] << 8));
    uint16_t crcCalc = crc16Modbus(buf_.data() + 2, 3 + plen);
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(flen));

    if (crcCalc == crcRx) {
      stats_.frames += 1;
      out.push_back(std::move(f));
    } else {
      stats_.crcErr += 1;  // 坏帧整帧丢弃，继续找下一帧头
    }
  }
}

// ---- 遥测解析 ----

bool parseEnc(const Frame& f, EncData& out) {
  if (f.len != 18 && f.len != 22) return false;
  const uint8_t* p = f.payload.data();
  for (int i = 0; i < 3; ++i) {
    out.count[i] = getI32(p + 4 * i);
    out.rpm[i] = static_cast<float>(getI16(p + 12 + 2 * i)) / 10.0f;
  }
  out.hasTs = (f.len == 22);
  out.tMs = out.hasTs ? getU32(p + 18) : 0;
  return true;
}

bool parseImu(const Frame& f, ImuData& out) {
  if (f.len != 18) return false;
  const uint8_t* p = f.payload.data();
  for (int i = 0; i < 3; ++i) {
    out.accelG[i] = static_cast<float>(getI16(p + 2 * i)) / 1000.0f;
    out.gyroDps[i] = static_cast<float>(getI16(p + 6 + 2 * i)) / 100.0f;
    out.rpyDeg[i] = static_cast<float>(getI16(p + 12 + 2 * i)) / 100.0f;
  }
  return true;
}

bool parseStatus(const Frame& f, StatusData& out) {
  if (f.len < 5) return false;
  out.uptime = f.payload[0];
  out.rsv = f.payload[1];
  out.imuOk = f.payload[2];
  out.en = f.payload[3];
  out.mode = f.payload[4];
  if (f.len > 5) {
    out.con = f.payload[5];
    out.hasCon = true;
  } else {
    out.con = 0;
    out.hasCon = false;
  }
  return true;
}

bool parseTelemetry(const Frame& f, TelemetryData& out) {
  if (f.len != 18) return false;
  const uint8_t* p = f.payload.data();
  for (int i = 0; i < 3; ++i) {
    out.tgtRpm[i] = static_cast<float>(getI16(p + 2 * i)) / 10.0f;
    out.rpm[i] = static_cast<float>(getI16(p + 6 + 2 * i)) / 10.0f;
    out.outPct[i] = static_cast<float>(getI16(p + 12 + 2 * i)) / 10.0f;
  }
  return true;
}

bool parseOdom(const Frame& f, OdomData& out) {
  if (f.len != 24) return false;
  const uint8_t* p = f.payload.data();
  out.vx = getF32(p);
  out.vy = getF32(p + 4);
  out.wz = getF32(p + 8);
  out.x = getF32(p + 12);
  out.y = getF32(p + 16);
  out.yaw = getF32(p + 20);
  return true;
}

bool parseSysidMeta(const Frame& f, SysidMeta& out) {
  if (f.len != 26) return false;
  const uint8_t* p = f.payload.data();
  out.mode = p[0];
  out.motor = p[1];
  out.decim = p[2];
  out.status = p[3];
  out.kp = getF32(p + 4);
  out.ki = getF32(p + 8);
  out.kd = getF32(p + 12);
  out.amp = static_cast<float>(getI16(p + 16)) / 100.0f;
  out.bias = static_cast<float>(getI16(p + 18)) / 100.0f;
  out.n = getU16(p + 20);
  out.tStartMs = getU32(p + 22);
  return true;
}

bool parseSysidData(const Frame& f, std::vector<SysidSample>& out) {
  if (f.len < 14 || (f.len - 4) % 10 != 0) return false;
  uint16_t first = getU16(f.payload.data());
  uint8_t cnt = f.payload[2];
  if (cnt != (f.len - 4) / 10) return false;
  for (uint8_t i = 0; i < cnt; ++i) {
    const uint8_t* p = f.payload.data() + 4 + 10 * i;
    SysidSample s;
    s.idx = static_cast<uint16_t>(first + i);
    s.tMs = getU32(p);
    s.ref = static_cast<float>(getI16(p + 4)) / 10.0f;
    s.rpm = static_cast<float>(getI16(p + 6)) / 10.0f;
    s.out = static_cast<float>(getI16(p + 8)) / 10.0f;
    out.push_back(s);
  }
  return true;
}

// ---- CON ----

bool unpackCon(const Frame& f, ConPayload& out) {
  if (f.len != kConPayloadSize || f.payload[1] != kConProtocolType ||
      f.payload[2] > CON_PING_RESPONSE || f.payload[3] != kConProtocolVersion) {
    return false;
  }
  out.sender = f.payload[0];
  out.msgType = f.payload[2];
  return true;
}

std::vector<uint8_t> packCon(uint8_t msgType, uint8_t sender) {
  return {sender, kConProtocolType, static_cast<uint8_t>(msgType & 0xFF), kConProtocolVersion};
}

// ---- 指令组包 ----

std::vector<uint8_t> packCmdVel(const float rpm[3]) {
  std::vector<uint8_t> v;
  v.reserve(6);
  for (int i = 0; i < 3; ++i) {
    float r = rpm[i];
    if (r > 3276.7f) r = 3276.7f;
    if (r < -3276.8f) r = -3276.8f;
    putI16(v, static_cast<int16_t>(::lround(r * 10.0f)));
  }
  return v;
}

std::vector<uint8_t> packCmdEn(bool en) {
  return {static_cast<uint8_t>(en ? 1 : 0)};
}

std::vector<uint8_t> packCmdPid(float kp, float ki, float kd) {
  std::vector<uint8_t> v;
  v.reserve(12);
  putF32(v, kp);
  putF32(v, ki);
  putF32(v, kd);
  return v;
}

std::vector<uint8_t> packCmdSign(const int8_t sign[3]) {
  return {static_cast<uint8_t>(sign[0] >= 0 ? 1 : 0xFF),
          static_cast<uint8_t>(sign[1] >= 0 ? 1 : 0xFF),
          static_cast<uint8_t>(sign[2] >= 0 ? 1 : 0xFF)};
}

std::vector<uint8_t> packCmdTwist(float vx, float vy, float wz) {
  std::vector<uint8_t> v;
  v.reserve(12);
  putF32(v, vx);
  putF32(v, vy);
  putF32(v, wz);
  return v;
}

std::vector<uint8_t> packPing(uint32_t tMs) {
  std::vector<uint8_t> v;
  v.reserve(4);
  v.push_back(static_cast<uint8_t>(tMs & 0xFF));
  v.push_back(static_cast<uint8_t>((tMs >> 8) & 0xFF));
  v.push_back(static_cast<uint8_t>((tMs >> 16) & 0xFF));
  v.push_back(static_cast<uint8_t>((tMs >> 24) & 0xFF));
  return v;
}

bool parsePing(const Frame& f, uint32_t& tMs) {
  if (f.len != 4) return false;
  tMs = getU32(f.payload.data());
  return true;
}

}  // namespace proto
}  // namespace mhost
