/// @file test_framecodec.cc
/// @brief 帧协议编解码单测：CRC 向量、全帧型组/解往返、坏帧容错与重同步
///
/// CRC 测试向量 "123456789" → 0x4B37 与 host_test/selftest.py 一致；
/// 解帧语义对齐 host_test/protocol.py FrameParser。

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "proto/FrameCodec.h"
#include "test_util.h"

using namespace mhost::proto;

namespace {

bool feedOne(FrameParser& p, const std::vector<uint8_t>& bytes, Frame* out) {
  auto frames = p.feed(bytes);
  if (frames.size() != 1) return false;
  *out = frames[0];
  return true;
}

void testCrc() {
  const char* vec = "123456789";
  CHECK_EQ(crc16Modbus(reinterpret_cast<const uint8_t*>(vec), 9), 0x4B37);
  CHECK_EQ(crc16Modbus(nullptr, 0), 0xFFFF);  // 空串 = 初值
  uint8_t one = 0x00;
  CHECK_EQ(crc16Modbus(&one, 1), 0x40BF);  // 0x00 单字节：8 次移位异或常数
}

void testPackParseRoundtrip() {
  uint8_t seq = 0;

  // ENC 帧
  EncData enc{{1, -2, 300000}, {12.3f, -60.5f, 0.0f}};
  std::vector<uint8_t> payload;
  for (int i = 0; i < 3; ++i) {
    uint32_t c;
    ::memcpy(&c, &enc.count[i], 4);
    payload.push_back(c & 0xFF);
    payload.push_back((c >> 8) & 0xFF);
    payload.push_back((c >> 16) & 0xFF);
    payload.push_back((c >> 24) & 0xFF);
  }
  for (int i = 0; i < 3; ++i) {
    int16_t r = static_cast<int16_t>(::lround(enc.rpm[i] * 10));
    payload.push_back(static_cast<uint8_t>(r & 0xFF));
    payload.push_back(static_cast<uint8_t>((r >> 8) & 0xFF));
  }
  auto frame = packFrame(PT_ENC_DATA, payload, &seq);
  CHECK_EQ(frame.size(), 7 + 18);

  FrameParser p;
  Frame got;
  CHECK(feedOne(p, frame, &got));
  CHECK_EQ(got.type, PT_ENC_DATA);
  CHECK_EQ(got.seq, 0);
  EncData enc2;
  CHECK(parseEnc(got, enc2));
  CHECK_EQ(enc2.count[0], 1);
  CHECK_EQ(enc2.count[1], -2);
  CHECK_EQ(enc2.count[2], 300000);
  CHECK(enc2.rpm[0] == 12.3f);
  CHECK(enc2.rpm[1] == -60.5f);

  // ODOM 帧（float32 往返）
  OdomData odom{0.25f, -0.01f, 0.5f, 1.5f, -2.25f, 3.14159f};
  auto odomBytes = packCmdTwist(odom.vx, odom.vy, odom.wz);  // 3×f32 复用
  std::vector<uint8_t> op;
  float vals[6] = {odom.vx, odom.vy, odom.wz, odom.x, odom.y, odom.yaw};
  for (float v : vals) {
    uint32_t bits;
    ::memcpy(&bits, &v, 4);
    for (int b = 0; b < 4; ++b) op.push_back((bits >> (8 * b)) & 0xFF);
  }
  auto of = packFrame(PT_ODOM_DATA, op, &seq);
  FrameParser p2;
  Frame og;
  CHECK(feedOne(p2, of, &og));
  OdomData oo;
  CHECK(parseOdom(og, oo));
  CHECK(oo.vx == odom.vx && oo.wz == odom.wz && oo.yaw == odom.yaw);
  (void)odomBytes;
}

void testPayloadCodecs() {
  // CMD_VEL：×10 取整 + 钳位
  float rpm[3] = {60.0f, -350.0f, 99999.0f};
  auto v = packCmdVel(rpm);
  CHECK_EQ(v.size(), 6);
  CHECK_EQ(v[0], 0x58);  // 600 = 0x0258
  CHECK_EQ(v[1], 0x02);
  int16_t r3 = static_cast<int16_t>(v[4] | (v[5] << 8));
  CHECK_EQ(r3, 32767);  // 钳位到 int16 上限（3276.7 rpm）
  int16_t r2 = static_cast<int16_t>(v[2] | (v[3] << 8));
  CHECK_EQ(r2, -3500);

  // CMD_TWIST / CMD_PID：float32 小端
  auto t = packCmdTwist(0.25f, 0.0f, -1.5f);
  CHECK_EQ(t.size(), 12);
  CHECK_EQ(t[0], 0x00);
  CHECK_EQ(t[1], 0x00);
  CHECK_EQ(t[2], 0x80);  // 0.25f = 0x3E800000
  CHECK_EQ(t[3], 0x3E);

  // CMD_EN / SIGN
  auto en = packCmdEn(true);
  CHECK_EQ(en.size(), 1);
  CHECK_EQ(en[0], 1);
  int8_t sg[3] = {1, -1, 1};
  auto sign = packCmdSign(sg);
  CHECK_EQ(sign[1], 0xFF);

  // PING
  auto ping = packPing(0x12345678u);
  CHECK_EQ(ping[0], 0x78);
  CHECK_EQ(ping[3], 0x12);

  // CON 载荷
  auto con = packCon(CON_ENABLE_REQUEST);
  CHECK_EQ(con.size(), 4);
  CHECK_EQ(con[0], kConSenderHost);
  CHECK_EQ(con[1], kConProtocolType);
  CHECK_EQ(con[2], 0);
  CHECK_EQ(con[3], kConProtocolVersion);
}

void testParserResync() {
  uint8_t seq = 100;
  auto f1 = packFrame(PT_STATUS, {1, 0, 1, 0, 1, 1}, &seq);
  auto f2 = packFrame(PT_CMD_STOP, {}, &seq);

  // 垃圾前缀 + 背靠背两帧
  FrameParser p;
  std::vector<uint8_t> stream = {0x00, 0xAA, 0x55, 0xFF};
  stream.insert(stream.end(), f1.begin(), f1.end());
  stream.insert(stream.end(), f2.begin(), f2.end());
  auto frames = p.feed(stream);
  CHECK_EQ(frames.size(), 2);
  CHECK_EQ(frames[0].type, PT_STATUS);
  CHECK_EQ(frames[1].type, PT_CMD_STOP);
  CHECK_EQ(p.stats().drop, 3u);       // 0x00 / 0xAA 0x55 0xFF 中的 3 字节
  CHECK_EQ(p.stats().frames, 2u);

  // 逐字节喂入（粘包/半包）
  FrameParser p2;
  int got = 0;
  for (uint8_t b : f1) {
    got += static_cast<int>(p2.feed(&b, 1).size());
  }
  CHECK_EQ(got, 1);

  // CRC 坏帧：整帧丢弃计入 crc_err，后续帧不受影响
  FrameParser p3;
  auto bad = f1;
  bad[bad.size() - 1] ^= 0xFF;  // 破坏 CRC 高字节
  std::vector<uint8_t> s2 = bad;
  s2.insert(s2.end(), f2.begin(), f2.end());
  auto out3 = p3.feed(s2);
  CHECK_EQ(out3.size(), 1);
  CHECK_EQ(out3[0].type, PT_CMD_STOP);
  CHECK_EQ(p3.stats().crcErr, 1u);

  // 伪帧头（len>32）：跳过 2 字节重找；type+len 两字节残留在流内随后被计为 drop
  FrameParser p4;
  std::vector<uint8_t> s3 = {0xAA, 0x55, 0x01, 0xFF};  // len=255 非法
  s3.insert(s3.end(), f1.begin(), f1.end());
  auto out4 = p4.feed(s3);
  CHECK_EQ(out4.size(), 1);
  CHECK_EQ(out4[0].type, PT_STATUS);
  CHECK_EQ(p4.stats().drop, 3u);  // 伪帧头 1 + 残留的 01 FF 2 字节

  // 流尾孤立 0xAA 保留
  FrameParser p5;
  std::vector<uint8_t> s4 = {0x01, 0x02, 0xAA};
  auto out5 = p5.feed(s4);
  CHECK(out5.empty());
  CHECK_EQ(p5.stats().drop, 2u);
  auto out6 = p5.feed(f1);
  CHECK_EQ(out6.size(), 1);  // 0xAA + f1 拼接后解出
}

void testStatusAndCon() {
  uint8_t seq = 7;
  auto f = packFrame(PT_STATUS, {9, 0, 1, 1, 1, 1}, &seq);
  FrameParser p;
  Frame got;
  CHECK(feedOne(p, f, &got));
  StatusData st;
  CHECK(parseStatus(got, st));
  CHECK_EQ(st.uptime, 9);
  CHECK_EQ(st.en, 1);
  CHECK_EQ(st.con, 1);
  CHECK(st.hasCon);

  // 旧固件 5B 帧（无 con 字节）
  auto f5 = packFrame(PT_STATUS, {3, 0, 1, 0, 1}, &seq);
  Frame g5;
  CHECK(feedOne(p, f5, &g5));
  StatusData st5;
  CHECK(parseStatus(g5, st5));
  CHECK(!st5.hasCon);

  // CON 校验：proto_type/ver 不符丢弃
  auto bad = packFrame(PT_CON, {2, 0x99, 0, 1}, &seq);
  Frame gb;
  CHECK(feedOne(p, bad, &gb));
  ConPayload c;
  CHECK(!unpackCon(gb, c));
  auto ok = packFrame(PT_CON, packCon(CON_PING_RESPONSE, kConSenderBoard), &seq);
  Frame go;
  CHECK(feedOne(p, ok, &go));
  CHECK(unpackCon(go, c));
  CHECK_EQ(c.msgType, CON_PING_RESPONSE);
  CHECK_EQ(c.sender, kConSenderBoard);

  // 长度不符的遥测帧拒绝
  auto shortEnc = packFrame(PT_ENC_DATA, {1, 2, 3}, &seq);
  Frame ge;
  CHECK(feedOne(p, shortEnc, &ge));
  EncData dummy;
  CHECK(!parseEnc(ge, dummy));
}

void testSysidFrames() {
  uint8_t seq = 0;
  FrameParser p;
  Frame f;

  // ENC 22B：尾部 u32 采样时刻（新固件）
  std::vector<uint8_t> ep;
  for (int i = 0; i < 3; ++i) { ep.push_back(0x11); ep.push_back(0); ep.push_back(0); ep.push_back(0); }
  for (int i = 0; i < 3; ++i) { ep.push_back(0x64); ep.push_back(0); }  // 10.0rpm
  ep.push_back(0x34); ep.push_back(0x12); ep.push_back(0); ep.push_back(0);  // t_ms=0x1234
  auto ef = packFrame(PT_ENC_DATA, ep, &seq);
  CHECK(feedOne(p, ef, &f));
  EncData enc;
  CHECK(parseEnc(f, enc));
  CHECK(enc.hasTs);
  CHECK_EQ(enc.tMs, 0x1234u);
  CHECK(enc.rpm[0] == 10.0f);

  // SYSID_META 26B
  std::vector<uint8_t> mp = {2, 0, 1, 2};
  auto f32 = [](std::vector<uint8_t>& v, float x) {
    uint32_t b;
    ::memcpy(&b, &x, 4);
    for (int i = 0; i < 4; ++i) v.push_back((b >> (8 * i)) & 0xFF);
  };
  f32(mp, 0.8f); f32(mp, 2.5f); f32(mp, 0.0f);
  mp.push_back(0x58); mp.push_back(0x02);   // amp 6.00
  mp.push_back(0xC4); mp.push_back(0x09);   // bias 25.00
  mp.push_back(0xD2); mp.push_back(0x04);   // n=1234
  mp.push_back(0x01); mp.push_back(0); mp.push_back(0); mp.push_back(0);  // t0=1
  CHECK_EQ(mp.size(), 26u);
  auto mf = packFrame(PT_SYSID_META, mp, &seq);
  CHECK(feedOne(p, mf, &f));
  SysidMeta m;
  CHECK(parseSysidMeta(f, m));
  CHECK_EQ(m.status, 2u);
  CHECK(m.kp == 0.8f && m.ki == 2.5f);
  CHECK(m.amp == 6.0f && m.bias == 25.0f);
  CHECK_EQ(m.n, 1234u);

  // SYSID_DATA：2 样本
  std::vector<uint8_t> dp = {0x08, 0x00, 2, 0};
  for (int k = 0; k < 2; ++k) {
    dp.push_back(100 + k); dp.push_back(0); dp.push_back(0); dp.push_back(0);  // t_ms
    dp.push_back(200); dp.push_back(0);    // ref 20.0
    dp.push_back(0xF0); dp.push_back(0);   // rpm 24.0
    dp.push_back(0x10); dp.push_back(0x01);  // out 27.2
  }
  auto df = packFrame(PT_SYSID_DATA, dp, &seq);
  CHECK(feedOne(p, df, &f));
  std::vector<SysidSample> ss;
  CHECK(parseSysidData(f, ss));
  CHECK_EQ(ss.size(), 2u);
  CHECK_EQ(ss[0].idx, 8u);
  CHECK_EQ(ss[1].idx, 9u);
  CHECK(ss[1].tMs == 101u);
  CHECK(ss[0].rpm == 24.0f);
  CHECK(ss[0].out == 27.2f);
}

}  // namespace

int main() {
  testCrc();
  testPackParseRoundtrip();
  testPayloadCodecs();
  testParserResync();
  testStatusAndCon();
  testSysidFrames();
  return testSummary("test_framecodec");
}
