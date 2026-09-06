#pragma once
/// @file FrameDefs.h
/// @brief C30D 帧协议常量与命令字定义
///
/// 单一事实来源为固件 C30D_Chassis/App/proto.h（同步修改），
/// 本文件与其、以及 host_test/protocol.py 逐字节一致。
///
/// 帧格式（小端）：
/// @verbatim
///   AA 55 | type(1B) | len(1B) | seq(1B) | payload(len B) | crc16_lo crc16_hi
/// @endverbatim
/// CRC16-MODBUS（0xA001/0xFFFF）覆盖 type+len+seq+payload，不含帧头。

#include <cstddef>
#include <cstdint>

namespace mhost {
namespace proto {

constexpr uint8_t kHead0 = 0xAA;
constexpr uint8_t kHead1 = 0x55;
constexpr size_t kFrameOverhead = 7;  ///< 帧头2 + type1 + len1 + seq1 + crc2
constexpr size_t kMaxPayload = 32;    ///< payload 上限（>32 视为伪帧头丢弃）
constexpr size_t kMaxFrame = kFrameOverhead + kMaxPayload;

/// 命令字：bit7=0 板→宿主（遥测），bit7=1 宿主→板（指令）
enum FrameType : uint8_t {
  PT_ENC_DATA = 0x01,   ///< 板→宿主 22B：int32 count[3] + int16 rpm_x10[3] + u32 t_ms
  PT_IMU_DATA = 0x02,   ///< 板→宿主 18B：int16 accel_mg[3] + gyro_x100[3] + rpy_x100[3]
  PT_STATUS = 0x03,     ///< 板→宿主 6B：uptime/rsv/imu_ok/en/mode/con
  PT_TELEMETRY = 0x04,  ///< 板→宿主 18B：int16 tgt/rpm/out ×x10（shell tel 开启）
  PT_ODOM_DATA = 0x05,  ///< 板→宿主 24B：float vx,vy,wz,x,y,yaw
  PT_CON = 0x06,        ///< 双向 4B：链路连接状态机
  PT_SYSID_DATA = 0x07, ///< 板→宿主 4+10n B：sysid 环形缓冲分帧回传（App/sysid.h）
  PT_SYSID_META = 0x08, ///< 板→宿主 26B：采集起止通告（含板上真实 PID 增益）
  PT_CMD_VEL = 0x81,    ///< 宿主→板 6B：int16 rpm_x10[3]（500ms 超时）
  PT_CMD_STOP = 0x82,   ///< 宿主→板 0B：刹车 + 退出闭环
  PT_CMD_EN = 0x83,     ///< 宿主→板 1B：u8 en（仅 1 使能）
  PT_CMD_PID = 0x84,    ///< 宿主→板 12B：float kp,ki,kd
  PT_CMD_SIGN = 0x85,   ///< 宿主→板 3B：int8 sign[3]
  PT_CMD_TWIST = 0x86,  ///< 宿主→板 12B：float vx,vy,wz（500ms 超时）
  PT_CMD_SYSID = 0x87,  ///< 宿主→板 14B：sysid 启动/停止/回传（host_test/tune_session.py）
  PT_CMD_SYSID_FREQS = 0x88,   ///< 宿主→板 2+2n B：扫频频率表（厘米Hz）
  PT_CMD_PING = 0x8F,   ///< 宿主→板 4B：u32 t_ms，板原样回显
};

/// 命令字短名（日志与 TCP stat 用），未知返回 "?"
const char* typeName(uint8_t type);

// ---- CON 帧（0x06）载荷 4B：[sender, proto_type, msg_type, proto_ver] ----

constexpr uint8_t kConSenderBoard = 0x01;   ///< 服务端（板）
constexpr uint8_t kConSenderHost = 0x02;    ///< 客户端（宿主，本程序）
constexpr uint8_t kConProtocolType = 0x03;  ///< 不符则丢帧
constexpr uint8_t kConProtocolVersion = 0x01;
constexpr size_t kConPayloadSize = 4;

/// CON 消息类型（与固件 App/con.h 的 con_msg_type_t 一致）
enum ConMsgType : uint8_t {
  CON_ENABLE_REQUEST = 0,    ///< 宿主→板，请求建链
  CON_ENABLE_RESPONSE = 1,   ///< 板→宿主
  CON_DISABLE_REQUEST = 2,   ///< 宿主→板，请求拆链
  CON_DISABLE_RESPONSE = 3,  ///< 板→宿主
  CON_PING_REQUEST = 4,      ///< 宿主→板，保活（喂板侧 3s 看门狗）
  CON_PING_RESPONSE = 5,     ///< 板→宿主
};

}  // namespace proto
}  // namespace mhost
