# chassis-host —— 轻量级 C30D 底盘上位机（muduo Reactor + 串口挂 epoll + 双缓冲日志）
cmake_minimum_required(VERSION 3.13)
project(chassis-host C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Werror)

find_package(Threads REQUIRED)

# ---- muduo 库定位：优先工程内 build/muduo-install（scripts/build_muduo.sh 产物），
#      其次 /usr/local（源码 make install 的默认前缀） ----
set(MUDUO_ROOT "" CACHE PATH "muduo 安装前缀（含 lib/ 与 include/）")
if(NOT MUDUO_ROOT)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/build/muduo-install/lib/libmuduo_net.a")
    set(MUDUO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/build/muduo-install")
  else()
    set(MUDUO_ROOT "/usr/local")
  endif()
endif()
set(MUDUO_INCLUDE_DIR "${MUDUO_ROOT}/include")
set(MUDUO_LIBS
    "${MUDUO_ROOT}/lib/libmuduo_net.a"
    "${MUDUO_ROOT}/lib/libmuduo_base.a")
message(STATUS "muduo: ${MUDUO_ROOT}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${MUDUO_INCLUDE_DIR})

# ---- 自研双缓冲异步日志库（无 muduo 依赖，可独立复用） ----
add_library(mhost_log STATIC
    log/Timestamp.cc
    log/LogStream.cc
    log/Logger.cc
    log/FileUtil.cc
    log/LogFile.cc
    log/AsyncLogging.cc)
target_link_libraries(mhost_log PUBLIC Threads::Threads)

# ---- 帧协议（纯 C++17，无 muduo 依赖，可单测） ----
add_library(mhost_proto STATIC
    proto/FrameCodec.cc)

# ---- CON 连接状态机 ----
add_library(mhost_link STATIC
    link/ConSm.cc)
target_link_libraries(mhost_link PUBLIC mhost_proto mhost_log)

# ---- 串口（tty fd 挂 muduo epoll） ----
add_library(mhost_serial STATIC
    serial/SerialPort.cc)
target_link_libraries(mhost_serial PUBLIC mhost_log ${MUDUO_LIBS} Threads::Threads)

# ---- 主程序 ----
add_executable(chassis_host
    app/ChassisHost.cc
    app/main.cc)
target_link_libraries(chassis_host
    mhost_serial mhost_link mhost_proto mhost_log
    ${MUDUO_LIBS} Threads::Threads)

# ---- 单元测试与性能基准 ----
enable_testing()
add_executable(test_framecodec tests/test_framecodec.cc)
target_link_libraries(test_framecodec mhost_proto)
add_test(NAME framecodec COMMAND test_framecodec)

add_executable(test_consm tests/test_consm.cc)
target_link_libraries(test_consm mhost_link mhost_proto)
add_test(NAME consm COMMAND test_consm)

add_executable(log_bench tests/log_bench.cc)
target_link_libraries(log_bench mhost_log Threads::Threads)

# ---- 模拟固件（pty，用于无硬件联调） ----
add_executable(mock_board tools/mock_board.cc)
target_link_libraries(mock_board mhost_proto mhost_log Threads::Threads)
