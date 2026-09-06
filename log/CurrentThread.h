#pragma once
/// @file CurrentThread.h
/// @brief 当前线程 id 缓存（避免每次日志都进系统调用）

namespace mhost {
namespace CurrentThread {

/// 返回内核线程 id（gettid），首次调用后缓存
int tid();

/// 线程 id 是否已完成缓存（调试用）
bool tidCached();

}  // namespace CurrentThread
}  // namespace mhost
