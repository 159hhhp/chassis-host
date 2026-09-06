#pragma once
/// @file test_util.h
/// @brief 轻量单测断言（不引第三方依赖）

#include <cstdio>
#include <string>

inline int g_checks = 0;
inline int g_failures = 0;

#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_failures;                                                              \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                \
    }                                                                            \
  } while (0)

#define CHECK_EQ(a, b)                                                           \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!((a) == (b))) {                                                         \
      ++g_failures;                                                              \
      std::printf("FAIL %s:%d: %s == %s (lhs=%s rhs=%s)\n", __FILE__, __LINE__,  \
                  #a, #b, std::to_string(a).c_str(), std::to_string(b).c_str()); \
    }                                                                            \
  } while (0)

/// 全部用例结束后调用；返回失败数（可作进程退出码）
inline int testSummary(const char* suite) {
  std::printf("%s: %d checks, %d failures\n", suite, g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
