// Minimal test harness: no third-party dependencies so the core builds anywhere.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace mptest {

struct Case {
    const char* name;
    std::function<void()> fn;
};

std::vector<Case>& registry();
extern int failures;

struct Register {
    Register(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

}  // namespace mptest

#define MP_CONCAT2(a, b) a##b
#define MP_CONCAT(a, b) MP_CONCAT2(a, b)
#define TEST(name)                                                              \
    static void name();                                                         \
    static mptest::Register MP_CONCAT(reg_, name)(#name, name);                 \
    static void name()

#define EXPECT_TRUE(cond)                                                       \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
            ++mptest::failures;                                                 \
        }                                                                       \
    } while (0)

#define EXPECT_NEAR(a, b, tol)                                                  \
    do {                                                                        \
        double va_ = (a), vb_ = (b);                                            \
        if (!(std::fabs(va_ - vb_) <= (tol))) {                                 \
            std::printf("  FAIL %s:%d: %s = %g, expected %g (tol %g)\n", __FILE__, __LINE__, #a, va_, vb_, \
                        (double)(tol));                                         \
            ++mptest::failures;                                                 \
        }                                                                       \
    } while (0)
