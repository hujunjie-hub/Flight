/*
 * Minimal host test support for the KF-GINS_QUAD port.
 * No external test framework: the firmware project must not add one, and the
 * evidence has to be reproducible with a plain g++ command line.
 */

#ifndef KFGINS_HOSTTEST_SUPPORT_HPP
#define KFGINS_HOSTTEST_SUPPORT_HPP

#include <cmath>
#include <cstdio>
#include <cstring>

namespace hosttest {

inline int &failureCount() {
    static int count = 0;
    return count;
}

inline int &checkCount() {
    static int count = 0;
    return count;
}

inline void reportFailure(const char *file, int line, const char *expression, double value,
                          double reference, double tolerance) {
    ++failureCount();
    std::printf("FAIL %s:%d: %s  value=%.12e reference=%.12e tolerance=%.3e\n", file, line, expression,
                value, reference, tolerance);
}

inline void reportCondition(const char *file, int line, const char *expression) {
    ++failureCount();
    std::printf("FAIL %s:%d: %s\n", file, line, expression);
}

inline int summary(const char *name) {
    std::printf("%s: %d checks, %d failures\n", name, checkCount(), failureCount());
    return (failureCount() == 0) ? 0 : 1;
}

/* Deterministic xorshift32 so that every reported number is reproducible. */
struct Rng {
    unsigned state;

    explicit Rng(unsigned seed) : state(seed ? seed : 0x9e3779b9u) {}

    unsigned next() {
        unsigned x = state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        state = x;
        return x;
    }

    /* Uniform in [-1, 1). */
    double symmetric() { return static_cast<double>(next()) / 2147483648.0 - 1.0; }
};

} // namespace hosttest

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++::hosttest::checkCount();                                                                \
        if (!(cond)) {                                                                             \
            ::hosttest::reportCondition(__FILE__, __LINE__, #cond);                               \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(value, reference, tolerance)                                                    \
    do {                                                                                           \
        ++::hosttest::checkCount();                                                                \
        const double v_ = (value);                                                                 \
        const double r_ = (reference);                                                             \
        if (!(std::fabs(v_ - r_) <= (tolerance))) {                                                \
            ::hosttest::reportFailure(__FILE__, __LINE__, #value " vs " #reference, v_, r_,         \
                                      (tolerance));                                                \
        }                                                                                          \
    } while (0)

#define CHECK_FINITE(value) CHECK(std::isfinite(value))

#endif // KFGINS_HOSTTEST_SUPPORT_HPP
