/*
 * KF-GINS_QUAD IMU increment builder.
 *
 * spec 05 §3.1: the INS mechanisation consumes increments, not instantaneous
 * rates.  The base integration is a fixed trapezoid over consecutive valid
 * samples with the real FCU-monotonic sample time; the first sample is only a
 * seed.  Missing, duplicated, reversed or clearly non-uniform samples must not
 * be turned into a fabricated continuous increment.
 *
 * The builder is deliberately free of RTOS, HAL and logging dependencies: the
 * platform owner supplies the samples and consumes the status flags.
 */

#ifndef KFGINS_QUAD_ADAPTER_HPP
#define KFGINS_QUAD_ADAPTER_HPP

#include <cstdint>

#include "kfgins/quad_types.hpp"

namespace kfg {

/* Sample-time window and uniformity limits, spec 05 §3.1 (1 kHz target). */
constexpr double kImuDtMinUs = 950.0;
constexpr double kImuDtMaxUs = 1100.0;
constexpr double kImuDtRatioTolerance = 0.01;

struct ImuSample {
    Vec3 gyro_radps;   /* body FRD, calibrated, installation rotated */
    Vec3 accel_mps2;   /* body FRD, calibrated, installation rotated */
    std::uint64_t sample_time_us; /* FCU monotonic, DRDY capture corrected */
    std::uint32_t sequence;
};

enum AdapterStatus : int {
    kAdapterSeed = 0,        /* first sample stored, no increment produced */
    kAdapterIncrement = 1,   /* 'increment' is valid */
    kAdapterBadSample = 2,   /* non-finite or null sample time */
    kAdapterSequenceError = 3, /* duplicate or backwards sequence number */
    kAdapterGap = 4,         /* dt outside [950, 1100] us or ratio > 1 % */
    kAdapterTimestampReversed = 5
};

class ImuIncrementBuilder {
public:
    ImuIncrementBuilder();

    void reset();

    /*
     * Pushes one sample.  On kAdapterIncrement the increment covers
     * (previous, current].  A rejected sample does not become the new seed, so
     * the next valid pair still forms a continuous increment; the caller must
     * treat the gap as a navigation-quality degradation.
     */
    AdapterStatus push(const ImuSample &sample, ImuIncrement &increment);

    bool has_seed() const { return has_seed_; }
    std::uint32_t last_sequence() const { return last_sequence_; }
    std::uint32_t gap_count() const { return gap_count_; }
    std::uint32_t sequence_error_count() const { return sequence_error_count_; }

private:
    bool has_seed_;
    ImuSample previous_;
    double previous_dt_us_;
    std::uint32_t last_sequence_;
    std::uint32_t gap_count_;
    std::uint32_t sequence_error_count_;
};

} // namespace kfg

#endif // KFGINS_QUAD_ADAPTER_HPP
