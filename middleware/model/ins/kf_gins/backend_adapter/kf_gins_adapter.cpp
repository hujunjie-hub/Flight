/*
 * KF-GINS_QUAD IMU increment builder implementation (spec 05 §3.1).
 */

#include "backend_adapter/kf_gins_adapter.hpp"

#include <cmath>

namespace kfg {

ImuIncrementBuilder::ImuIncrementBuilder()
    : has_seed_(false), previous_dt_us_(0.0), last_sequence_(0), gap_count_(0), sequence_error_count_(0) {
    setZero(previous_.gyro_radps);
    setZero(previous_.accel_mps2);
    previous_.sample_time_us = 0;
    previous_.sequence = 0;
}

void ImuIncrementBuilder::reset() {
    has_seed_ = false;
    previous_dt_us_ = 0.0;
    last_sequence_ = 0;
    gap_count_ = 0;
    sequence_error_count_ = 0;
}

AdapterStatus ImuIncrementBuilder::push(const ImuSample &sample, ImuIncrement &increment) {
    if (sample.sample_time_us == 0) {
        return kAdapterBadSample;
    }
    if (!allFiniteVec(sample.gyro_radps) || !allFiniteVec(sample.accel_mps2)) {
        return kAdapterBadSample;
    }

    if (!has_seed_) {
        previous_ = sample;
        has_seed_ = true;
        last_sequence_ = sample.sequence;
        previous_dt_us_ = 0.0;
        return kAdapterSeed;
    }

    /* Sequence must advance; duplicates and replays are rejected. */
    if (sample.sequence <= last_sequence_) {
        ++sequence_error_count_;
        return kAdapterSequenceError;
    }

    if (sample.sample_time_us <= previous_.sample_time_us) {
        ++sequence_error_count_;
        return kAdapterTimestampReversed;
    }

    const double dt_us = static_cast<double>(sample.sample_time_us - previous_.sample_time_us);
    if (!(dt_us >= kImuDtMinUs) || !(dt_us <= kImuDtMaxUs)) {
        ++gap_count_;
        return kAdapterGap;
    }
    if (previous_dt_us_ > 0.0) {
        const double ratio = dt_us / previous_dt_us_;
        if (!(std::fabs(ratio - 1.0) <= kImuDtRatioTolerance)) {
            ++gap_count_;
            return kAdapterGap;
        }
    }

    const double dt_s = dt_us * 1.0e-6;

    /* Fixed trapezoid integration; the first sample only seeds the interval. */
    for (int i = 0; i < 3; ++i) {
        increment.dtheta(i, 0) = 0.5 * (previous_.gyro_radps(i, 0) + sample.gyro_radps(i, 0)) * dt_s;
        increment.dvel(i, 0) = 0.5 * (previous_.accel_mps2(i, 0) + sample.accel_mps2(i, 0)) * dt_s;
    }
    increment.dt = dt_s;

    previous_ = sample;
    previous_dt_us_ = dt_us;
    last_sequence_ = sample.sequence;
    return kAdapterIncrement;
}

} // namespace kfg
