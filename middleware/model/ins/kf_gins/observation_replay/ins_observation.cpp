/*
 * Public sample -> M3 observation journal bridge implementation.
 */

#include "observation_replay/ins_observation.hpp"

#include <cmath>
#include <cstring>

namespace ins {
namespace {

ObservationRecord makeRecord(std::uint64_t boot_id, std::uint32_t source_id,
                             std::uint32_t source_sequence, ObservationKind kind,
                             std::uint64_t sample_time_us) {
    ObservationRecord record;
    std::memset(&record, 0, sizeof(record));
    record.id.boot_id = boot_id;
    record.id.source_id = source_id;
    record.id.source_sequence = source_sequence;
    record.id.kind = kind;
    record.sample_time_us = sample_time_us;
    record.result = ObservationResult::kAccepted;
    return record;
}

} // namespace

AcceptStatus acceptGnssSample(InsHistory &history, const GnssSample &sample, uint64_t arrival_us,
                              uint32_t policy_epoch, uint32_t current_mapping_epoch,
                              std::uint32_t position_noise_profile,
                              std::uint32_t velocity_noise_profile) {
    const bool position = (sample.quality_flags & GNSS_Q_POSITION_VALID) != 0u;
    const bool velocity = (sample.quality_flags & GNSS_Q_VELOCITY_VALID) != 0u;
    if (!position && !velocity) {
        return AcceptStatus::kInvalidArgument;
    }

    AcceptStatus worst = AcceptStatus::kAccepted;
    if (position) {
        ObservationRecord record = makeRecord(sample.header.boot_id, sample.header.source_id,
                                              sample.sample_sequence,
                                              ObservationKind::kGnssPosition,
                                              sample.header.timestamp_sample_us);
        record.policy_epoch = policy_epoch;
        record.noise_profile = position_noise_profile;
        record.calibration_version = sample.calibration_version;
        record.mapping_epoch = sample.header.source_mapping_epoch;
        record.a[0] = sample.latitude_deg * 0.017453292519943295;
        record.a[1] = sample.longitude_deg * 0.017453292519943295;
        record.a[2] = (double)sample.altitude_m;
        record.b[0] = (double)sample.position_variance_m2[0];
        record.b[1] = (double)sample.position_variance_m2[1];
        record.b[2] = (double)sample.position_variance_m2[2];
        /* Estimator observation model takes one-sigma noise. */
        record.b[0] = std::sqrt(record.b[0]);
        record.b[1] = std::sqrt(record.b[1]);
        record.b[2] = std::sqrt(record.b[2]);
        const AcceptStatus status =
            history.acceptObservation(record, arrival_us, policy_epoch, current_mapping_epoch);
        if (status != AcceptStatus::kAccepted) {
            worst = status;
        }
    }
    if (velocity) {
        ObservationRecord record = makeRecord(sample.header.boot_id, sample.header.source_id,
                                              sample.sample_sequence,
                                              ObservationKind::kGnssVelocity,
                                              sample.header.timestamp_sample_us);
        record.policy_epoch = policy_epoch;
        record.noise_profile = velocity_noise_profile;
        record.mapping_epoch = sample.header.source_mapping_epoch;
        record.a[0] = (double)sample.velocity_ned_mps[0];
        record.a[1] = (double)sample.velocity_ned_mps[1];
        record.a[2] = (double)sample.velocity_ned_mps[2];
        record.b[0] = std::sqrt((double)sample.velocity_variance_m2ps2[0]);
        record.b[1] = std::sqrt((double)sample.velocity_variance_m2ps2[1]);
        record.b[2] = std::sqrt((double)sample.velocity_variance_m2ps2[2]);
        /* The body rate used by the lever-arm term is supplied by the caller
         * through the observation payload; zero means "no rotation term". */
        const AcceptStatus status =
            history.acceptObservation(record, arrival_us, policy_epoch, current_mapping_epoch);
        if (status != AcceptStatus::kAccepted) {
            worst = status;
        }
    }
    return worst;
}

AcceptStatus acceptBaroSample(InsHistory &history, const BaroSample &sample, uint64_t arrival_us,
                              uint32_t policy_epoch, std::uint32_t noise_profile) {
    if ((sample.quality_flags & BARO_Q_ALTITUDE_VALID) == 0u) {
        return AcceptStatus::kInvalidArgument;
    }
    ObservationRecord record = makeRecord(sample.header.boot_id, sample.header.source_id,
                                          sample.header.sequence, ObservationKind::kBaroHeight,
                                          sample.header.timestamp_sample_us);
    record.policy_epoch = policy_epoch;
    record.noise_profile = noise_profile;
    record.calibration_version=sample.calibration_version;
    record.a[0] = (double)sample.altitude_m;
    record.b[0] = std::sqrt((double)sample.variance_m2);
    return history.acceptObservation(record, arrival_us, policy_epoch, 0u);
}

} // namespace ins
