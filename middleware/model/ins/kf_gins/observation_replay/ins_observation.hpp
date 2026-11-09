/*
 * Public sample -> M3 observation journal bridge.
 *
 * Owner: Agent B.  The journal (ins_replay.hpp) is deliberately independent of
 * the public ICD; this bridge is the only place where public samples
 * (GnssSample, BaroSample) are converted into journal records, so the M2 ->
 * M3 hand-off has one testable seam.
 */

#ifndef INS_OBSERVATION_HPP
#define INS_OBSERVATION_HPP

#include "base_types/common_base.h"
#include "observation_replay/ins_replay.hpp"

namespace ins {

/*
 * A GNSS sample carries position and velocity together; the estimator treats
 * them as two independent observations (spec 05 §3.2).  Each record gets its
 * own kind and reuses the sample sequence for de-duplication, so replaying the
 * same public sample twice is rejected for both records.
 */
AcceptStatus acceptGnssSample(InsHistory &history, const GnssSample &sample, uint64_t arrival_us,
                              uint32_t policy_epoch, uint32_t current_mapping_epoch,
                              std::uint32_t position_noise_profile,
                              std::uint32_t velocity_noise_profile);

AcceptStatus acceptBaroSample(InsHistory &history, const BaroSample &sample, uint64_t arrival_us,
                              uint32_t policy_epoch, std::uint32_t noise_profile);

} // namespace ins

#endif // INS_OBSERVATION_HPP
