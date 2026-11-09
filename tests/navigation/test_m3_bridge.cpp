/*
 * M3 observation bridge tests: public GnssSample/BaroSample -> journal.
 */

#include <cmath>
#include <cstdio>
#include <cstring>

#include "observation_replay/ins_observation.hpp"
#include "observation_replay/ins_replay.hpp"
#include "kfgins/quad_core.hpp"
#include "kfgins/quad_models.hpp"

using namespace kfg;
using namespace ins;

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        ++g_checks;                                                                                \
        if (!(cond)) {                                                                             \
            ++g_failures;                                                                          \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                            \
        }                                                                                          \
    } while (0)

namespace {

GnssSample gnssSample(std::uint64_t boot_id, std::uint32_t sequence, std::uint64_t time_us) {
    GnssSample sample;
    std::memset(&sample, 0, sizeof(sample));
    sample.header.boot_id = boot_id;
    sample.header.source_id = SOURCE_GNSS;
    sample.header.timestamp_sample_us = time_us;
    sample.header.source_mapping_epoch = 5u;
    sample.sensor_instance_id = sequence;
    sample.sample_sequence = sequence;
    sample.latitude_deg = 30.5;
    sample.longitude_deg = 114.3;
    sample.altitude_m = 35.0f;
    sample.velocity_ned_mps[0] = 1.0f;
    sample.velocity_ned_mps[1] = -0.5f;
    sample.velocity_ned_mps[2] = 0.1f;
    for (int i = 0; i < 3; ++i) {
        sample.position_variance_m2[i] = 0.25f;
        sample.velocity_variance_m2ps2[i] = 0.04f;
    }
    sample.fix_type = GNSS_FIX_RTK_FIXED;
    sample.satellite_count = 20u;
    sample.quality_flags = GNSS_Q_POSITION_VALID | GNSS_Q_VELOCITY_VALID |
                           GNSS_Q_POSITION_COV_VALID | GNSS_Q_VELOCITY_COV_VALID;
    return sample;
}

BaroSample baroSample(std::uint64_t boot_id, std::uint32_t sequence, std::uint64_t time_us) {
    BaroSample sample;
    std::memset(&sample, 0, sizeof(sample));
    sample.header.boot_id = boot_id;
    sample.header.source_id = SOURCE_BARO;
    sample.header.sequence = sequence;
    sample.header.timestamp_sample_us = time_us;
    sample.sensor_instance_id = sequence;
    sample.altitude_m = 0.6f;
    sample.variance_m2 = 0.09f;
    sample.quality_flags = BARO_Q_ALTITUDE_VALID | BARO_Q_VARIANCE_VALID | BARO_Q_REFERENCE_VALID;
    return sample;
}

void test_gnss_conversion(void) {
    InsHistory history;
    const GnssSample sample = gnssSample(0xB00B00ULL, 11u, 1000000u);

    CHECK(acceptGnssSample(history, sample, 1000500u, 1u, 5u, 1u, 1u) == AcceptStatus::kAccepted);
    CHECK(history.journalCount() == 2u); /* position + velocity */

    const ObservationRecord &position = history.journal(0);
    CHECK(position.id.kind == ObservationKind::kGnssPosition);
    CHECK(position.sample_time_us == 1000000u);
    CHECK(position.mapping_epoch == 5u);
    CHECK(position.a[0] == 30.5 * 0.017453292519943295);
    CHECK(std::fabs(position.b[0] - 0.5) < 1e-6); /* std from variance 0.25 */

    const ObservationRecord &velocity = history.journal(1);
    CHECK(velocity.id.kind == ObservationKind::kGnssVelocity);
    CHECK(velocity.a[0] == 1.0);
    CHECK(std::fabs(velocity.b[2] - 0.2) < 1e-6);

    /* Same public sample again: both records are duplicates, nothing added. */
    CHECK(acceptGnssSample(history, sample, 1000500u, 1u, 5u, 1u, 1u) == AcceptStatus::kDuplicate);
    CHECK(history.journalCount() == 2u);

    /* Stale: 25 ms after the measurement epoch. */
    GnssSample stale = gnssSample(0xB00B00ULL, 12u, 2000000u);
    CHECK(acceptGnssSample(history, stale, 2025001u, 1u, 5u, 1u, 1u) == AcceptStatus::kTooOld);

    /* Position-only sample: one record. */
    GnssSample position_only = gnssSample(0xB00B00ULL, 13u, 3000000u);
    position_only.quality_flags = GNSS_Q_POSITION_VALID;
    CHECK(acceptGnssSample(history, position_only, 3000500u, 1u, 5u, 1u, 1u) ==
          AcceptStatus::kAccepted);
    CHECK(history.journalCount() == 3u);

    /* Neither position nor velocity valid: rejected before the journal. */
    GnssSample empty = gnssSample(0xB00B00ULL, 14u, 4000000u);
    empty.quality_flags = 0u;
    CHECK(acceptGnssSample(history, empty, 4000500u, 1u, 5u, 1u, 1u) ==
          AcceptStatus::kInvalidArgument);
}

void test_baro_conversion(void) {
    InsHistory history;
    const BaroSample sample = baroSample(0xB00B00ULL, 21u, 1000000u);
    CHECK(acceptBaroSample(history, sample, 1000500u, 1u, 1u) == AcceptStatus::kAccepted);
    CHECK(history.journalCount() == 1u);
    CHECK(history.journal(0).id.kind == ObservationKind::kBaroHeight);
    CHECK(std::fabs(history.journal(0).b[0] - 0.3) < 1e-6);

    CHECK(acceptBaroSample(history, sample, 1000500u, 1u, 1u) == AcceptStatus::kDuplicate);

    BaroSample invalid = baroSample(0xB00B00ULL, 22u, 2000000u);
    invalid.quality_flags = BARO_Q_VARIANCE_VALID;
    CHECK(acceptBaroSample(history, invalid, 2000500u, 1u, 1u) == AcceptStatus::kInvalidArgument);
}

void test_replay_with_bridge_records(void) {
    QuadConfig config;
    config.imunoise.gyr_arw = vec3(0.0015, 0.0015, 0.0015);
    config.imunoise.acc_vrw = vec3(0.025, 0.025, 0.025);
    config.imunoise.gyrbias_std = vec3(5.0e-4, 5.0e-4, 5.0e-4);
    config.imunoise.accbias_std = vec3(0.02, 0.02, 0.02);
    config.imunoise.gyrscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    config.imunoise.accscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    config.imunoise.corr_time = 3600.0;
    config.imunoise.baro_bias_walk = 1.0e-4;
    config.antlever = vec3(0.12, -0.05, 0.21);
    config.nis_gate_enable = true;
    config.baro_origin_height_m = 30.0;

    QuadInitState init;
    init.nominal.pos = vec3(30.5 * kD2R, 114.3 * kD2R, 35.0);
    init.nominal.vel = vec3(0.0, 0.0, 0.0);
    init.nominal.euler = vec3(0.01, -0.02, 0.4);
    init.nominal.cbn = eulerToMatrix(init.nominal.euler);
    init.nominal.qbn = matrixToQuat(init.nominal.cbn);
    init.nominal.gyrbias = vec3(1.0e-4, -2.0e-4, 3.0e-4);
    init.nominal.accbias = vec3(0.01, -0.02, 0.03);
    init.nominal.gyrscale = vec3(0.0, 0.0, 0.0);
    init.nominal.accscale = vec3(0.0, 0.0, 0.0);
    init.nominal.baro_bias = 0.5;
    init.pos_std = vec3(1.0, 1.0, 1.5);
    init.vel_std = vec3(0.1, 0.1, 0.1);
    init.euler_std = vec3(0.02, 0.02, 0.1);
    init.gyrbias_std = config.imunoise.gyrbias_std;
    init.accbias_std = config.imunoise.accbias_std;
    init.gyrscale_std = config.imunoise.gyrscale_std;
    init.accscale_std = config.imunoise.accscale_std;
    init.baro_bias_std = 1.5;

    QuadCore core;
    CHECK(core.initialize(config, init) == kOk);
    InsHistory history;
    ReplayEngine engine;
    ReplayCandidate candidate;

    for (std::uint32_t step = 1u; step <= 16u; ++step) {
        const std::uint64_t time_us = (std::uint64_t)step * 1000u;
        const Vec3 f_nav = vec3(0.0, 0.0, -gravity(core.nominal().pos));
        const Vec3 f_body = matvecTransposed(core.nominal().cbn, f_nav);
        ImuIncrement increment;
        increment.dt = 0.001;
        for (int i = 0; i < 3; ++i) {
            increment.dtheta(i, 0) = core.nominal().gyrbias(i, 0) * 0.001;
            increment.dvel(i, 0) = (f_body(i, 0) + core.nominal().accbias(i, 0)) * 0.001;
        }
        CHECK(core.propagate(increment) == kOk);
        CHECK(history.pushImu(time_us, increment, step));

        if (step == 8u) {
            CHECK(acceptBaroSample(history, baroSample(0xB00B00ULL, step, time_us), time_us + 100u,
                                   1u, 1u) == AcceptStatus::kAccepted);
            /* Apply the same baro to the active core as the forward run. */
            BaroObs obs;
            obs.height = 0.6;
            obs.std = 0.3;
            CHECK(core.updateBaro(obs) == kOk);
            QuadCheckpoint checkpoint;
            CHECK(core.saveCheckpoint(&checkpoint, 1u, 0u, 1u, time_us, step) == kOk);
            CHECK(history.pushCheckpoint(checkpoint));
        }
    }

    std::uint32_t index = 0;
    CHECK(history.findCheckpoint(8000u, &index));
    QuadCore shadow;
    CHECK(shadow.initialize(config, init) == kOk);
    CHECK(engine.replay(history, index, 16000u, 16u, 1u, shadow, &candidate) == kOk);
    CHECK(candidate.applied_observations == 0u); /* the only observation is before the checkpoint */

    double worst = 0.0;
    for (int r = 0; r < kRank22; ++r) {
        for (int c = 0; c < kRank22; ++c) {
            worst = std::fmax(worst, std::fabs(core.covariance()(r, c) - shadow.covariance()(r, c)));
        }
    }
    CHECK(worst < 1.0e-7);
}

} // namespace

int main() {
    std::printf("M3 observation bridge tests\n");
    test_gnss_conversion();
    test_baro_conversion();
    test_replay_with_bridge_records();
    std::printf("test_m3_bridge: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
