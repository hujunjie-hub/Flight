/*
 * M3 history / journal / shadow replay tests.
 *
 * Verifies the properties of spec 05 §5.2-§5.3 that can be checked on the
 * Host: deterministic ordering, de-duplication, acceptance boundary, journal
 * capacity, checkpoint selection, replay equivalence with the forward run and
 * the atomic-commit guards.
 */

#include <cmath>
#include <cstdio>
#include <cstring>

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

#define CHECK_NEAR(value, reference, tolerance)                                                    \
    do {                                                                                           \
        ++g_checks;                                                                                \
        const double v_ = (value);                                                                 \
        const double r_ = (reference);                                                             \
        if (!(std::fabs(v_ - r_) <= (tolerance))) {                                                \
            ++g_failures;                                                                          \
            std::printf("FAIL %s:%d: %s = %.12g, expected %.12g (tol %.3g)\n", __FILE__, __LINE__, \
                        #value, v_, r_, (tolerance));                                              \
        }                                                                                          \
    } while (0)

namespace {

QuadConfig make_config() {
    QuadConfig cfg;
    cfg.imunoise.gyr_arw = vec3(0.0015, 0.0015, 0.0015);
    cfg.imunoise.acc_vrw = vec3(0.025, 0.025, 0.025);
    cfg.imunoise.gyrbias_std = vec3(5.0e-4, 5.0e-4, 5.0e-4);
    cfg.imunoise.accbias_std = vec3(0.02, 0.02, 0.02);
    cfg.imunoise.gyrscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    cfg.imunoise.accscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    cfg.imunoise.corr_time = 3600.0;
    cfg.imunoise.baro_bias_walk = 1.0e-4;
    cfg.antlever = vec3(0.12, -0.05, 0.21);
    cfg.nis_gate_enable = true;
    cfg.baro_origin_height_m = 30.0;
    return cfg;
}

QuadInitState make_init() {
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
    init.gyrbias_std = vec3(5.0e-4, 5.0e-4, 5.0e-4);
    init.accbias_std = vec3(0.02, 0.02, 0.02);
    init.gyrscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    init.accscale_std = vec3(1.0e-4, 1.0e-4, 1.0e-4);
    init.baro_bias_std = 1.5;
    return init;
}

ImuIncrement static_increment(const QuadNominal &nominal, double dt) {
    const Vec3 f_nav = vec3(0.0, 0.0, -gravity(nominal.pos));
    const Vec3 f_body = matvecTransposed(nominal.cbn, f_nav);
    ImuIncrement increment;
    increment.dt = dt;
    for (int i = 0; i < 3; ++i) {
        increment.dtheta(i, 0) = nominal.gyrbias(i, 0) * (1.0 + nominal.gyrscale(i, 0)) * dt;
        increment.dvel(i, 0) = (f_body(i, 0) + nominal.accbias(i, 0)) *
                               (1.0 + nominal.accscale(i, 0)) * dt;
    }
    return increment;
}

ObservationRecord baro_record(std::uint64_t time_us, std::uint32_t sequence, double height) {
    ObservationRecord record;
    std::memset(&record, 0, sizeof(record));
    record.id.boot_id = 0xB00B00ULL;
    record.id.source_id = 12u;
    record.id.source_sequence = sequence;
    record.id.kind = ObservationKind::kBaroHeight;
    record.sample_time_us = time_us;
    record.policy_epoch = 1u;
    record.noise_profile = 1u;
    record.calibration_version = 1u;
    record.mapping_epoch = 0u;
    record.a[0] = height;
    record.b[0] = 0.3;
    return record;
}

ObservationRecord gnss_position_record(std::uint64_t time_us, std::uint32_t sequence,
                                       const Vec3 &blh) {
    ObservationRecord record;
    std::memset(&record, 0, sizeof(record));
    record.id.boot_id = 0xB00B00ULL;
    record.id.source_id = 11u;
    record.id.source_sequence = sequence;
    record.id.kind = ObservationKind::kGnssPosition;
    record.sample_time_us = time_us;
    record.policy_epoch = 1u;
    record.noise_profile = 1u;
    record.calibration_version = 1u;
    record.mapping_epoch = 7u;
    record.a[0] = blh(0, 0);
    record.a[1] = blh(1, 0);
    record.a[2] = blh(2, 0);
    record.b[0] = 0.5;
    record.b[1] = 0.5;
    record.b[2] = 1.0;
    return record;
}

void apply_to_active(QuadCore &core, const ObservationRecord &record, bool &applied) {
    kfg::Status status = kfg::kErrArgument;
    if (record.id.kind == ObservationKind::kBaroHeight) {
        BaroObs observation;
        observation.height = record.a[0];
        observation.std = record.b[0];
        status = core.updateBaro(observation);
    } else if (record.id.kind == ObservationKind::kGnssPosition) {
        GnssPositionObs observation;
        observation.blh = vec3(record.a[0], record.a[1], record.a[2]);
        observation.std = vec3(record.b[0], record.b[1], record.b[2]);
        status = core.updateGnssPosition(observation);
    }
    applied = (status == kfg::kOk);
}

struct ForwardResult {
    QuadCore core;
    InsHistory history;
    std::uint32_t last_sequence;
    std::uint64_t last_time_us;
};

void run_forward(ForwardResult &result) {
    const QuadConfig config = make_config();
    const QuadInitState init = make_init();
    CHECK(result.core.initialize(config, init) == kOk);

    const double dt = 0.001;
    const std::uint32_t checkpoints_every = 8u;

    for (std::uint32_t step = 1u; step <= 40u; ++step) {
        const std::uint64_t time_us = static_cast<std::uint64_t>(step) * 1000u;
        const ImuIncrement increment = static_increment(result.core.nominal(), dt);
        CHECK(result.core.propagate(increment) == kOk);
        CHECK(result.history.pushImu(time_us, increment, step));

        bool has_observation = false;
        ObservationRecord record;
        if ((step == 10u) || (step == 20u) || (step == 30u)) {
            record = baro_record(time_us, step, 0.6);
            has_observation = true;
        } else if (step == 24u) {
            const Vec3 ant = gnssPositionPrediction(result.core.nominal(), config);
            record = gnss_position_record(time_us, step, ant);
            has_observation = true;
        }

        if (has_observation) {
            CHECK(result.history.acceptObservation(record, time_us + 500u, 1u, 7u) ==
                  AcceptStatus::kAccepted);
            bool applied = false;
            apply_to_active(result.core, record, applied);
            CHECK(applied);
        }

        if ((step % checkpoints_every) == 0u) {
            QuadCheckpoint checkpoint;
            CHECK(result.core.saveCheckpoint(&checkpoint, 1u, 0u, 1u, time_us, step) == kOk);
            CHECK(result.history.pushCheckpoint(checkpoint));
        }
        result.last_sequence = step;
        result.last_time_us = time_us;
    }
}

double max_state_diff(const QuadCore &lhs, const QuadCore &rhs) {
    double worst = 0.0;
    const QuadNominal &a = lhs.nominal();
    const QuadNominal &b = rhs.nominal();
    for (int i = 0; i < 3; ++i) {
        worst = std::fmax(worst, std::fabs(a.pos(i, 0) - b.pos(i, 0)));
        worst = std::fmax(worst, std::fabs(a.vel(i, 0) - b.vel(i, 0)));
        worst = std::fmax(worst, std::fabs(a.gyrbias(i, 0) - b.gyrbias(i, 0)));
        worst = std::fmax(worst, std::fabs(a.accbias(i, 0) - b.accbias(i, 0)));
    }
    worst = std::fmax(worst, std::fabs(a.baro_bias - b.baro_bias));
    for (int r = 0; r < kRank22; ++r) {
        for (int c = 0; c < kRank22; ++c) {
            worst = std::fmax(worst, std::fabs(lhs.covariance()(r, c) - rhs.covariance()(r, c)));
        }
    }
    return worst;
}

void test_replay_equivalence() {
    ForwardResult forward;
    run_forward(forward);

    std::uint32_t checkpoint_index = 0u;
    CHECK(forward.history.findCheckpoint(24000u, &checkpoint_index));
    CHECK(forward.history.checkpoint(checkpoint_index).sample_time_us == 24000u);

    /* The shadow must carry the same configuration; the checkpoint only holds
     * state, not noise/lever/gate settings. */
    QuadCore shadow;
    CHECK(shadow.initialize(make_config(), make_init()) == kOk);
    ReplayEngine engine;
    ReplayCandidate candidate;
    CHECK(engine.replay(forward.history, checkpoint_index, forward.last_time_us,
                        forward.last_sequence, 1u, shadow, &candidate) == kOk);
    CHECK(candidate.target_imu_sequence == forward.last_sequence);
    CHECK(candidate.applied_observations == 1u); /* only the baro at 30 ms is after 24 ms */

    /* The 32-byte IMU slot stores six floats (spec 07 §6.4), so replaying the
     * stored increments is not bit-identical to the forward run that used
     * doubles; the difference is bounded by the float quantization of the
     * increments and stays far below any navigation-relevant threshold. */
    const double diff = max_state_diff(forward.core, shadow);
    std::printf("replay equivalence: max |active - shadow| = %.3e (float IMU slot)\n", diff);
    CHECK_NEAR(diff, 0.0, 1.0e-7);

    /* Pausing after every numerical operation reproduces the synchronous
     * shadow, while leaving the active trajectory untouched. */
    QuadCore sliced;
    CHECK(sliced.initialize(make_config(), make_init()) == kOk);
    ReplayCursor cursor;
    ReplayCandidate sliced_candidate{};
    CHECK(cursor.begin(forward.history,checkpoint_index,1u,sliced)==kOk);
    bool done=false;
    unsigned operations=0;
    while(!done && operations++<100)
        CHECK(cursor.step(forward.history,forward.last_time_us,forward.last_sequence,
                          sliced,&sliced_candidate,&done)==kOk);
    CHECK(done);
    CHECK(operations>1);
    CHECK_NEAR(max_state_diff(sliced,shadow),0.0,1e-12);
    CHECK(sliced_candidate.applied_observations==candidate.applied_observations);
    CHECK(sliced_candidate.target_imu_sequence==candidate.target_imu_sequence);
    /* An isolated source probe is not inserted into the active journal and
     * cannot be accidentally committed as a navigation correction. */
    ObservationRecord probe=baro_record(31000u,999u,1000.0);
    QuadCore isolated;
    CHECK(isolated.initialize(make_config(),make_init())==kOk);
    ReplayCursor probe_cursor;
    ReplayCandidate probe_candidate{};
    CHECK(probe_cursor.begin(forward.history,checkpoint_index,1u,isolated,&probe,1,forward.last_time_us)==kOk);
    done=false;operations=0;
    while(!done && operations++<100)
      CHECK(probe_cursor.step(forward.history,forward.last_time_us,forward.last_sequence,
                             isolated,&probe_candidate,&done)==kOk);
    CHECK(done && probe_candidate.shadow_only);
    CHECK(!probe_candidate.latest_evaluation[2].accepted);
    CHECK(engine.commit(forward.core,probe_candidate,1u,forward.last_sequence)==kErrArgument);
    CHECK_NEAR(max_state_diff(forward.core,shadow),0.0,1.0e-7);

    /* Committing the candidate at the last boundary reproduces the same state. */
    QuadCore committed;
    CHECK(committed.initialize(make_config(), make_init()) == kOk);
    CHECK(engine.commit(committed, candidate, 1u, forward.last_sequence) == kOk);
    CHECK_NEAR(max_state_diff(forward.core, committed), 0.0, 1.0e-7);

    /* Stale revision and wrong boundary are refused. */
    QuadCore other;
    CHECK(other.initialize(make_config(), make_init()) == kOk);
    CHECK(engine.commit(other, candidate, 2u, forward.last_sequence) == kErrArgument);
    CHECK(engine.commit(other, candidate, 1u, forward.last_sequence + 1u) ==
          kErrTimeDiscontinuity);
}

void test_acceptance_rules() {
    InsHistory history;
    ObservationRecord record = baro_record(1000000u, 1u, 0.5);

    CHECK(history.acceptObservation(record, 1000500u, 1u, 7u) == AcceptStatus::kAccepted);
    CHECK(history.journalCount() == 1u);
    CHECK(history.journalHighWatermark() == 1u);

    /* Same measurement id and epoch: duplicate, not a second slot. */
    CHECK(history.acceptObservation(record, 1000500u, 1u, 7u) == AcceptStatus::kDuplicate);
    CHECK(history.journalCount() == 1u);

    /* Older than the 20 ms fusion window. */
    ObservationRecord old = baro_record(2000000u, 2u, 0.5);
    CHECK(history.acceptObservation(old, 2020001u, 1u, 7u) == AcceptStatus::kTooOld);

    /* Arrival before the mapped epoch: invalid. */
    CHECK(history.acceptObservation(old, 1999000u, 1u, 7u) == AcceptStatus::kTooOld);

    /* GNSS mapped through a superseded mapping epoch. */
    QuadConfig config = make_config();
    QuadInitState init = make_init();
    QuadCore core;
    CHECK(core.initialize(config, init) == kOk);
    ObservationRecord gnss = gnss_position_record(3000000u, 3u, init.nominal.pos);
    CHECK(history.acceptObservation(gnss, 3000500u, 1u, 9u) == AcceptStatus::kInvalidArgument);
    CHECK(history.acceptObservation(gnss, 3000500u, 1u, 7u) == AcceptStatus::kAccepted);

    /* Exactly at the window edge is still accepted, one microsecond later not. */
    ObservationRecord edge = baro_record(4000000u, 4u, 0.5);
    CHECK(history.acceptObservation(edge, 4020000u, 1u, 7u) == AcceptStatus::kAccepted);
    ObservationRecord edge2 = baro_record(5000000u, 5u, 0.5);
    CHECK(history.acceptObservation(edge2, 5020001u, 1u, 7u) == AcceptStatus::kTooOld);

    /* Journal capacity: 64 slots, reject afterwards without overwriting. */
    InsHistory full;
    for (std::uint32_t i = 0; i < kJournalSlots; ++i) {
        ObservationRecord item = baro_record(6000000u + i, 100u + i, 0.5);
        CHECK(full.acceptObservation(item, 6000000u + i + 100u, 1u, 7u) == AcceptStatus::kAccepted);
    }
    ObservationRecord overflow = baro_record(6100000u, 999u, 0.5);
    CHECK(full.acceptObservation(overflow, 6100100u, 1u, 7u) == AcceptStatus::kJournalFull);
    CHECK(full.journalCount() == kJournalSlots);
}

void test_checkpoint_selection_and_imu_ring() {
    InsHistory history;
    const QuadConfig config = make_config();
    const QuadInitState init = make_init();
    QuadCore core;
    CHECK(core.initialize(config, init) == kOk);

    for (std::uint32_t step = 1u; step <= 20u; ++step) {
        const std::uint64_t time_us = static_cast<std::uint64_t>(step) * 1000u;
        const ImuIncrement increment = static_increment(core.nominal(), 0.001);
        CHECK(core.propagate(increment) == kOk);
        CHECK(history.pushImu(time_us, increment, step));
        if ((step % 5u) == 0u) {
            QuadCheckpoint checkpoint;
            CHECK(core.saveCheckpoint(&checkpoint, 1u, 0u, 1u, time_us, step) == kOk);
            CHECK(history.pushCheckpoint(core, 1u, 0u, 1u, time_us, step));
            const auto &stored = history.checkpoint(history.checkpointCount() - 1u);
            CHECK(std::memcmp(&stored, &checkpoint, sizeof(stored)) == 0);
        }
    }
    CHECK(history.imuCount() == 20u);
    CHECK(history.checkpointCount() == 4u);

    std::uint32_t index = 0u;
    CHECK(history.findCheckpoint(12000u, &index));
    CHECK(history.checkpoint(index).sample_time_us == 10000u);
    CHECK(history.findCheckpoint(20000u, &index));
    CHECK(history.checkpoint(index).sample_time_us == 20000u);
    CHECK(!history.findCheckpoint(1000u, &index));

    /* A replay that is not aligned to a stored IMU boundary is refused. */
    QuadCore shadow;
    CHECK(shadow.initialize(make_config(), make_init()) == kOk);
    ReplayEngine engine;
    ReplayCandidate candidate;
    CHECK(history.findCheckpoint(10000u, &index));
    CHECK(engine.replay(history, index, 20000u, 999u, 1u, shadow, &candidate) ==
          kErrTimeDiscontinuity);

    /* Ring overwrite: more than 256 samples keep the newest window. */
    for (std::uint32_t step = 21u; step <= 300u; ++step) {
        const ImuIncrement increment = static_increment(core.nominal(), 0.001);
        CHECK(history.pushImu(static_cast<std::uint64_t>(step) * 1000u, increment, step));
    }
    CHECK(history.imuCount() == kImuSlots);
    CHECK(history.imuSlot(history.imuCount() - 1u).sequence == 300u);

    /* Direct serialization also maintains chronological order after wrapping. */
    for (std::uint32_t i = 0u; i < kCheckpointSlots; ++i) {
        const std::uint32_t step = 301u + i;
        CHECK(history.pushCheckpoint(core, 1u, 0u, 1u, step * 1000u, step));
    }
    CHECK(history.checkpointCount() == kCheckpointSlots);
    CHECK(!history.findCheckpoint(300000u, &index));
    CHECK(history.findCheckpoint(301000u, &index));
    CHECK(history.checkpoint(index).imu_sequence == 301u);
    CHECK(history.findCheckpoint((300u + kCheckpointSlots) * 1000u, &index));
    CHECK(history.checkpoint(index).imu_sequence == 300u + kCheckpointSlots);
}

} // namespace

int main() {
    std::printf("M3 replay/journal tests\n");
    test_replay_equivalence();
    test_acceptance_rules();
    test_checkpoint_selection_and_imu_ring();
    std::printf("test_replay: %d checks, %d failures\n", g_checks, g_failures);
    return (g_failures == 0) ? 0 : 1;
}
