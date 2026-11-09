/*
 * M3 INS history, observation journal and shadow replay.
 *
 * Owner: Agent B.  Authority: spec 05 §5.2/§5.3 (fixed-lag history, journal,
 * checkpoint and P4 atomic commit) and spec 07 §6.4 (memory).
 *
 * Design boundaries:
 *   - The active filter state is written by the prediction path only.  This
 *     module never mutates an active QuadCore: it restores a checkpoint into a
 *     separate shadow core, replays, and hands the candidate back for the
 *     caller to commit at an IMU boundary.
 *   - Observations enter one journal; the arrival queue and the processed
 *     index reference the same slot (no second payload copy).
 *   - A replayed observation is applied at most once, in deterministic
 *     (sample_time, source_id, source_sequence, kind) order.
 *
 * Memory (spec 07 §6.4): 256 x 32 B IMU slots, 24 x 2304 B checkpoints,
 * 64 x 320 B journal slots.  The rings are static members: no heap.
 */

#ifndef INS_REPLAY_HPP
#define INS_REPLAY_HPP

#include <cstddef>
#include <cstdint>

#include "kfgins/quad_core.hpp"

namespace ins {

constexpr std::uint32_t kImuSlots = 256u;
constexpr std::uint32_t kCheckpointSlots = 24u;
constexpr std::uint32_t kJournalSlots = 64u;
constexpr std::uint32_t kJournalSlotBytes = 320u;
constexpr std::uint64_t kMaxObservationAgeUs = 20000u;
constexpr std::uint64_t kCheckpointIntervalUs = 8000u;

/* Fixed 32-byte IMU slot: six float increments + dt_us + sequence.  The sample
 * time is reconstructed from the ring base time (spec 07 §6.4). */
struct ImuSlot {
    float dtheta[3];
    float dvel[3];
    std::uint32_t dt_us;
    std::uint32_t sequence;
};

enum class ObservationKind : std::uint16_t {
    kGnssPosition = 1,
    kGnssVelocity = 2,
    kBaroHeight = 3,
    kMagHeading = 4
};

enum class ObservationResult : std::int32_t {
    kAccepted = 0,
    kRejectedNis = 1,
    kRejectedRange = 2,
    kRejectedStale = 3,
    kRejectedVersion = 4
};

struct ObservationId {
    std::uint64_t boot_id;
    std::uint32_t source_id;
    std::uint32_t source_sequence;
    ObservationKind kind;
    std::uint16_t reserved;
};

/* One journal record.  'payload' holds the standardized observation in the
 * same units the estimator consumes; it is never re-parsed from the wire. */
struct ObservationRecord {
    ObservationId id;
    std::uint64_t sample_time_us;
    std::uint32_t policy_epoch;
    std::uint32_t noise_profile;
    std::uint32_t calibration_version;
    std::uint32_t mapping_epoch;
    std::uint32_t journal_sequence;
    ObservationResult result;
    /* Payload: interpreted by 'kind'.
     *   GNSS position : a = blh, b = std
     *   GNSS velocity : a = velocity NED, b = std, omega = body rate
     *   Baro height   : a(0) = relative height, b(0) = std
     */
    double a[3];
    double b[3];
    double omega[3];
};

enum class AcceptStatus {
    kAccepted = 0,
    kDuplicate = 1,
    kTooOld = 2,
    kJournalFull = 3,
    kInvalidArgument = 4
};

struct ObservationEvaluation {
    std::uint32_t source_id=0, source_sequence=0, policy_epoch=0;
    std::uint64_t sample_time_us=0;
    bool accepted=false;
};
/* Candidate handed back by the replay; the caller owns the commit decision. */
struct ReplayCandidate {
    kfg::QuadCheckpoint checkpoint; /* resulting state at the target boundary */
    std::uint32_t base_fusion_revision;
    std::uint32_t journal_high_watermark;
    std::uint32_t applied_observations;
    std::uint32_t target_imu_sequence;
    std::uint64_t accepted_sample_time_us[4]; /* per observation kind, internal */
    ObservationEvaluation latest_evaluation[4];
    bool shadow_only=false; /* Probe candidates can never be committed. */
};

class InsHistory {
public:
    InsHistory();

    void reset();

    /* IMU ring.  Returns false when the ring is full (the caller must keep
     * propagating and count the overwrite; the ring never grows). */
    bool pushImu(std::uint64_t sample_time_us, const kfg::ImuIncrement &increment,
                 std::uint32_t sequence);

    /* Checkpoint ring on a delta_x = 0 boundary. */
    bool pushCheckpoint(const kfg::QuadCheckpoint &checkpoint);
    bool pushCheckpoint(const kfg::QuadCore &core, std::uint32_t fusion_revision,
                        std::uint32_t origin_id, std::uint32_t state_epoch,
                        std::uint64_t sample_time_us, std::uint32_t imu_sequence);

    /* Journal insertion with validation and de-duplication.  A rejected
     * observation is not stored: this is the arrival-side gate (age, epoch,
     * payload validity).  'now_us' is the arrival time used only for age. */
    AcceptStatus acceptObservation(const ObservationRecord &observation, std::uint64_t arrival_us,
                                   std::uint32_t current_policy_epoch,
                                   std::uint32_t current_mapping_epoch);

    std::uint32_t imuCount() const { return imu_count_; }
    std::uint32_t checkpointCount() const { return checkpoint_count_; }
    std::uint32_t journalCount() const { return journal_count_; }
    std::uint32_t journalHighWatermark() const { return journal_sequence_; }

    /* Logical index: 0 is the oldest retained entry, count-1 the newest. */
    const ImuSlot &imuSlot(std::uint32_t index) const;
    std::uint64_t imuSampleTime(std::uint32_t index) const;
    std::uint32_t imuSequence(std::uint32_t index) const;

    /* Slot index as returned by findCheckpoint(). */
    const kfg::QuadCheckpoint &checkpoint(std::uint32_t index) const;
    const ObservationRecord &journal(std::uint32_t index) const;

    /* Most recent checkpoint whose sample time is <= 'sample_time_us'.  The
     * search is O(1) through the generation ring (spec 05 §5.3 step 5). */
    bool findCheckpoint(std::uint64_t sample_time_us, std::uint32_t *index) const;
    void invalidateAfter(std::uint64_t correction_start_us, std::uint32_t revision);
    void retireJournal();
    bool nearestImuTime(std::uint64_t raw_time_us, std::uint64_t *boundary_us) const;

private:
    void advanceCheckpoint();
    bool hasObservation(const ObservationId &id, std::uint64_t sample_time_us) const;

    ImuSlot imu_[kImuSlots];
    /* Spec 07 §6.4: the ring stores only a base sample time and reconstructs
     * every epoch from dt_us, so no per-slot absolute time is kept. */
    std::uint64_t imu_base_time_us_;
    std::uint32_t imu_head_;
    std::uint32_t imu_count_;

    kfg::QuadCheckpoint checkpoints_[kCheckpointSlots];
    std::uint32_t checkpoint_head_;
    std::uint32_t checkpoint_count_;

    ObservationRecord journal_[kJournalSlots];
    std::uint32_t journal_head_;
    std::uint32_t journal_count_;
    std::uint32_t journal_sequence_;
};

class ReplayEngine {
public:
    ReplayEngine();

    /* Replays every journal observation inside (checkpoint_time, target_time]
     * on top of the checkpoint into 'shadow', propagating the stored IMU
     * increments in between.  The caller passes the fusion revision the
     * candidate is based on; the active filter is not touched at all. */
    kfg::Status replay(const InsHistory &history, std::uint32_t checkpoint_index,
                       std::uint64_t target_time_us, std::uint32_t target_imu_sequence,
                       std::uint32_t base_fusion_revision, kfg::QuadCore &shadow,
                       ReplayCandidate *candidate) const;

    /* Commits a candidate into 'active' when the candidate target is the last
     * completed IMU boundary and the fusion revision still matches. */
    kfg::Status commit(kfg::QuadCore &active, const ReplayCandidate &candidate,
                       std::uint32_t current_fusion_revision, std::uint32_t last_imu_sequence);

    static bool observationBefore(const ObservationRecord &lhs, const ObservationRecord &rhs);
};

/* Resumable shadow cursor. The serialized owner pins journal retirement while
 * active. New journal insertion cancels/restarts the cursor; history overwrite
 * or a changed revision is rejected, never committed. One step performs one
 * propagation or one observation update, so callers can meter CPU time. */
class ReplayCursor {
public:
    kfg::Status begin(const InsHistory &history, std::uint32_t checkpoint_index,
                      std::uint32_t revision, kfg::QuadCore &shadow,
                      const ObservationRecord *probes=nullptr, unsigned probe_count=0,
                      std::uint64_t target_time=UINT64_MAX);
    kfg::Status step(const InsHistory &history, std::uint64_t target_time,
                     std::uint32_t target_sequence, kfg::QuadCore &shadow,
                     ReplayCandidate *candidate, bool *done);
    void cancel() { active_ = false; }
    bool active() const { return active_; }
    std::uint64_t baseTime() const { return base_time_; }
private:
    const ObservationRecord &record(const InsHistory &h,unsigned index) const {
        return index<kJournalSlots ? h.journal(index) : probes_[index-kJournalSlots];
    }
    std::uint32_t order_[kJournalSlots+2]{};
    std::uint64_t quantized_[kJournalSlots+2]{};
    ObservationRecord probes_[2]{};
    unsigned probe_count_=0;
    ObservationEvaluation latest_evaluation_[4]{};
    std::uint64_t accepted_[4]{};
    std::uint32_t count_=0, next_=0, applied_=0, revision_=0, watermark_=0;
    std::uint32_t sequence_=0, origin_=0, state_=0;
    std::uint64_t base_time_=0, time_=0;
    kfg::ImuIncrement increment_{};
    bool active_=false;
};

} // namespace ins

#endif // INS_REPLAY_HPP
