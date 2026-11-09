/*
 * M3 INS history / observation journal / shadow replay implementation.
 *
 * See ins_replay.hpp for the design contract.  No heap, no exceptions: the
 * rings are fixed-size members and the replay order buffer is a small stack
 * array bounded by kJournalSlots.
 */

#include "observation_replay/ins_replay.hpp"

#include <cmath>
#include <cstring>

namespace ins {
namespace {

bool sameId(const ObservationId &a, const ObservationId &b) {
    return (a.boot_id == b.boot_id) && (a.source_id == b.source_id) &&
           (a.source_sequence == b.source_sequence) && (a.kind == b.kind);
}

bool observationLess(const ObservationRecord &a, const ObservationRecord &b) {
    if (a.sample_time_us != b.sample_time_us) {
        return a.sample_time_us < b.sample_time_us;
    }
    if (a.id.source_id != b.id.source_id) {
        return a.id.source_id < b.id.source_id;
    }
    if (a.id.source_sequence != b.id.source_sequence) {
        return a.id.source_sequence < b.id.source_sequence;
    }
    return static_cast<std::uint16_t>(a.id.kind) < static_cast<std::uint16_t>(b.id.kind);
}

kfg::Status applyObservation(kfg::QuadCore &core, const ObservationRecord &record) {
    switch (record.id.kind) {
    case ObservationKind::kGnssPosition: {
        kfg::GnssPositionObs observation;
        observation.blh = kfg::vec3(record.a[0], record.a[1], record.a[2]);
        observation.std = kfg::vec3(record.b[0], record.b[1], record.b[2]);
        return core.updateGnssPosition(observation);
    }
    case ObservationKind::kGnssVelocity: {
        kfg::GnssVelocityObs observation;
        observation.vel_ned = kfg::vec3(record.a[0], record.a[1], record.a[2]);
        observation.std = kfg::vec3(record.b[0], record.b[1], record.b[2]);
        observation.omega_ib = kfg::vec3(record.omega[0], record.omega[1], record.omega[2]);
        return core.updateGnssVelocity(observation);
    }
    case ObservationKind::kBaroHeight: {
        kfg::BaroObs observation;
        observation.height = record.a[0];
        observation.std = record.b[0];
        return core.updateBaro(observation);
    }
    case ObservationKind::kMagHeading: {
        /* Tilt compensation uses the state at the observation's IMU epoch. */
        const auto tilt=kfg::eulerToMatrix(kfg::vec3(core.nominal().euler(0,0),
                                                   core.nominal().euler(1,0),0));
        const auto field=kfg::matvec(tilt,kfg::vec3(record.a[0],record.a[1],record.a[2]));
        const double horizontal=std::hypot(field(0,0),field(1,0));
        if(horizontal<record.omega[1]) return kfg::kErrNisRejected;
        const double heading=record.omega[0]-std::atan2(field(1,0),field(0,0));
        return core.updateHeading(heading,record.b[0]);
    }
    }
    return kfg::kErrArgument;
}

} // namespace

/* ------------------------------------------------------------------------- */
/* InsHistory                                                                */
/* ------------------------------------------------------------------------- */

InsHistory::InsHistory()
    : imu_base_time_us_(0u), imu_head_(0u), imu_count_(0u), checkpoint_head_(0u),
      checkpoint_count_(0u), journal_head_(0u), journal_count_(0u), journal_sequence_(0u) {
    std::memset(imu_, 0, sizeof(imu_));
    std::memset(checkpoints_, 0, sizeof(checkpoints_));
    std::memset(journal_, 0, sizeof(journal_));
}

void InsHistory::reset() {
    imu_base_time_us_ = 0u;
    imu_head_ = 0u;
    imu_count_ = 0u;
    checkpoint_head_ = 0u;
    checkpoint_count_ = 0u;
    journal_head_ = 0u;
    journal_count_ = 0u;
    journal_sequence_ = 0u;
    std::memset(imu_, 0, sizeof(imu_));
    std::memset(checkpoints_, 0, sizeof(checkpoints_));
    std::memset(journal_, 0, sizeof(journal_));
}

bool InsHistory::pushImu(std::uint64_t sample_time_us, const kfg::ImuIncrement &increment,
                         std::uint32_t sequence) {
    if (!(increment.dt > 0.0)) {
        return false;
    }
    const std::uint32_t slot = imu_head_;
    ImuSlot &target = imu_[slot];
    if (imu_count_ == 0u) {
        imu_base_time_us_ = sample_time_us;
    } else if (imu_count_ == kImuSlots) {
        /* Overwriting the oldest slot: the reconstructed window base advances
         * by that slot's duration. */
        imu_base_time_us_ += imu_[(slot + 1u) % kImuSlots].dt_us;
    }
    for (int axis = 0; axis < 3; ++axis) {
        target.dtheta[axis] = static_cast<float>(increment.dtheta(axis, 0));
        target.dvel[axis] = static_cast<float>(increment.dvel(axis, 0));
    }
    target.dt_us = static_cast<std::uint32_t>(increment.dt * 1.0e6 + 0.5);
    target.sequence = sequence;

    imu_head_ = (imu_head_ + 1u) % kImuSlots;
    if (imu_count_ < kImuSlots) {
        ++imu_count_;
    }
    return true;
}

bool InsHistory::pushCheckpoint(const kfg::QuadCheckpoint &checkpoint) {
    if (checkpoint.magic != kfg::kQuadCheckpointMagic) {
        return false;
    }
    checkpoints_[checkpoint_head_] = checkpoint;
    advanceCheckpoint();
    return true;
}

bool InsHistory::pushCheckpoint(const kfg::QuadCore &core, std::uint32_t fusion_revision,
                                std::uint32_t origin_id, std::uint32_t state_epoch,
                                std::uint64_t sample_time_us, std::uint32_t imu_sequence) {
    // Serialization validates the state before writing any bytes to the slot.
    if (core.saveCheckpoint(&checkpoints_[checkpoint_head_], fusion_revision, origin_id,
                            state_epoch, sample_time_us, imu_sequence) != kfg::kOk)
        return false;
    advanceCheckpoint();
    return true;
}

void InsHistory::advanceCheckpoint() {
    checkpoint_head_ = (checkpoint_head_ + 1u) % kCheckpointSlots;
    if (checkpoint_count_ < kCheckpointSlots) {
        ++checkpoint_count_;
    }
}

const ImuSlot &InsHistory::imuSlot(std::uint32_t index) const {
    const std::uint32_t slot = (imu_head_ + kImuSlots - imu_count_ + index) % kImuSlots;
    return imu_[slot];
}

std::uint64_t InsHistory::imuSampleTime(std::uint32_t index) const {
    std::uint64_t time_us = imu_base_time_us_;
    for (std::uint32_t i = 0u; i < index; ++i) {
        const std::uint32_t slot = (imu_head_ + kImuSlots - imu_count_ + i + 1u) % kImuSlots;
        time_us += imu_[slot].dt_us;
    }
    return time_us;
}

std::uint32_t InsHistory::imuSequence(std::uint32_t index) const {
    const std::uint32_t slot = (imu_head_ + kImuSlots - imu_count_ + index) % kImuSlots;
    return imu_[slot].sequence;
}

const kfg::QuadCheckpoint &InsHistory::checkpoint(std::uint32_t index) const {
    return checkpoints_[index % kCheckpointSlots];
}

const ObservationRecord &InsHistory::journal(std::uint32_t index) const {
    const std::uint32_t slot = (journal_head_ + kJournalSlots - journal_count_ + index) % kJournalSlots;
    return journal_[slot];
}

bool InsHistory::hasObservation(const ObservationId &id, std::uint64_t sample_time_us) const {
    (void)sample_time_us;
    for (std::uint32_t i = 0; i < journal_count_; ++i) {
        const ObservationRecord &record = journal(i);
        if (sameId(record.id, id)) {
            return true;
        }
    }
    return false;
}

AcceptStatus InsHistory::acceptObservation(const ObservationRecord &observation,
                                           std::uint64_t arrival_us,
                                           std::uint32_t current_policy_epoch,
                                           std::uint32_t current_mapping_epoch) {
    (void)current_policy_epoch; /* policy is stored per record, not compared */

    if (observation.sample_time_us == 0u) {
        return AcceptStatus::kInvalidArgument;
    }
    if (arrival_us < observation.sample_time_us) {
        return AcceptStatus::kTooOld; /* mapped epoch in the future is invalid */
    }
    if ((arrival_us - observation.sample_time_us) > kMaxObservationAgeUs) {
        return AcceptStatus::kTooOld;
    }
    /* A GNSS observation mapped through a superseded mapping cannot be placed
     * on the current time axis; natively timed sources carry epoch 0. */
    if ((observation.mapping_epoch != 0u) && (observation.mapping_epoch != current_mapping_epoch)) {
        return AcceptStatus::kInvalidArgument;
    }
    if (hasObservation(observation.id, observation.sample_time_us)) {
        return AcceptStatus::kDuplicate;
    }
    if (journal_count_ >= kJournalSlots) {
        return AcceptStatus::kJournalFull;
    }

    ObservationRecord stored = observation;
    stored.journal_sequence = journal_sequence_;
    stored.result = ObservationResult::kAccepted;
    journal_[journal_head_] = stored;
    journal_head_ = (journal_head_ + 1u) % kJournalSlots;
    ++journal_count_;
    ++journal_sequence_;
    return AcceptStatus::kAccepted;
}

bool InsHistory::findCheckpoint(std::uint64_t sample_time_us, std::uint32_t *index) const {
    if (index == nullptr) {
        return false;
    }
    bool found = false;
    std::uint64_t best_time = 0u;
    std::uint32_t best_index = 0u;

    for (std::uint32_t i = 0; i < checkpoint_count_; ++i) {
        const std::uint32_t slot = (checkpoint_head_ + kCheckpointSlots - checkpoint_count_ + i) %
                                   kCheckpointSlots;
        if (checkpoints_[slot].magic == kfg::kQuadCheckpointMagic &&
            checkpoints_[slot].sample_time_us <= sample_time_us) {
            if (!found || (checkpoints_[slot].sample_time_us >= best_time)) {
                best_time = checkpoints_[slot].sample_time_us;
                best_index = slot;
                found = true;
            }
        }
    }
    if (found) {
        *index = best_index;
    }
    return found;
}

void InsHistory::invalidateAfter(std::uint64_t time, std::uint32_t revision) {
    for (auto &cp : checkpoints_) {
        if (cp.sample_time_us > time) cp.magic = 0;
        else if (cp.magic == kfg::kQuadCheckpointMagic) cp.fusion_revision = revision;
    }
}
void InsHistory::retireJournal() {
    std::uint64_t oldest = UINT64_MAX;
    for (const auto &cp : checkpoints_)
        if (cp.magic == kfg::kQuadCheckpointMagic && cp.sample_time_us < oldest) oldest=cp.sample_time_us;
    /* Only reclaim records before every usable checkpoint. Retained payloads
       are compacted in place; no replay runs concurrently with this owner. */
    std::uint32_t count=0;
    for(std::uint32_t i=0;i<journal_count_;++i) {
        const auto record=journal(i);
        if(oldest==UINT64_MAX || record.sample_time_us+550u>=oldest) {
            const auto slot=(journal_head_+kJournalSlots-journal_count_+count)%kJournalSlots;
            journal_[slot]=record; ++count;
        }
    }
    journal_head_=(journal_head_+kJournalSlots-journal_count_+count)%kJournalSlots;
    journal_count_=count;
}
bool InsHistory::nearestImuTime(std::uint64_t raw,std::uint64_t *boundary) const {
    if(!boundary||!imu_count_)return false;
    std::uint64_t t=imu_base_time_us_,distance=UINT64_MAX,best=0;
    for(std::uint32_t i=0;i<imu_count_;++i){
        const auto d=t>raw?t-raw:raw-t;
        if(d<distance){distance=d;best=t;}
        if(i+1u<imu_count_)t+=imuSlot(i+1u).dt_us;
    }
    *boundary=best;return distance<=550u;
}

/* ------------------------------------------------------------------------- */
/* ReplayEngine                                                              */
/* ------------------------------------------------------------------------- */

ReplayEngine::ReplayEngine() = default;

bool ReplayEngine::observationBefore(const ObservationRecord &lhs, const ObservationRecord &rhs) {
    return observationLess(lhs, rhs);
}

kfg::Status ReplayEngine::replay(const InsHistory &history, std::uint32_t checkpoint_index,
                                 std::uint64_t target_time_us, std::uint32_t target_imu_sequence,
                                 std::uint32_t base_fusion_revision, kfg::QuadCore &shadow,
                                 ReplayCandidate *candidate) const {
    if (candidate == nullptr) {
        return kfg::kErrArgument;
    }
    if (history.checkpointCount() == 0u) {
        return kfg::kErrArgument;
    }

    const kfg::QuadCheckpoint &base = history.checkpoint(checkpoint_index);
    candidate->shadow_only=false;
    for(auto &evaluation:candidate->latest_evaluation) evaluation=ObservationEvaluation{};
    std::memset(candidate->accepted_sample_time_us,0,sizeof(candidate->accepted_sample_time_us));
    if (base.sample_time_us >= target_time_us) {
        return kfg::kErrArgument; /* nothing to replay */
    }
    kfg::Status status = shadow.restoreCheckpoint(base);
    if (status != kfg::kOk) {
        return status;
    }

    /* Collect and order the journal observations inside the replay window. */
    std::uint32_t order[kJournalSlots];
    std::uint64_t quantized[kJournalSlots];
    std::uint32_t count = 0u;
    for (std::uint32_t i = 0; i < history.journalCount(); ++i) {
        const ObservationRecord &record = history.journal(i);
        if ((record.sample_time_us > base.sample_time_us) &&
            (record.sample_time_us <= target_time_us)) {
            if(!history.nearestImuTime(record.sample_time_us,&quantized[i])||
               quantized[i]<=base.sample_time_us)return kfg::kErrTimeDiscontinuity;
            order[count++] = i;
        }
    }
    /* Insertion sort: at most 64 entries, deterministic order. */
    for (std::uint32_t i = 1u; i < count; ++i) {
        const std::uint32_t key = order[i];
        std::uint32_t j = i;
        while ((j > 0u) && observationLess(history.journal(order[j - 1u]), history.journal(key))) {
            order[j] = order[j - 1u];
            --j;
        }
        order[j] = key;
    }
    /* The loop above sorted descending; reverse it into ascending order. */
    for (std::uint32_t i = 0u; i < count / 2u; ++i) {
        const std::uint32_t tmp = order[i];
        order[i] = order[count - 1u - i];
        order[count - 1u - i] = tmp;
    }

    /* Walk the retained IMU slots after the checkpoint up to the target. */
    std::uint32_t next_observation = 0u;
    std::uint32_t applied = 0u;
    std::uint32_t last_sequence = base.imu_sequence;

    for (std::uint32_t i = 0; i < history.imuCount(); ++i) {
        const std::uint64_t sample_time_us = history.imuSampleTime(i);
        if (sample_time_us <= base.sample_time_us) {
            continue;
        }
        if (sample_time_us > target_time_us) {
            break;
        }

        const ImuSlot &slot = history.imuSlot(i);
        kfg::ImuIncrement increment;
        increment.dt = static_cast<double>(slot.dt_us) * 1.0e-6;
        increment.dtheta = kfg::vec3(slot.dtheta[0], slot.dtheta[1], slot.dtheta[2]);
        increment.dvel = kfg::vec3(slot.dvel[0], slot.dvel[1], slot.dvel[2]);
        status = shadow.propagate(increment);
        if (status != kfg::kOk) {
            return status;
        }
        last_sequence = slot.sequence;

        /* Apply every observation quantized to this boundary, in order. */
        while (next_observation < count) {
            ObservationRecord record = history.journal(order[next_observation]);
            /* Nearest IMU endpoint, tie to the earlier endpoint. Never split
               coning/sculling intervals or rewrite the raw sample epoch. */
            const auto nearest=quantized[order[next_observation]];
            if(nearest<sample_time_us) return kfg::kErrTimeDiscontinuity;
            if(nearest>sample_time_us) break;
            if(record.id.kind==ObservationKind::kGnssVelocity) {
                for(int axis=0;axis<3;++axis)
                    record.omega[axis]=(increment.dtheta(axis,0)/increment.dt-shadow.nominal().gyrbias(axis,0))/
                        (1.0+shadow.nominal().gyrscale(axis,0));
            }
            const kfg::Status observation_status = applyObservation(shadow, record);
            if(observation_status!=kfg::kOk && observation_status!=kfg::kErrNisRejected)
                return observation_status;
            {
                unsigned kind=static_cast<unsigned>(record.id.kind);
                if(kind>=1 && kind<=4) candidate->latest_evaluation[kind-1]={
                    record.id.source_id,record.id.source_sequence,record.policy_epoch,
                    record.sample_time_us,observation_status==kfg::kOk};
            }
            if (observation_status == kfg::kOk) {
                ++applied;
                const auto kind=static_cast<unsigned>(record.id.kind);
                if(kind>=1u&&kind<=4u)candidate->accepted_sample_time_us[kind-1u]=record.sample_time_us;
            }
            ++next_observation;
        }
    }

    if (last_sequence != target_imu_sequence || next_observation != count) {
        return kfg::kErrTimeDiscontinuity;
    }

    status = shadow.saveCheckpoint(&candidate->checkpoint, base_fusion_revision, base.origin_id,
                                   base.state_epoch, target_time_us, target_imu_sequence);
    if (status != kfg::kOk) {
        return status;
    }
    candidate->base_fusion_revision = base_fusion_revision;
    candidate->journal_high_watermark = history.journalHighWatermark();
    candidate->applied_observations = applied;
    candidate->target_imu_sequence = target_imu_sequence;
    return kfg::kOk;
}

kfg::Status ReplayCursor::begin(const InsHistory &h, std::uint32_t index,
                                std::uint32_t revision, kfg::QuadCore &shadow,
                                const ObservationRecord *probes,unsigned probe_count,
                                std::uint64_t target) {
    cancel();
    if(index>=h.checkpointCount() || probe_count>2 || (probe_count && !probes)) return kfg::kErrArgument;
    const auto &base=h.checkpoint(index);
    auto result=shadow.restoreCheckpoint(base);
    if(result!=kfg::kOk) return result;
    count_=next_=applied_=0; revision_=revision; watermark_=h.journalHighWatermark();
    sequence_=base.imu_sequence; base_time_=time_=base.sample_time_us;
    origin_=base.origin_id; state_=base.state_epoch;
    std::memset(accepted_,0,sizeof(accepted_));
    for(auto &evaluation:latest_evaluation_) evaluation=ObservationEvaluation{};
    probe_count_=probe_count;
    for(unsigned i=0;i<probe_count;++i) probes_[i]=probes[i];
    for(std::uint32_t i=0;i<h.journalCount()+probe_count;++i) {
        unsigned slot=i<h.journalCount() ? i : kJournalSlots+i-h.journalCount();
        const auto &r=record(h,slot);
        if(r.sample_time_us<=base_time_) continue;
        if(!h.imuCount() || r.sample_time_us>h.imuSampleTime(h.imuCount()-1))
            return kfg::kErrTimeDiscontinuity;
        std::uint64_t endpoint=0;
        if(!h.nearestImuTime(r.sample_time_us,&endpoint) || endpoint<=base_time_)
            return kfg::kErrTimeDiscontinuity;
        if(endpoint>target) continue;
        std::uint32_t j=count_;
        while(j && observationLess(r,record(h,order_[j-1]))) {
            order_[j]=order_[j-1]; quantized_[j]=quantized_[j-1]; --j;
        }
        order_[j]=slot; quantized_[j]=endpoint; ++count_;
    }
    active_=true; return kfg::kOk;
}
kfg::Status ReplayCursor::step(const InsHistory &h,std::uint64_t target,
                               std::uint32_t target_sequence,kfg::QuadCore &shadow,
                               ReplayCandidate *out,bool *done) {
    if(!active_ || !out || !done) return kfg::kErrArgument;
    *done=false;
    if(watermark_!=h.journalHighWatermark() || target<time_) return kfg::kErrTimeDiscontinuity;
    if(next_<count_ && quantized_[next_]<=time_) {
        if(quantized_[next_]!=time_ || !increment_.dt) return kfg::kErrTimeDiscontinuity;
        auto observation=record(h,order_[next_]);
        if(observation.id.kind==ObservationKind::kGnssVelocity)
            for(unsigned axis=0;axis<3;++axis)
                observation.omega[axis]=(increment_.dtheta(axis,0)/increment_.dt-shadow.nominal().gyrbias(axis,0))/
                    (1.0+shadow.nominal().gyrscale(axis,0));
        auto result=applyObservation(shadow,observation);
        if(result!=kfg::kOk && result!=kfg::kErrNisRejected) return result;
        const auto kind=static_cast<unsigned>(observation.id.kind);
        if(kind>=1 && kind<=4) latest_evaluation_[kind-1]={observation.id.source_id,
            observation.id.source_sequence,observation.policy_epoch,observation.sample_time_us,result==kfg::kOk};
        if(result==kfg::kOk) {
            ++applied_;
            if(kind>=1 && kind<=4) accepted_[kind-1]=observation.sample_time_us;
        }
        ++next_; return kfg::kOk;
    }
    if(time_==target) {
        if(sequence_!=target_sequence || next_!=count_) return kfg::kErrTimeDiscontinuity;
        auto result=shadow.saveCheckpoint(&out->checkpoint,revision_,origin_,state_,time_,sequence_);
        if(result!=kfg::kOk) return result;
        out->base_fusion_revision=revision_; out->journal_high_watermark=watermark_;
        out->applied_observations=applied_; out->target_imu_sequence=sequence_;
        std::memcpy(out->accepted_sample_time_us,accepted_,sizeof(accepted_));
        for(unsigned i=0;i<4;++i) out->latest_evaluation[i]=latest_evaluation_[i];
        out->shadow_only=probe_count_!=0;
        *done=true; cancel(); return kfg::kOk;
    }
    /* Search by physical epoch each slice; the IMU ring may advance between
     * calls, so a saved logical ring index would silently select a new slot. */
    for(std::uint32_t i=0;i<h.imuCount();++i) {
        auto t=h.imuSampleTime(i);
        if(t<=time_) continue;
        const auto &s=h.imuSlot(i);
        if(t>target || t-time_!=s.dt_us) return kfg::kErrTimeDiscontinuity;
        increment_.dt=s.dt_us*1e-6;
        increment_.dtheta=kfg::vec3(s.dtheta[0],s.dtheta[1],s.dtheta[2]);
        increment_.dvel=kfg::vec3(s.dvel[0],s.dvel[1],s.dvel[2]);
        auto result=shadow.propagate(increment_);
        if(result!=kfg::kOk) return result;
        time_=t; sequence_=s.sequence; return kfg::kOk;
    }
    return kfg::kErrTimeDiscontinuity;
}

kfg::Status ReplayEngine::commit(kfg::QuadCore &active, const ReplayCandidate &candidate,
                                 std::uint32_t current_fusion_revision,
                                 std::uint32_t last_imu_sequence) {
    if (candidate.shadow_only || candidate.base_fusion_revision != current_fusion_revision) {
        return kfg::kErrArgument; /* stale candidate: a newer commit happened */
    }
    if (candidate.target_imu_sequence != last_imu_sequence) {
        return kfg::kErrTimeDiscontinuity; /* not at the last completed boundary */
    }
    return active.restoreCheckpoint(candidate.checkpoint);
}

} // namespace ins
