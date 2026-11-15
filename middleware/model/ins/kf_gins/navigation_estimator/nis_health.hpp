#ifndef FLIGHT_KF_GINS_NIS_HEALTH_HPP
#define FLIGHT_KF_GINS_NIS_HEALTH_HPP

#include <cstdint>

#include "kfgins/quad_types.hpp"

namespace ins {

/* A persistent NIS disagreement revokes the affected navigation capability.
 * It does not mark the GNSS source FAILED or alter SensorUsePolicy. The latch
 * is cleared only by an explicit, authorized estimator reset. */
class NisCapabilityLatch {
public:
    enum : std::uint8_t {
        kPositionLatched = 1u << 0,
        kVelocityLatched = 1u << 1
    };

    void reset() {
        position_latched_ = false;
        velocity_latched_ = false;
    }

    std::uint8_t update(std::uint8_t position_reject_streak,
                        std::uint8_t velocity_reject_streak) {
        std::uint8_t newly_latched = 0u;
        if (!position_latched_ &&
            position_reject_streak >= kfg::kNisConsecutiveRejectLimit) {
            position_latched_ = true;
            newly_latched |= kPositionLatched;
        }
        if (!velocity_latched_ &&
            velocity_reject_streak >= kfg::kNisConsecutiveRejectLimit) {
            velocity_latched_ = true;
            newly_latched |= kVelocityLatched;
        }
        return newly_latched;
    }

    bool positionUsable() const { return !position_latched_; }
    bool velocityUsable() const { return !velocity_latched_; }

private:
    bool position_latched_ = false;
    bool velocity_latched_ = false;
};

} // namespace ins

#endif // FLIGHT_KF_GINS_NIS_HEALTH_HPP
