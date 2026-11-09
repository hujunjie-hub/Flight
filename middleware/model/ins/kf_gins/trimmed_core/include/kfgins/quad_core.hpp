/*
 * KF-GINS_QUAD trimmed 22-state core.
 *
 * The propagation and GNSS-position mathematics are a fixed-dimension port of
 * the locked upstream files:
 *   src/kf-gins/gi_engine.cpp  (insPropagation / EKFPredict / EKFUpdate /
 *                               gnssUpdate / stateFeedback)
 *   src/kf-gins/insmech.cpp    (insMech / velUpdate / posUpdate / attUpdate)
 * commit 8291a93e49de513fe9d21f819500d39082ded611 (GPL-3.0).
 *
 * Controlled modifications (full list in ../../MODIFICATIONS.md):
 *   - 21 -> 22 error states, 18 -> 19 process-noise inputs (baro bias);
 *   - Eigen dynamic matrices replaced by fixed-size storage, no heap;
 *   - S.inverse() replaced by Cholesky based SPD solves;
 *   - std::exit()/console output replaced by status codes and counters;
 *   - GNSS velocity and barometer observations added (project extension);
 *   - NIS gating added around every observation.
 */

#ifndef KFGINS_QUAD_CORE_HPP
#define KFGINS_QUAD_CORE_HPP

#include "quad_math.hpp"
#include "quad_types.hpp"

namespace kfg {

constexpr std::size_t kCovarianceUpperCount = kRank22 * (kRank22 + 1) / 2;
constexpr std::size_t covarianceUpperDiagonalIndex(std::size_t row) {
    return row * (2 * kRank22 + 1 - row) / 2;
}

/*
 * 2304-byte checkpoint on the delta_x = 0 boundary (spec 05 §5.2, spec 07 §6).
 *
 * Layout: 32 B header + 7 doubles of the previous compensated IMU increment
 * (the coning/sculling "previous value") + 23 doubles of nominal state
 * (euler is recomputed from the quaternion) + the 253 doubles of the upper
 * triangle of P.  Total 2296 B, padded to the fixed 2304 B slot.
 *
 * The checkpoint is a byte-stable POD so a replay candidate can be copied and
 * compared without touching the active filter.
 */
struct QuadCheckpoint {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t flags;
    std::uint32_t fusion_revision;
    std::uint32_t imu_sequence;
    std::uint32_t origin_id;
    std::uint32_t state_epoch;
    std::uint64_t sample_time_us;

    double previous_dt;
    double previous_dtheta[3];
    double previous_dvel[3];

    double nominal[23];          /* pos3 vel3 quat4 bg3 ba3 sg3 sa3 bh1 */
    double covariance_upper[kCovarianceUpperCount]; /* row-major upper triangle */
    std::uint8_t reserved[8];
};

constexpr std::uint32_t kQuadCheckpointMagic = 0x4B464743u; /* "KFGC" */
constexpr std::uint16_t kQuadCheckpointVersion = 1u;
constexpr std::size_t kQuadCheckpointBytes = 2304u;

static_assert(sizeof(QuadCheckpoint) == kQuadCheckpointBytes,
              "checkpoint must fit the fixed 2304-byte slot");


class QuadCore {
public:
    QuadCore();

    /*
     * Validates the configuration and builds the 22x22 initial covariance from
     * the one-sigma inputs.  Must be called once before any propagation.
     */
    Status initialize(const QuadConfig &config, const QuadInitState &init);

    /*
     * One INS propagation step over one IMU increment.  The increment is
     * compensated with the current bias/scale estimates exactly like the
     * upstream GIEngine::insPropagation() does for 'imucur'.
     */
    Status propagate(const ImuIncrement &increment);

    /*
     * GNSS antenna position update (upstream gnssUpdate plus NIS gate).
     * 'obs.blh' is the antenna phase-centre geodetic position; the IMU to
     * antenna lever arm of the configuration is applied internally.
     */
    Status updateGnssPosition(const GnssPositionObs &obs);

    /*
     * GNSS antenna velocity update.  The observation model includes the lever
     * arm rotation term and the navigation-frame rotation term of spec 05
     * §3.2; 'obs.omega_ib' is the time-aligned, bias/scale compensated body
     * angular rate at the measurement epoch.
     */
    Status updateGnssVelocity(const GnssVelocityObs &obs);

    /*
     * Barometer relative height update.  'obs.height' is the caller-computed
     * relative pressure height h_baro - h_baro0; the local origin ellipsoid
     * height is configured through QuadConfig::baro_origin_height_m.
     */
    Status updateBaro(const BaroObs &obs);
    Status updateHeading(double heading_rad, double variance_rad2);

    const QuadNominal &nominal() const { return nominal_; }
    const Mat22 &covariance() const { return covariance_; }
    const QuadConfig &config() const { return config_; }
    const QuadDiagnostics &diagnostics() const { return diagnostics_; }
    bool initialized() const { return initialized_; }

    /* Copies the full covariance into 'out' (row-major 22x22). */
    void copyCovariance(double *out) const;

    /* Uses the core's existing scratch and leaves the current covariance intact. */
    bool validateInitialCovariance(const double (&upper)[kCovarianceUpperCount]);
    /* Requires a validated positive-definite upper triangle. The floor may only raise yaw variance. */
    Status setInitialCovariance(const double (&upper)[kCovarianceUpperCount],
                                double heading_variance_floor);

    /* Test/diagnostic helper: overwrite the covariance (does not reinitialise). */
    Status setCovarianceForTest(const Mat22 &cov);

    /*
     * Checkpoint on the current delta_x = 0 boundary (spec 05 §5.2).  The
     * metadata owned by M3 is passed through so the checkpoint is
     * self-describing for replay (fusion revision, origin/state epoch, IMU
     * boundary and sequence).
     */
    Status saveCheckpoint(QuadCheckpoint *out, std::uint32_t fusion_revision, std::uint32_t origin_id,
                          std::uint32_t state_epoch, std::uint64_t sample_time_us,
                          std::uint32_t imu_sequence) const;

    /* Restores a checkpoint into this core; the error state is reset to zero
     * because checkpoints only exist on a feedback boundary. */
    Status restoreCheckpoint(const QuadCheckpoint &in);

private:
    enum UpdateKind { kUpdateGnssPosition = 0, kUpdateGnssVelocity = 1, kUpdateBaro = 2, kUpdateHeading = 3 };

    struct Workspace {
        Mat22 phi;     /* state transition, then reused as K*H*P during updates */
        Mat22 scratch; /* temporary product / covariance backup */
        Mat22x3 k;     /* Kalman gain (22x3) */
        Mat22x3 ks;    /* K*S (22x3) */
        Mat3x22 hp;    /* H*P (3x22) */
        Mat3 s;        /* innovation covariance */
        Mat3 l;        /* Cholesky factor of s */
    };

    Status propagateCovariance(const QuadNominal &pre, const ImuIncrement &compensated);
    void buildPhi(const QuadNominal &pre, const ImuIncrement &compensated);
    Status applyUpdate(int m, const double *dz, const double *h, const double *r_noise, int dof,
                       UpdateKind kind);

    QuadConfig config_;
    QuadNominal nominal_;
    Mat22 covariance_;
    Vec22 dx_;
    ImuIncrement previous_increment_; /* compensated, upstream 'imupre_' */
    bool has_previous_increment_;
    bool initialized_;
    QuadDiagnostics diagnostics_;
    Workspace workspace_;
};

} // namespace kfg

#endif // KFGINS_QUAD_CORE_HPP
