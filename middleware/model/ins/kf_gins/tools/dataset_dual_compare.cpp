/*
 * Project Host Reference vs Pristine Upstream on the official KF-GINS data set.
 *
 * Drives the pristine upstream GIEngine and the QUAD trimmed 21-state core with
 * the *same* loop, the same increments and the same GNSS measurements, then
 * reports the per-epoch difference.  This isolates implementation equivalence
 * from file-format/loop plumbing: both engines receive identical inputs.
 *
 * Host-only tool.  Requires a checkout of the locked upstream commit for the
 * Eigen headers and the gi_engine/insmech sources (not vendored here).
 *
 * Build (from the Flight root):
 *   g++ -std=c++14 -O2 -D_USE_MATH_DEFINES \
 *       -I <upstream>/src -I <upstream>/ThirdParty/eigen-3.3.9 \
 *       -I middleware/model/ins/kf_gins/trimmed_core/include \
 *       middleware/model/ins/kf_gins/tools/dataset_dual_compare.cpp \
 *       <upstream>/src/kf-gins/gi_engine.cpp <upstream>/src/kf-gins/insmech.cpp \
 *       middleware/model/ins/kf_gins/trimmed_core/src/quad_core.cpp middleware/model/ins/kf_gins/trimmed_core/src/quad_models.cpp \
 *       -o dataset_dual_compare.exe
 *
 * Usage:
 *   dataset_dual_compare <kf-gins.yaml>     # paths inside the yaml are relative to CWD
 */
/*
 * Temporary diagnostic (not shipped): drive the pristine upstream engine and
 * the QUAD core with the identical dataset loop and report the first
 * divergence.  Build with the upstream sources and Eigen include.
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "common/types.h"
#include "kf-gins/gi_engine.h"

#include "kfgins/quad_core.hpp"
#include "kfgins/quad_models.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;

struct Cfg {
    std::map<std::string, std::vector<double>> num;
    std::map<std::string, std::string> str;
    std::vector<double> v3(const std::string &k, std::vector<double> fb) {
        auto it = num.find(k);
        return (it == num.end() || it->second.size() < 3u) ? fb : it->second;
    }
    double s(const std::string &k, double fb) {
        auto it = num.find(k);
        return (it == num.end() || it->second.empty()) ? fb : it->second[0];
    }
};

std::string trim(std::string s) {
    const char *ws = " \t\r\n";
    auto b = s.find_first_not_of(ws);
    if (b == std::string::npos) return std::string();
    auto e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

std::vector<double> nums(std::string s) {
    for (char &c : s) if (c == '[' || c == ']' || c == ',') c = ' ';
    std::vector<double> v; std::istringstream is(s); double x;
    while (is >> x) v.push_back(x);
    return v;
}

void loadCfg(const char *path, Cfg &cfg) {
    std::ifstream f(path); std::string line, section;
    while (std::getline(f, line)) {
        auto h = line.find('#'); if (h != std::string::npos) line = line.substr(0, h);
        line = trim(line); if (line.empty()) continue;
        auto c = line.find(':'); if (c == std::string::npos) continue;
        std::string k = trim(line.substr(0, c)), v = trim(line.substr(c + 1));
        if (v.empty()) { section = k; continue; }
        std::string key = section.empty() ? k : section + "." + k;
        if (v[0] == '[') cfg.num[key] = nums(v);
        else if (v[0] == '"') cfg.str[key] = v.substr(1, v.size() - 2);
        else cfg.num[key] = nums(v);
    }
}

struct ImuRec { double t, dt, dth[3], dvel[3]; };
struct GnssRec { double t, blh[3], std[3]; };

class ImuFile {
public:
    explicit ImuFile(const std::string &p) { f.open(p); prev = 0; }
    bool next(ImuRec &r) {
        std::string line;
        if (!std::getline(f, line)) return false;
        std::istringstream is(line); std::vector<double> v; double x;
        while (is >> x) v.push_back(x);
        if (v.size() < 7) return false;
        r.t = v[0];
        for (int i = 0; i < 3; ++i) { r.dth[i] = v[1 + i]; r.dvel[i] = v[4 + i]; }
        double dt = r.t - prev; r.dt = (dt < 0.1) ? dt : 0.005; prev = r.t;
        return true;
    }
private:
    std::ifstream f; double prev;
};

class GnssFile {
public:
    explicit GnssFile(const std::string &p) { f.open(p); }
    bool next(GnssRec &r) {
        std::string line;
        if (!std::getline(f, line)) return false;
        std::istringstream is(line); std::vector<double> v; double x;
        while (is >> x) v.push_back(x);
        if (v.size() < 7) return false;
        r.t = v[0]; r.blh[0] = v[1] * kD2R; r.blh[1] = v[2] * kD2R; r.blh[2] = v[3];
        r.std[0] = v[4]; r.std[1] = v[5]; r.std[2] = v[6];
        return true;
    }
private:
    std::ifstream f;
};

int toUpdate(double t1, double t2, double u) {
    if (std::fabs(t1 - u) < 0.001) return 1;
    if (std::fabs(t2 - u) <= 0.001) return 2;
    if (t1 < u && u < t2) return 3;
    return 0;
}

bool interp(const ImuRec &p, ImuRec &c, double ts, ImuRec &m) {
    if (p.t > ts || c.t < ts) return false;
    double lam = (ts - p.t) / (c.t - p.t);
    m.t = ts;
    for (int i = 0; i < 3; ++i) { m.dth[i] = c.dth[i] * lam; m.dvel[i] = c.dvel[i] * lam; }
    m.dt = ts - p.t;
    for (int i = 0; i < 3; ++i) { c.dth[i] -= m.dth[i]; c.dvel[i] -= m.dvel[i]; }
    c.dt -= m.dt;
    return true;
}

IMU toImu(const ImuRec &r) {
    IMU imu; imu.time = r.t; imu.dt = r.dt;
    imu.dtheta = Eigen::Vector3d(r.dth[0], r.dth[1], r.dth[2]);
    imu.dvel = Eigen::Vector3d(r.dvel[0], r.dvel[1], r.dvel[2]);
    imu.odovel = 0.0;
    return imu;
}

kfg::ImuIncrement toInc(const ImuRec &r) {
    kfg::ImuIncrement i; i.dt = r.dt;
    i.dtheta = kfg::vec3(r.dth[0], r.dth[1], r.dth[2]);
    i.dvel = kfg::vec3(r.dvel[0], r.dvel[1], r.dvel[2]);
    return i;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: %s config.yaml\n", argv[0]); return 2; }
    Cfg cfg; loadCfg(argv[1], cfg);

    std::vector<double> ip = cfg.v3("initpos", {0, 0, 0}), iv = cfg.v3("initvel", {0, 0, 0}),
                        ia = cfg.v3("initatt", {0, 0, 0});
    std::vector<double> gb = cfg.v3("initgyrbias", {0, 0, 0}), ab = cfg.v3("initaccbias", {0, 0, 0});
    std::vector<double> gs = cfg.v3("initgyrscale", {0, 0, 0}), as = cfg.v3("initaccscale", {0, 0, 0});
    std::vector<double> ps = cfg.v3("initposstd", {1, 1, 1}), vs = cfg.v3("initvelstd", {1, 1, 1}),
                        ats = cfg.v3("initattstd", {0.1, 0.1, 0.1});
    std::vector<double> arw = cfg.v3("imunoise.arw", {0, 0, 0}), vrw = cfg.v3("imunoise.vrw", {0, 0, 0});
    std::vector<double> gbs = cfg.v3("imunoise.gbstd", {0, 0, 0}), abs_ = cfg.v3("imunoise.abstd", {0, 0, 0});
    std::vector<double> gss = cfg.v3("imunoise.gsstd", {0, 0, 0}), ass = cfg.v3("imunoise.asstd", {0, 0, 0});
    double corr = cfg.s("imunoise.corrtime", 1.0);
    std::vector<double> lever = cfg.v3("antlever", {0, 0, 0});
    double start = cfg.s("starttime", 0.0), end = cfg.s("endtime", -1.0);

    GINSOptions opt;
    for (int i = 0; i < 3; ++i) {
        opt.initstate.pos[i] = ip[i] * kD2R;
        opt.initstate.vel[i] = iv[i];
        opt.initstate.euler[i] = ia[i] * kD2R;
        opt.initstate_std.pos[i] = ps[i];
        opt.initstate_std.vel[i] = vs[i];
        opt.initstate_std.euler[i] = ats[i] * kD2R;
        opt.initstate.imuerror.gyrbias[i] = gb[i] * kD2R / 3600.0;
        opt.initstate.imuerror.accbias[i] = ab[i] * 1e-5;
        opt.initstate.imuerror.gyrscale[i] = gs[i] * 1e-6;
        opt.initstate.imuerror.accscale[i] = as[i] * 1e-6;
        opt.initstate_std.imuerror.gyrbias[i] = gbs[i] * kD2R / 3600.0;
        opt.initstate_std.imuerror.accbias[i] = abs_[i] * 1e-5;
        opt.initstate_std.imuerror.gyrscale[i] = gss[i] * 1e-6;
        opt.initstate_std.imuerror.accscale[i] = ass[i] * 1e-6;
        opt.imunoise.gyr_arw[i] = arw[i] * kD2R / 60.0;
        opt.imunoise.acc_vrw[i] = vrw[i] / 60.0;
        opt.imunoise.gyrbias_std[i] = gbs[i] * kD2R / 3600.0;
        opt.imunoise.accbias_std[i] = abs_[i] * 1e-5;
        opt.imunoise.gyrscale_std[i] = gss[i] * 1e-6;
        opt.imunoise.accscale_std[i] = ass[i] * 1e-6;
    }
    opt.initstate.pos[2] = ip[2];
    opt.imunoise.corr_time = corr * 3600.0;
    opt.antlever = Eigen::Vector3d(lever[0], lever[1], lever[2]);

    std::ostringstream sink;
    std::streambuf *saved = std::cout.rdbuf(sink.rdbuf());
    GIEngine engine(opt);
    std::cout.rdbuf(saved);

    kfg::QuadConfig qc;
    qc.imunoise.gyr_arw = kfg::vec3(arw[0] * kD2R / 60.0, arw[1] * kD2R / 60.0, arw[2] * kD2R / 60.0);
    qc.imunoise.acc_vrw = kfg::vec3(vrw[0] / 60.0, vrw[1] / 60.0, vrw[2] / 60.0);
    qc.imunoise.gyrbias_std = kfg::vec3(gbs[0] * kD2R / 3600.0, gbs[1] * kD2R / 3600.0, gbs[2] * kD2R / 3600.0);
    qc.imunoise.accbias_std = kfg::vec3(abs_[0] * 1e-5, abs_[1] * 1e-5, abs_[2] * 1e-5);
    qc.imunoise.gyrscale_std = kfg::vec3(gss[0] * 1e-6, gss[1] * 1e-6, gss[2] * 1e-6);
    qc.imunoise.accscale_std = kfg::vec3(ass[0] * 1e-6, ass[1] * 1e-6, ass[2] * 1e-6);
    qc.imunoise.corr_time = corr * 3600.0;
    qc.imunoise.baro_bias_walk = 0.0;
    qc.antlever = kfg::vec3(lever[0], lever[1], lever[2]);
    qc.nis_gate_enable = false;
    qc.baro_origin_height_m = 0.0;

    kfg::QuadInitState qi;
    qi.nominal.pos = kfg::vec3(ip[0] * kD2R, ip[1] * kD2R, ip[2]);
    qi.nominal.vel = kfg::vec3(iv[0], iv[1], iv[2]);
    qi.nominal.euler = kfg::vec3(ia[0] * kD2R, ia[1] * kD2R, ia[2] * kD2R);
    qi.nominal.cbn = kfg::eulerToMatrix(qi.nominal.euler);
    qi.nominal.qbn = kfg::matrixToQuat(qi.nominal.cbn);
    qi.nominal.gyrbias = kfg::vec3(gb[0] * kD2R / 3600.0, gb[1] * kD2R / 3600.0, gb[2] * kD2R / 3600.0);
    qi.nominal.accbias = kfg::vec3(ab[0] * 1e-5, ab[1] * 1e-5, ab[2] * 1e-5);
    qi.nominal.gyrscale = kfg::vec3(gs[0] * 1e-6, gs[1] * 1e-6, gs[2] * 1e-6);
    qi.nominal.accscale = kfg::vec3(as[0] * 1e-6, as[1] * 1e-6, as[2] * 1e-6);
    qi.nominal.baro_bias = 0.0;
    qi.pos_std = kfg::vec3(ps[0], ps[1], ps[2]);
    qi.vel_std = kfg::vec3(vs[0], vs[1], vs[2]);
    qi.euler_std = kfg::vec3(ats[0] * kD2R, ats[1] * kD2R, ats[2] * kD2R);
    qi.gyrbias_std = qc.imunoise.gyrbias_std;
    qi.accbias_std = qc.imunoise.accbias_std;
    qi.gyrscale_std = qc.imunoise.gyrscale_std;
    qi.accscale_std = qc.imunoise.accscale_std;
    qi.baro_bias_std = 1.0;

    kfg::QuadCore core;
    if (core.initialize(qc, qi) != kfg::kOk) { std::fprintf(stderr, "init failed\n"); return 1; }

    ImuFile imuf(cfg.str["imupath"]); GnssFile gnssf(cfg.str["gnsspath"]);
    ImuRec imu; do { if (!imuf.next(imu)) return 1; } while (imu.t < start);
    GnssRec gnss; do { if (!gnssf.next(gnss)) return 1; } while (gnss.t <= start);
    ImuRec prev = imu;
    bool gnss_valid = true;

    engine.addImuData(toImu(imu), true);
    GNSS g; g.time = gnss.t; g.blh = Eigen::Vector3d(gnss.blh[0], gnss.blh[1], gnss.blh[2]);
    g.std = Eigen::Vector3d(gnss.std[0], gnss.std[1], gnss.std[2]); g.isvalid = true;
    engine.addGnssData(g);

    long epoch = 0; double worst_pos = 0, worst_vel = 0, worst_att = 0; long first_bad = -1;

    while (true) {
        if (gnss.t < prev.t) { GnssRec n; if (gnssf.next(n)) { gnss = n; gnss_valid = true; } }

        ImuRec cur;
        if (!imuf.next(cur)) break;
        if (end >= 0 && cur.t > end) break;

        int act = toUpdate(prev.t, cur.t, gnss_valid ? gnss.t : -1.0);
        /* upstream */
        if (gnss_valid) {
            g.time = gnss.t; g.blh = Eigen::Vector3d(gnss.blh[0], gnss.blh[1], gnss.blh[2]);
            g.std = Eigen::Vector3d(gnss.std[0], gnss.std[1], gnss.std[2]); g.isvalid = true;
            engine.addGnssData(g);
        }
        if (act == 3) {
            ImuRec mid; interp(prev, cur, gnss.t, mid);
            IMU im; im.time = mid.t; im.dt = mid.dt;
            im.dtheta = Eigen::Vector3d(mid.dth[0], mid.dth[1], mid.dth[2]);
            im.dvel = Eigen::Vector3d(mid.dvel[0], mid.dvel[1], mid.dvel[2]); im.odovel = 0;
            engine.addImuData(im);
            engine.newImuProcess();
            kfg::GnssPositionObs o; o.blh = kfg::vec3(gnss.blh[0], gnss.blh[1], gnss.blh[2]);
            o.std = kfg::vec3(gnss.std[0], gnss.std[1], gnss.std[2]);
            core.propagate(toInc(mid));
            core.updateGnssPosition(o);
            gnss_valid = false;
            IMU im2 = toImu(cur); engine.addImuData(im2); engine.newImuProcess();
            core.propagate(toInc(cur));
        } else {
            IMU im = toImu(cur); engine.addImuData(im); engine.newImuProcess();
            if (act == 2) {
                kfg::GnssPositionObs o; o.blh = kfg::vec3(gnss.blh[0], gnss.blh[1], gnss.blh[2]);
                o.std = kfg::vec3(gnss.std[0], gnss.std[1], gnss.std[2]);
                core.propagate(toInc(cur));
                core.updateGnssPosition(o);
                gnss_valid = false;
            } else if (act == 1) {
                kfg::GnssPositionObs o; o.blh = kfg::vec3(gnss.blh[0], gnss.blh[1], gnss.blh[2]);
                o.std = kfg::vec3(gnss.std[0], gnss.std[1], gnss.std[2]);
                core.updateGnssPosition(o);
                gnss_valid = false;
                core.propagate(toInc(cur));
            } else {
                core.propagate(toInc(cur));
            }
        }
        prev = cur;

        NavState ref = engine.getNavState();
        const kfg::QuadNominal &n = core.nominal();
        double dlat = (n.pos(0, 0) - ref.pos[0]) * 6378137.0;
        double dlon = (n.pos(1, 0) - ref.pos[1]) * 6378137.0 * std::cos(ref.pos[0]);
        double dh = n.pos(2, 0) - ref.pos[2];
        double dp = std::sqrt(dlat * dlat + dlon * dlon + dh * dh);
        double dv = (n.vel - kfg::vec3(ref.vel[0], ref.vel[1], ref.vel[2])).d[0];
        (void)dv;
        double dvel = std::sqrt(std::pow(n.vel(0, 0) - ref.vel[0], 2) + std::pow(n.vel(1, 0) - ref.vel[1], 2) +
                                std::pow(n.vel(2, 0) - ref.vel[2], 2));
        double datt = std::max(std::fabs(n.euler(0, 0) - ref.euler[0]),
                               std::max(std::fabs(n.euler(1, 0) - ref.euler[1]),
                                        std::fabs(n.euler(2, 0) - ref.euler[2])));
        worst_pos = std::max(worst_pos, dp);
        worst_vel = std::max(worst_vel, dvel);
        worst_att = std::max(worst_att, datt);
        if (first_bad < 0 && (dp > 1e-6 || dvel > 1e-6)) {
            first_bad = epoch;
            std::printf("first divergence at epoch %ld (t=%.6f): dp=%.3e dvel=%.3e datt=%.3e\n", epoch, cur.t, dp, dvel, datt);
        }
        ++epoch;
    }
    std::printf("epochs=%ld worst dp=%.3e dvel=%.3e datt=%.3e first_bad=%ld\n", epoch, worst_pos, worst_vel, worst_att, first_bad);
    return 0;
}
