#pragma once
#include "sensors/sensors.hpp"

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace ad::perception {

// ---------------------------------------------------------------------------
// Lightweight 2×2 matrix — avoids pulling in control/linalg.hpp here.
// ---------------------------------------------------------------------------
struct M2 {
    double e[4];  // row-major [m00, m01, m10, m11]

    M2& operator+=(const M2& o) {
        for (int i = 0; i < 4; ++i) e[i] += o.e[i];
        return *this;
    }
    M2 operator+(const M2& o) const { M2 r = *this; r += o; return r; }

    M2 inv() const {
        double det = e[0]*e[3] - e[1]*e[2];
        if (std::abs(det) < 1e-18) det = (det >= 0 ? 1e-18 : -1e-18);
        return {{ e[3]/det, -e[1]/det, -e[2]/det, e[0]/det }};
    }

    // M2 @ vec2
    std::array<double,2> mv(double v0, double v1) const {
        return { e[0]*v0 + e[1]*v1, e[2]*v0 + e[3]*v1 };
    }
};

static inline M2 m2_from_R(const double R[4]) {
    return {{ R[0], R[1], R[2], R[3] }};
}

// ---------------------------------------------------------------------------
// FusedDetection — output of information-filter fusion across a cluster.
// ---------------------------------------------------------------------------
struct FusedDetection {
    double z[2] = {0.0, 0.0};   // fused world-frame position [m]
    double R[4] = {0.0,0.0,0.0,0.0};  // fused 2×2 covariance
    std::string kind;            // most common camera class label, or ""

    bool   has_v   = false;
    double vx = 0.0, vy = 0.0;  // velocity prior from radar Doppler
    std::vector<std::string> sensors;
};

// ---------------------------------------------------------------------------
// Mahalanobis gating — squared distance with combined covariance.
// gate = χ²(2 DOF, p=0.99) ≈ 9.21
// ---------------------------------------------------------------------------
static inline double mahal2(const ad::sensors::Detection& a,
                             const ad::sensors::Detection& b)
{
    M2 S = (m2_from_R(a.R) + m2_from_R(b.R)).inv();
    double dz0 = a.z[0] - b.z[0], dz1 = a.z[1] - b.z[1];
    auto Sv = S.mv(dz0, dz1);
    return dz0*Sv[0] + dz1*Sv[1];
}

// Greedy single-linkage clustering — O(n²).
static inline std::vector<std::vector<int>>
cluster(const std::vector<ad::sensors::Detection>& dets, double gate = 9.21)
{
    const int n = static_cast<int>(dets.size());
    std::vector<int> labels(n);
    for (int i = 0; i < n; ++i) labels[i] = i;

    for (int i = 0; i < n; ++i)
        for (int j = i+1; j < n; ++j)
            if (labels[i] != labels[j] && mahal2(dets[i], dets[j]) < gate) {
                int old_lab = labels[j];
                for (int k = 0; k < n; ++k)
                    if (labels[k] == old_lab) labels[k] = labels[i];
            }

    // Group by canonical label
    std::vector<std::vector<int>> groups;
    std::vector<int> seen;
    for (int i = 0; i < n; ++i) {
        bool found = false;
        for (int s = 0; s < static_cast<int>(seen.size()); ++s)
            if (seen[s] == labels[i]) { groups[s].push_back(i); found = true; break; }
        if (!found) { seen.push_back(labels[i]); groups.push_back({i}); }
    }
    return groups;
}

// ---------------------------------------------------------------------------
// Information-filter fusion over a cluster of detections.
// Radar Doppler → velocity prior via LOS projection.
// ---------------------------------------------------------------------------
static inline FusedDetection fuse_cluster(
    const std::vector<ad::sensors::Detection>& dets,
    const std::vector<int>& idx)
{
    M2    info = {{0,0,0,0}};
    double infz[2] = {0.0, 0.0};
    FusedDetection fd;

    double vx_sum = 0.0, vy_sum = 0.0;
    int    radar_count = 0;

    for (int i : idx) {
        const auto& d = dets[i];
        M2 Ri = m2_from_R(d.R).inv();
        info += Ri;
        auto Rz = Ri.mv(d.z[0], d.z[1]);
        infz[0] += Rz[0]; infz[1] += Rz[1];

        // Accumulate sensor tags
        fd.sensors.push_back(d.sensor);

        // Best kind from camera
        if (d.has_kind && fd.kind.empty()) fd.kind = d.kind;

        // Radar Doppler → (vx, vy) estimate via LOS direction at fused position
        if (d.has_vr) {
            double ux = d.z[0], uy = d.z[1];  // LOS from origin (approximate)
            double r  = std::hypot(ux, uy);
            if (r > 1e-3) { ux /= r; uy /= r; }
            vx_sum += d.vr * ux;
            vy_sum += d.vr * uy;
            ++radar_count;
        }
    }

    M2 cov = info.inv();
    auto z  = cov.mv(infz[0], infz[1]);
    fd.z[0] = z[0]; fd.z[1] = z[1];
    fd.R[0] = cov.e[0]; fd.R[1] = cov.e[1];
    fd.R[2] = cov.e[2]; fd.R[3] = cov.e[3];

    if (radar_count > 0) {
        fd.has_v = true;
        fd.vx = vx_sum / radar_count;
        fd.vy = vy_sum / radar_count;
    }
    return fd;
}

// ---------------------------------------------------------------------------
// Public API: cluster + fuse a raw detection list.
// ---------------------------------------------------------------------------
inline std::vector<FusedDetection>
fuse(const std::vector<ad::sensors::Detection>& dets, double gate = 9.21)
{
    if (dets.empty()) return {};
    auto groups = cluster(dets, gate);
    std::vector<FusedDetection> out;
    out.reserve(groups.size());
    for (const auto& g : groups)
        out.push_back(fuse_cluster(dets, g));
    return out;
}

// Convenience wrapper: measure + fuse in one call.
inline std::vector<FusedDetection>
perceive(const ad::sensors::SensorSuite& suite,
         double ego_x, double ego_y, double ego_psi,
         const std::vector<ad::world::ObjectEntry>& objects,
         double gate = 9.21)
{
    return fuse(suite.measure(ego_x, ego_y, ego_psi, objects), gate);
}

}  // namespace ad::perception
