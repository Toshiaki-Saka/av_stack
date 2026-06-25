#pragma once
#include "world/scenario.hpp"

#include <array>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace ad::sensors {

// ---------------------------------------------------------------------------
// Shared RNG — seed for reproducible simulation (default seed=0).
// ---------------------------------------------------------------------------
namespace detail {
    inline std::mt19937_64& rng() {
        static std::mt19937_64 g(0);
        return g;
    }
}

inline void seed_rng(uint64_t seed) { detail::rng().seed(seed); }

static inline double randn(double mu, double sigma) {
    return std::normal_distribution<double>(mu, sigma)(detail::rng());
}
static inline double randu() {
    return std::uniform_real_distribution<double>(0.0, 1.0)(detail::rng());
}

// ---------------------------------------------------------------------------
// Lightweight 2×2 matrix helpers (row-major: a b / c d).
// ---------------------------------------------------------------------------
struct Mat2 {
    double a, b, c, d;

    static Mat2 diag(double m00, double m11) { return {m00, 0.0, 0.0, m11}; }

    // R = Rot(angle) @ diag(s00, s11) @ Rot(angle)^T
    static Mat2 rotated_diag(double angle, double s00, double s11) {
        double co = std::cos(angle), si = std::sin(angle);
        // Rot @ diag
        double ra = co*s00, rb = -si*s11;
        double rc = si*s00, rd =  co*s11;
        // (Rot@diag) @ Rot^T
        return { ra*co + rb*si,  ra*(-si) + rb*co,
                 rc*co + rd*si,  rc*(-si) + rd*co };
    }
};

// ---------------------------------------------------------------------------
// Detection — output of a single sensor measurement.
// R[4] is stored row-major: [R00, R01, R10, R11].
// ---------------------------------------------------------------------------
struct Detection {
    double      z[2]   = {0.0, 0.0};   // world-frame position [m]
    double      R[4]   = {0.0,0.0,0.0,0.0};  // 2×2 covariance
    std::string sensor;                 // "lidar" | "radar" | "camera"
    bool        has_vr = false;
    double      vr     = 0.0;          // radial velocity (radar only) [m/s]
    bool        has_kind = false;
    std::string kind;                   // object class (camera only)
};

// ---------------------------------------------------------------------------
// Visibility check — returns true if within range and FOV.
// bearing is ego-frame angle relative to heading [rad].
// ---------------------------------------------------------------------------
static inline bool _visible(double ego_x, double ego_y, double ego_psi,
                             double ox, double oy,
                             double max_range, double half_fov,
                             double& r_out, double& bearing_out)
{
    double dx = ox - ego_x, dy = oy - ego_y;
    r_out = std::hypot(dx, dy);
    if (r_out > max_range || r_out < 1e-3) { bearing_out = 0.0; return false; }
    double b = std::atan2(dy, dx) - ego_psi;
    bearing_out = std::atan2(std::sin(b), std::cos(b));
    return std::abs(bearing_out) <= half_fov;
}

// ---------------------------------------------------------------------------
// Lidar — isotropic Gaussian position noise, σ=0.15 m.
// ---------------------------------------------------------------------------
class Lidar {
public:
    double max_range, half_fov, sigma, p_miss;

    explicit Lidar(double max_range_ = 80.0,
                   double half_fov_  = M_PI / 3.0,
                   double sigma_     = 0.15,
                   double p_miss_    = 0.05)
        : max_range(max_range_), half_fov(half_fov_),
          sigma(sigma_), p_miss(p_miss_) {}

    std::vector<Detection>
    measure(double ego_x, double ego_y, double ego_psi,
            const std::vector<ad::world::ObjectEntry>& objects) const
    {
        std::vector<Detection> out;
        for (const auto& obj : objects) {
            double r, bearing;
            if (!_visible(ego_x, ego_y, ego_psi, obj.x, obj.y,
                          max_range, half_fov, r, bearing)) continue;
            if (randu() < p_miss) continue;

            Detection d;
            d.z[0]   = obj.x + randn(0.0, sigma);
            d.z[1]   = obj.y + randn(0.0, sigma);
            d.R[0]   = sigma*sigma; d.R[1] = 0.0;
            d.R[2]   = 0.0;        d.R[3] = sigma*sigma;
            d.sensor = "lidar";
            out.push_back(d);
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// Radar — anisotropic noise aligned to the LOS direction.
// ---------------------------------------------------------------------------
class Radar {
public:
    double max_range, half_fov, sigma_r, sigma_lat, sigma_vr, p_miss;

    explicit Radar(double max_range_ = 120.0,
                   double half_fov_  = M_PI / 4.0,
                   double sigma_r_   = 0.4,
                   double sigma_lat_ = 1.2,
                   double sigma_vr_  = 0.1,
                   double p_miss_    = 0.05)
        : max_range(max_range_), half_fov(half_fov_),
          sigma_r(sigma_r_), sigma_lat(sigma_lat_),
          sigma_vr(sigma_vr_), p_miss(p_miss_) {}

    std::vector<Detection>
    measure(double ego_x, double ego_y, double ego_psi,
            const std::vector<ad::world::ObjectEntry>& objects) const
    {
        std::vector<Detection> out;
        for (const auto& obj : objects) {
            double r, bearing;
            if (!_visible(ego_x, ego_y, ego_psi, obj.x, obj.y,
                          max_range, half_fov, r, bearing)) continue;
            if (randu() < p_miss) continue;

            // World-frame LOS angle for radar R rotation
            double ang = std::atan2(obj.y - ego_y, obj.x - ego_x);
            double co  = std::cos(ang), si = std::sin(ang);

            double n_r   = randn(0.0, sigma_r);
            double n_lat = randn(0.0, sigma_lat);

            Detection d;
            d.z[0]   = obj.x + co*n_r - si*n_lat;
            d.z[1]   = obj.y + si*n_r + co*n_lat;
            Mat2 Rm  = Mat2::rotated_diag(ang, sigma_r*sigma_r, sigma_lat*sigma_lat);
            d.R[0] = Rm.a; d.R[1] = Rm.b; d.R[2] = Rm.c; d.R[3] = Rm.d;
            d.sensor = "radar";
            d.has_vr = true;
            d.vr     = obj.vx*co + obj.vy*si + randn(0.0, sigma_vr);
            out.push_back(d);
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// Camera — polar noise (bearing σ=0.6°, range σ_rel=10%).
// ---------------------------------------------------------------------------
class Camera {
public:
    double max_range, half_fov, sigma_bearing, range_rel, p_miss;

    explicit Camera(double max_range_     = 70.0,
                    double half_fov_      = 35.0 * M_PI / 180.0,
                    double sigma_bearing_ = 0.6  * M_PI / 180.0,
                    double range_rel_     = 0.10,
                    double p_miss_        = 0.04)
        : max_range(max_range_), half_fov(half_fov_),
          sigma_bearing(sigma_bearing_), range_rel(range_rel_), p_miss(p_miss_) {}

    std::vector<Detection>
    measure(double ego_x, double ego_y, double ego_psi,
            const std::vector<ad::world::ObjectEntry>& objects) const
    {
        std::vector<Detection> out;
        for (const auto& obj : objects) {
            double r, bearing;
            if (!_visible(ego_x, ego_y, ego_psi, obj.x, obj.y,
                          max_range, half_fov, r, bearing)) continue;
            if (randu() < p_miss) continue;

            double r_meas = r * (1.0 + randn(0.0, range_rel));
            double b_meas = bearing + randn(0.0, sigma_bearing);
            double ang    = ego_psi + b_meas;   // world-frame direction

            Detection d;
            d.z[0] = ego_x + r_meas * std::cos(ang);
            d.z[1] = ego_y + r_meas * std::sin(ang);

            double sr = range_rel * r, sl = r * sigma_bearing;
            Mat2 Rm = Mat2::rotated_diag(ang, sr*sr, sl*sl);
            d.R[0] = Rm.a; d.R[1] = Rm.b; d.R[2] = Rm.c; d.R[3] = Rm.d;
            d.sensor   = "camera";
            d.has_kind = true;
            d.kind     = obj.kind;
            out.push_back(d);
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// SensorSuite — aggregates all three sensors.
// ---------------------------------------------------------------------------
class SensorSuite {
public:
    Lidar  lidar;
    Radar  radar;
    Camera camera;

    std::vector<Detection>
    measure(double ego_x, double ego_y, double ego_psi,
            const std::vector<ad::world::ObjectEntry>& objects) const
    {
        auto l = lidar .measure(ego_x, ego_y, ego_psi, objects);
        auto r = radar .measure(ego_x, ego_y, ego_psi, objects);
        auto c = camera.measure(ego_x, ego_y, ego_psi, objects);
        l.insert(l.end(), r.begin(), r.end());
        l.insert(l.end(), c.begin(), c.end());
        return l;
    }
};

}  // namespace ad::sensors
