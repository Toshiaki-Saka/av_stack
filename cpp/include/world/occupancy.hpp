#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace ad::world {

// Predicted position at a single horizon step.
struct PredPos { double x, y; };

// 2-D probabilistic occupancy grid built from multi-step Gaussian splatting.
// Each horizon h in [horizons[0]…horizons.back()] produces one grid layer.
//
//   sigma(h)  = sigma0 + growth * h
//   g(cell) = 1 - prod_i( 1 - Gauss_i(cell) )   (probabilistic OR)
class OccupancyGrid {
public:
    double x_min;
    double y_min, y_max;
    double res;      // grid cell size [m]
    double sigma0;   // initial positional uncertainty [m]
    double growth;   // uncertainty growth per horizon step [m/step]
    double car_l;    // splat half-length along heading [m]
    double car_w;    // splat half-width [m]
    double x_range;  // x extent [m]

    int nx, ny;

    explicit OccupancyGrid(double x_min_   = 0.0,
                           double x_range_ = 80.0,
                           double y_min_   = -2.0,
                           double y_max_   = 5.5,
                           double res_     = 0.5,
                           double sigma0_  = 0.6,
                           double growth_  = 0.06,
                           double car_l_   = 2.2,
                           double car_w_   = 0.9)
        : x_min(x_min_), y_min(y_min_), y_max(y_max_),
          res(res_), sigma0(sigma0_), growth(growth_),
          car_l(car_l_), car_w(car_w_), x_range(x_range_)
    {
        nx = static_cast<int>(x_range_ / res_) + 1;
        ny = static_cast<int>((y_max_ - y_min_) / res_) + 1;
    }

    // Build one occupancy layer per horizon value.
    // tracks_pred: outer index = object, inner index = horizon, value = predicted (x,y).
    // horizons: which horizon indices to build grids for (e.g. {5,10,20}).
    // Returns vector of grids in the same order as horizons.
    std::vector<std::vector<float>>
    predict(const std::vector<std::vector<PredPos>>& tracks_pred,
            const std::vector<int>& horizons) const
    {
        std::vector<std::vector<float>> grids;
        grids.reserve(horizons.size());

        for (int h : horizons) {
            std::vector<float> g(nx * ny, 0.0f);
            double sigma = sigma0 + growth * h;
            double sl    = std::max(sigma, car_l);   // splat half-length
            double sw    = std::max(sigma, car_w);   // splat half-width

            for (const auto& track : tracks_pred) {
                if (h >= static_cast<int>(track.size())) continue;
                double px = track[h].x;
                double py = track[h].y;

                // Bounding box in cell space
                int ix_lo = std::max(0, static_cast<int>((px - 3*sl - x_min) / res));
                int ix_hi = std::min(nx-1, static_cast<int>((px + 3*sl - x_min) / res) + 1);
                int iy_lo = std::max(0, static_cast<int>((py - 3*sw - y_min) / res));
                int iy_hi = std::min(ny-1, static_cast<int>((py + 3*sw - y_min) / res) + 1);

                for (int ix = ix_lo; ix <= ix_hi; ++ix) {
                    double cx = x_min + ix * res;
                    double ex = (cx - px) / sl;
                    for (int iy = iy_lo; iy <= iy_hi; ++iy) {
                        double cy = y_min + iy * res;
                        double ey = (cy - py) / sw;
                        double p  = std::exp(-0.5 * (ex*ex + ey*ey));
                        auto& cell = g[ix * ny + iy];
                        cell = static_cast<float>(1.0 - (1.0 - cell) * (1.0 - p));
                    }
                }
            }
            grids.push_back(std::move(g));
        }
        return grids;
    }

    // Sample the grid at world position (wx, wy); nearest-cell lookup.
    float sample(const std::vector<float>& grid, double wx, double wy) const {
        int ix = std::max(0, std::min(nx-1, static_cast<int>((wx - x_min) / res)));
        int iy = std::max(0, std::min(ny-1, static_cast<int>((wy - y_min) / res)));
        return grid[ix * ny + iy];
    }

    // Maximum occupancy value along a sequence of world-frame (x, y) waypoints.
    float path_risk(const std::vector<float>& grid,
                    const std::vector<double>& xs,
                    const std::vector<double>& ys) const
    {
        float risk = 0.0f;
        for (int i = 0; i < static_cast<int>(xs.size()); ++i)
            risk = std::max(risk, sample(grid, xs[i], ys[i]));
        return risk;
    }
};

}  // namespace ad::world
