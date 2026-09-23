// Adapted from unitree_rl_mjlab/deploy/include/LinearInterpolator.h.
// Same piecewise-linear keyframe interpolation, retyped onto double and with
// the assert replaced by a defensive empty-input guard.
#pragma once

#include <cassert>
#include <vector>

namespace mini_pi
{

inline std::vector<double> linear_interpolate(double t,
                                              const std::vector<double>& ts,
                                              const std::vector<std::vector<double>>& ys)
{
    if (ys.empty() || ts.size() != ys.size()) return {};
    if (ts.size() == 1) return ys[0];

    if (t <= ts.front()) return ys.front();
    if (t >= ts.back())  return ys.back();

    for (std::size_t i = 0; i + 1 < ts.size(); ++i)
    {
        if (t >= ts[i] && t <= ts[i + 1])
        {
            const double span = ts[i + 1] - ts[i];
            const double a = span > 0.0 ? (t - ts[i]) / span : 1.0;
            std::vector<double> out(ys[i].size());
            for (std::size_t j = 0; j < ys[i].size(); ++j)
                out[j] = ys[i][j] * (1.0 - a) + ys[i + 1][j] * a;
            return out;
        }
    }
    return ys.back();
}

} // namespace mini_pi
