#ifndef GAIT_XY_HPP
#define GAIT_XY_HPP

#include <cmath>

struct GaitXY { double x, y; };

// Hip is at (0, 0); +x is forward/out, -y is down.
// Hip: 0 degrees down, +90 degrees along +x.
// Knee: 0 degrees straight; positive flexion makes shin angle hip - knee.
inline GaitXY gaitXY(double t) {
    const double PI = 3.14159265358979323846;
    const double L1 = 5.0, L2 = 5.0;
    const double w = 2 * PI * t;
    const double hip = 9.941
        + 16.992 * std::cos(w) - 3.192 * std::sin(w)
        - 2.850 * std::cos(2 * w) - 2.913 * std::sin(2 * w)
        - 1.331 * std::cos(3 * w) + 1.310 * std::sin(3 * w)
        - 0.270 * std::cos(4 * w) - 0.175 * std::sin(4 * w)
        - 0.477 * std::cos(5 * w) + 0.148 * std::sin(5 * w)
        - 0.024 * std::cos(6 * w) + 0.142 * std::sin(6 * w);
    const double knee = 25.267
        + 1.735 * std::cos(w) - 22.374 * std::sin(w)
        - 17.199 * std::cos(2 * w) - 1.113 * std::sin(2 * w)
        - 3.498 * std::cos(3 * w) + 3.991 * std::sin(3 * w)
        - 1.100 * std::cos(4 * w) + 0.052 * std::sin(4 * w)
        - 1.052 * std::cos(5 * w) + 0.254 * std::sin(5 * w)
        - 0.321 * std::cos(6 * w) + 0.377 * std::sin(6 * w);
    const double a = hip * PI / 180, b = (hip - knee) * PI / 180;
    return { L1 * std::sin(a) + L2 * std::sin(b), -L1 * std::cos(a) - L2 * std::cos(b) };
}

#endif
