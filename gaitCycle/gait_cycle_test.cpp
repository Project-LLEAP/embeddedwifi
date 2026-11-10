
#include <cmath>
#include <iostream>
#include "gait_xy.hpp"
#include <fstream>

struct LegAngles {
    float hipDeg;
    float kneeDeg;
};

// Hip is at (0, 0); +x is forward/out, -y is down.
// Hip: 0 degrees down, +90 degrees along +x.
// Knee: 0 degrees straight; positive flexion makes shin angle hip - knee.
// Uses the positive-knee (backward-bending) solution.
// Returns false for invalid dimensions or an unreachable target.
// On failure, angles is left unchanged.
bool legIK(float x, float y, float thighLength, float shinLength, LegAngles *angles)
{
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(thighLength) || !std::isfinite(shinLength) ||
        thighLength <= 0.0f || shinLength <= 0.0f) {
        return false;
    }

    const float distance = std::hypot(x, y);
    const float minReach = std::fabs(thighLength - shinLength);
    const float maxReach = thighLength + shinLength;

    // At the hip itself, the hip angle is undefined.
    if (distance == 0.0f ||
        distance < minReach || distance > maxReach) {
        return false;
    }

    float cosKnee =
        (distance * distance -
         thighLength * thighLength -
         shinLength * shinLength) /
        (2.0f * thighLength * shinLength);

    // Protect acos against floating-point rounding.
    if (cosKnee > 1.0f) cosKnee = 1.0f;
    if (cosKnee < -1.0f) cosKnee = -1.0f;

    const float knee = std::acos(cosKnee);
    const float hip =
        std::atan2(x, -y) +
        std::atan2(shinLength * std::sin(knee),
                   thighLength + shinLength * std::cos(knee));

    constexpr float RAD_TO_DEGREES = 57.2957795f;
    angles->hipDeg = hip * RAD_TO_DEGREES;
    angles->kneeDeg = knee * RAD_TO_DEGREES;
    return true;
}

int main(){
    //Serial.begin(115200);
    LegAngles angles;
    float finalAngles[101][2]{};  // 101 samples, including t = 0 and t = 1

    for (int index = 0; index <= 100; ++index) {
        double t = index / 100.0;
        GaitXY footPos = gaitXY(t);

        // Match the lengths used in gait_xy.hpp.
        if (!legIK(footPos.x, footPos.y, 5, 5, &angles)) {
            std::cout << "ERROR: invalid position at t = " << t << '\n';
            continue;
        }

        finalAngles[index][0] = angles.hipDeg;
        finalAngles[index][1] = angles.kneeDeg;
    }
    std::ofstream file("finalAngles.json");
    if (!file) {
        std::cerr << "ERROR: could not open output file\n";
        return 1;
    }

    file << "[";
    for (int index = 0; index < 101; ++index) {
        if (index > 0) file << ", ";
        file << "[" << finalAngles[index][0]
            << ", " << finalAngles[index][1] << "]";
    }
    file << "]\n";

        return 0;
    }

void loop(){
  
}
