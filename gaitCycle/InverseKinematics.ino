
#include <cmath>

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

void setup(){
    Serial.begin(115200);
    LegAngles angles;
    legIK(sqrt(50), -sqrt(50), 5, 5, &angles);
    Serial.print(angles.hipDeg);
    Serial.print("   ");
    Serial.println(angles.kneeDeg);
}

void loop(){
  
}
