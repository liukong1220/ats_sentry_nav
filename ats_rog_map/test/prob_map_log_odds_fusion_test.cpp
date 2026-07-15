#include <rog_map/prob_map.h>

#include <cmath>
#include <iostream>

namespace {

bool expectNear(const float actual, const float expected, const char *label) {
    if (std::fabs(actual - expected) < 1.0e-5F) {
        return true;
    }
    std::cerr << label << ": expected " << expected << ", got " << actual << std::endl;
    return false;
}

}  // namespace

int main() {
    bool ok = true;
    ok = expectNear(
        rog_map::ProbMap::applyRaycastLogOddsUpdate(0.0F, 1.0F, -0.5F, -2.0F, 2.0F, 1, 4),
        -1.0F,
        "mixed hit and miss should use net evidence") && ok;
    ok = expectNear(
        rog_map::ProbMap::applyRaycastLogOddsUpdate(1.5F, 1.0F, -0.5F, -2.0F, 2.0F, 2, 0),
        2.0F,
        "hit update should clamp to max") && ok;
    ok = expectNear(
        rog_map::ProbMap::applyRaycastLogOddsUpdate(-1.5F, 1.0F, -0.5F, -2.0F, 2.0F, 0, 2),
        -2.0F,
        "miss update should clamp to min") && ok;
    return ok ? 0 : 1;
}
