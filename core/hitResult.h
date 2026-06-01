#pragma once
#include <limits>
#include "../src/Vetor.h"

struct HitResult {
    double t      = std::numeric_limits<double>::infinity();
    Vetor  normal = Vetor();
    bool   hit()  const { return t < std::numeric_limits<double>::infinity(); }
};