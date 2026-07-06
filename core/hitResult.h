#pragma once
#include <limits>
#include "../src/Vetor.h"

// Forward declaration para evitar erro de inclusão circular (se necessário)
struct ObjectData; 

struct HitResult {
    double t        = std::numeric_limits<double>::infinity();
    Vetor  normal   = Vetor();
    const ObjectData* hit_obj = nullptr; // Adicionado: o objeto atingido

    bool   hit()    const { return t < std::numeric_limits<double>::infinity(); }
};