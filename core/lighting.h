#pragma once
#include <array>
#include <vector>
#include <functional>
#include "../src/Ponto.h"
#include "../src/Vetor.h"
#include "hitResult.h"
#include "../utils/scene/sceneSchema.hpp" 

std::array<double, 3> calcular_cor_phong(
    const Ponto& P,
    const Vetor& N_in,
    const Vetor& ray_dir,
    const ObjectData& hit_obj,
    const SceneData& scene_data,
    const std::function<HitResult(const ObjectData&, const Ponto&, const Vetor&)>& intersect_func
);