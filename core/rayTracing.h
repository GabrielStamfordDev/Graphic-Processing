#pragma once
#include "Camera.h"

namespace{
    int limite_recursao = 15;
}

void Trace(const CenaProcessada& dados, const Camera& cam, const SceneData& scene);