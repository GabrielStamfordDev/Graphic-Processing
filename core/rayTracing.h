#pragma once
#include "Camera.h"

namespace{
    int limite_recursao = 10;
}

void Trace(const CenaProcessada& dados, const Camera& cam, const SceneData& scene);