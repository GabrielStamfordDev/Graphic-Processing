#include "lighting.h"
#include <cmath>
#include <algorithm>

const double EPSILON = 1e-4;
namespace {
    bool checar_sombra(
        const Ponto& ponto_deslocado,
        const Vetor& L_normalizado,
        double distancia_luz,
        const std::vector<ObjectData>& objetos,
        const std::function<HitResult(const ObjectData&, const Ponto&, const Vetor&)>& intersect_func
    ) {
        for (const auto& obj_sombra : objetos) {
            HitResult hr = intersect_func(obj_sombra, ponto_deslocado, L_normalizado);
            if (hr.hit() && hr.t > EPSILON && hr.t < distancia_luz) {
                return true;
            }
        }
        return false;
    }
}

std::array<double, 3> calcular_cor_phong(
    const Ponto& P,
    const Vetor& N_in,
    const Vetor& ray_dir,
    const ObjectData& hit_obj,
    const SceneData& scene_data,
    const std::function<HitResult(const ObjectData&, const Ponto&, const Vetor&)>& intersect_func
) {
    auto Ia = scene_data.globalLight.color;
    auto mat = hit_obj.material;
    Vetor N = N_in;
    N = N.normalize();

    Vetor V = (scene_data.camera.lookfrom - P).normalize();

    double cor_r = mat.ka.r * Ia.r;
    double cor_g = mat.ka.g * Ia.g;
    double cor_b = mat.ka.b * Ia.b;

    for (const auto& luz : scene_data.lightList) {
        Vetor vetor_luz = luz.pos - P;
        double distancia_luz = vetor_luz.magnitude();
        Vetor L = vetor_luz.normalize();

        Ponto P_sombra = P + (N * EPSILON);

        if (checar_sombra(P_sombra, L, distancia_luz, scene_data.objects, intersect_func)) {
            continue;
        }

        double L_dot_N = N.dot(L);
        if (L_dot_N <= 0.0) {
            continue;
        }

        cor_r += mat.color.r * L_dot_N * luz.color.r;
        cor_g += mat.color.g * L_dot_N * luz.color.g;
        cor_b += mat.color.b * L_dot_N * luz.color.b;

        Vetor R = ((N * (2.0 * L_dot_N)) - L).normalize();
        double R_dot_V = R.dot(V);

        if (R_dot_V > 0.0) {
            double spec = std::pow(R_dot_V, mat.ns);
            cor_r += mat.ks.r * spec * luz.color.r;
            cor_g += mat.ks.g * spec * luz.color.g;
            cor_b += mat.ks.b * spec * luz.color.b;
        }
    }

    cor_r = std::min(1.0, std::max(0.0, cor_r));
    cor_g = std::min(1.0, std::max(0.0, cor_g));
    cor_b = std::min(1.0, std::max(0.0, cor_b));

    return {cor_r, cor_g, cor_b};
}