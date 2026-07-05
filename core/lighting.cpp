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
    const ColorData& cor_global,
    const Ponto& LookFrom_atual,
    const std::vector<LightData>& LightList,
    const std::vector<ObjectData>& valid_objects,
    const std::function<HitResult(const ObjectData&, const Ponto&, const Vetor&)>& intersect_func
) {
    auto Ia = cor_global;
    auto mat = hit_obj.material;

    Vetor N = N_in.normalize();
    Vetor V = (-ray_dir).normalize();

    double cor_r = mat.ka.r * Ia.r;
    double cor_g = mat.ka.g * Ia.g;
    double cor_b = mat.ka.b * Ia.b;

    for (const auto& luz : LightList) {
        Vetor vetor_luz = luz.pos - P;
        double distancia_luz = vetor_luz.magnitude();
        Vetor L = vetor_luz.normalize();

        // Atenuação simples: 1.0 / (1.0 + 0.1 * distancia)
        // Isso evita o estouro de luz sem adicionar novas dependências complexas
        double atenuacao = 1.0 / (1.0 + 0.1 * distancia_luz);

        double L_dot_N = N.dot(L);
        if (L_dot_N <= 0.0) continue;

        Ponto P_sombra = P + (N * EPSILON);
        if (checar_sombra(P_sombra, L, distancia_luz, valid_objects, intersect_func)) {
            continue;
        }

        // Difuso com atenuação
        cor_r += mat.color.r * L_dot_N * luz.color.r * atenuacao;
        cor_g += mat.color.g * L_dot_N * luz.color.g * atenuacao;
        cor_b += mat.color.b * L_dot_N * luz.color.b * atenuacao;

        // Especular com atenuação
        Vetor R = ((N * (2.0 * L_dot_N)) - L).normalize();
        double R_dot_V = R.dot(V);

        if (R_dot_V > 0.0) {
            double spec = std::pow(R_dot_V, mat.ns);
            cor_r += mat.ks.r * spec * luz.color.r * atenuacao;
            cor_g += mat.ks.g * spec * luz.color.g * atenuacao;
            cor_b += mat.ks.b * spec * luz.color.b * atenuacao;
        }
    }

    cor_r = std::clamp(cor_r, 0.0, 1.0);
    cor_g = std::clamp(cor_g, 0.0, 1.0);
    cor_b = std::clamp(cor_b, 0.0, 1.0);

    return {cor_r, cor_g, cor_b};
}