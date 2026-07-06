#include "rayTracing.h"
#include "lighting.h"
#include "geometry.h"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <fstream>
#include "octree.h"

inline double infinity() {
    return std::numeric_limits<double>::infinity();
}

namespace {
    ColorData cor_global;
    std::vector<LightData> LightList;
    std::vector<ObjectData> objetos_validos;
    
    // 🔍 COMPONENTES DE ACELERAÇÃO ISOLADA
    OctreeNode* rootNode = nullptr; 
    std::vector<const ObjectData*> planos_infinitos; // Guardará apenas os planos

    Ponto CameraPos;

    double tonemap(double x){
        return x / (x + 1.0);
    }

    bool notTIR(double n_i, double n_t, const Vetor& N, const Vetor& d_neg) {
        double cos_i = N.dot(d_neg);
        double ratio = n_i / n_t;
        double discriminant = 1.0 - ratio * ratio * (1.0 - cos_i * cos_i);
        return discriminant >= 0.0;
    }

    Vetor refractDirection(double n_i, double n_t, const Vetor& N, const Vetor& d_neg) {
        double ratio = n_i / n_t;
        double cos_i = N.dot(d_neg);
        double disc = 1.0 - ratio * ratio * (1.0 - cos_i * cos_i);
        double cos_t = std::sqrt(disc);

        Vetor d = d_neg * (-1.0);
        return d * ratio + N * (ratio * cos_i - cos_t);
    }

    HitResult busca_linear_antiga(const Ponto& origem, const Vetor& direcao) {
        HitResult closest_hr = { infinity(), Vetor(), nullptr };
        for (const auto& obj : objetos_validos) {
            HitResult hr = intersect_object(obj, origem, direcao);
            if (hr.hit() && hr.t < closest_hr.t) {
                closest_hr = hr;
            }
        }
        return closest_hr;
    }

    // 🔍 NOVA FUNÇÃO DE INTERSEÇÃO HÍBRIDA (Octree + Planos Lineares)
    HitResult intersectar_cena_hibrida(const Ponto& origem, const Vetor& direcao) {
        HitResult closest_hr = { infinity(), Vetor(), nullptr };

        // 1. Se houver Octree, busca nela os objetos finitos
        if (rootNode != nullptr) {
            closest_hr = rootNode->search(origem, direcao);
        }

        // 2. Testa contra os planos da cena de forma linear (sempre seguro)
        for (const auto& plano_ptr : planos_infinitos) {
            HitResult hr = intersect_object(*plano_ptr, origem, direcao);
            if (hr.hit() && hr.t < closest_hr.t && hr.t > 1e-4) {
                closest_hr = hr;
            }
        }

        return closest_hr;
    }

    std::array<double, 3> RayTracer(const Vetor& direcao, const Ponto& origem, int iteracao) {
        std::array<double, 3> arr = {0.0, 0.0, 0.0};
        if (iteracao >= limite_recursao) return arr;

        HitResult hr;
        if (rootNode != nullptr || !planos_infinitos.empty()) {
            hr = intersectar_cena_hibrida(origem, direcao);
        } else {
            hr = busca_linear_antiga(origem, direcao);
        }

        if (hr.t == infinity() || hr.hit_obj == nullptr) return arr;

        const ObjectData* hit_obj = hr.hit_obj;
        Vetor hit_normal = hr.normal;

        Ponto P = origem + direcao * hr.t;
        bool entrando = direcao.dot(hit_normal) < 0.0;
        Vetor normal = entrando ? hit_normal : hit_normal * -1.0;

        auto [c_r, c_g, c_b] = calcular_cor_phong(
                P, 
                normal, 
                direcao, 
                *hit_obj, 
                cor_global, 
                CameraPos, 
                LightList, 
                objetos_validos, 
                [](const ObjectData& obj_sombra, const Ponto& orig, const Vetor& dir) -> HitResult {
                    // Usar o acelerador híbrido também para raios de sombra
                    if (rootNode != nullptr || !planos_infinitos.empty()) {
                        return intersectar_cena_hibrida(orig, dir);
                    } else {
                        return intersect_object(obj_sombra, orig, dir);
                    }
                }
            );

        arr = {c_r, c_g, c_b};
        MaterialData mat = hit_obj->material;

        // REFLEXÃO
        if (mat.kr.r > 0.0 || mat.kr.g > 0.0 || mat.kr.b > 0.0) {
            Vetor refletido = (direcao - normal * (2.0 * direcao.dot(normal))).normalize();
            Ponto P_refletido = P + normal * 1e-4;
            auto cor_ref = RayTracer(refletido, P_refletido, iteracao + 1);

            arr[0] += mat.kr.r * cor_ref[0];
            arr[1] += mat.kr.g * cor_ref[1];
            arr[2] += mat.kr.b * cor_ref[2];
        }

        // REFRAÇÃO
        if (mat.kt.r > 0.0 || mat.kt.g > 0.0 || mat.kt.b > 0.0) {
            double n_i = entrando ? 1.0 : mat.ni;
            double n_t = entrando ? mat.ni : 1.0;

            if (notTIR(n_i, n_t, normal, -direcao)) {
                Vetor refratado = refractDirection(n_i, n_t, normal, -direcao).normalize();
                Ponto P_refratado = P - normal * 1e-4;
                auto cor_refr = RayTracer(refratado, P_refratado, iteracao + 1);

                arr[0] += mat.kt.r * cor_refr[0];
                arr[1] += mat.kt.g * cor_refr[1];
                arr[2] += mat.kt.b * cor_refr[2];
            } else {
                Vetor refletido_tir = (direcao - normal * (2.0 * direcao.dot(normal))).normalize();
                Ponto P_tir = P + normal * 1e-4;
                auto cor_tir = RayTracer(refletido_tir, P_tir, iteracao + 1);

                arr[0] += mat.kt.r * cor_tir[0];
                arr[1] += mat.kt.g * cor_tir[1];
                arr[2] += mat.kt.b * cor_tir[2];
            }
        }

        return arr;
    }
}

void Trace(const CenaProcessada& dados, const Camera& cam, const SceneData& scene){
    std::vector<std::array<int, 3>> image_buffer(cam.hres * cam.vres);
    objetos_validos = dados.valid_objects;
    LightList = scene.lightList;
    cor_global = scene.globalLight.color;
    CameraPos = scene.camera.lookfrom;

    // ==========================================
    // RENDERIZAÇÃO PRINCIPAL COM OCTREE
    // ==========================================
    std::cerr << "🚀 Construindo Octree e iniciando renderizacao..." << std::endl;
    auto ini_octree = std::chrono::high_resolution_clock::now();

    Ponto gMin(1e20, 1e20, 1e20), gMax(-1e20, -1e20, -1e20);
    std::vector<const ObjectData*> ptrs;
    bool possui_objetos_finitos = false;

    // Separação limpa de objetos na montagem da árvore
    for(const auto& obj : objetos_validos) {
        if (obj.objType == "plane") {
            planos_infinitos.push_back(&obj);
            continue; 
        }
        
        // Só entra na Octree o que for finito!
        ptrs.push_back(&obj);
        possui_objetos_finitos = true;
        auto [oMin, oMax] = getBoundingBox(obj);
        gMin = Ponto(std::min(gMin.getX(), oMin.getX()), std::min(gMin.getY(), oMin.getY()), std::min(gMin.getZ(), oMin.getZ()));
        gMax = Ponto(std::max(gMax.getX(), oMax.getX()), std::max(gMax.getY(), oMax.getY()), std::max(gMax.getZ(), oMax.getZ()));
    }

    if (!possui_objetos_finitos) {
        gMin = Ponto(-1000.0, -1000.0, -1000.0);
        gMax = Ponto(1000.0, 1000.0, 1000.0);
    }

    // Cria a árvore apenas com os objetos que possuem tamanho fixo!
    if (possui_objetos_finitos) {
        rootNode = new OctreeNode(ptrs, gMin, gMax, 0);
    }

    for(int j = 0; j < cam.vres; j++){
        for(int i = 0; i < cam.hres; i++){
            Vetor ray = cam.getRayDirection((double)i, (double)j);
            auto c = RayTracer(ray, CameraPos, 1);
            image_buffer[j * cam.hres + i] = {
                std::min(255, (int)(255.999 * tonemap(c[0]))),
                std::min(255, (int)(255.999 * tonemap(c[1]))),
                std::min(255, (int)(255.999 * tonemap(c[2])))
            };
        }
    }
    auto fim_octree = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tempo_octree = fim_octree - ini_octree;

    if (rootNode != nullptr) {
        delete rootNode;
        rootNode = nullptr;
    }
    planos_infinitos.clear();

    std::cerr << "✅ Renderizacao concluida em: " << tempo_octree.count() << " segundos." << std::endl;

    for (const auto& pixel : image_buffer) {
        std::cout << pixel[0] << ' ' << pixel[1] << ' ' << pixel[2] << '\n';
    }
}