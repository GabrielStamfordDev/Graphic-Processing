#include "rayTracing.h"

#include "lighting.h"

#include "geometry.h"

#include <cmath>

#include <algorithm>



namespace {

    ColorData cor_global;

    std::vector<LightData> LightList;

    std::vector<ObjectData> objetos_validos;

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



    std::array<double, 3> RayTracer(const Vetor& direcao, const Ponto& origem, int iteracao) {

        std::array<double, 3> arr = {0.0, 0.0, 0.0};



        if (iteracao >= limite_recursao) return arr;



        double closest_t = infinity();

        const ObjectData* hit_obj = nullptr;

        Vetor hit_normal;



        for (const auto& objeto : objetos_validos) {

            HitResult hr = intersect_object(objeto, origem, direcao);

            if (hr.t < closest_t) {

                closest_t = hr.t;

                hit_obj = &objeto;

                hit_normal = hr.normal;

            }

        }



        if (!hit_obj) return arr;



        Ponto P = origem + direcao * closest_t;



        bool entrando = direcao.dot(hit_normal) < 0.0;

        Vetor normal = entrando ? hit_normal : hit_normal * -1.0;



        auto [c_r, c_g, c_b] = calcular_cor_phong(

            P, normal, direcao,

            *hit_obj,

            cor_global,

            CameraPos,

            LightList,

            objetos_validos,

            intersect_object

        );



        arr = {c_r, c_g, c_b};



        MaterialData mat = hit_obj->material;



        // REFLEXÃO

        if (mat.kr.r > 0.0 || mat.kr.g > 0.0 || mat.kr.b > 0.0) {

            Vetor refletido =

                (direcao - normal * (2.0 * direcao.dot(normal))).normalize();



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

                Vetor refratado =

                    refractDirection(n_i, n_t, normal, -direcao).normalize();



                Ponto P_refratado = P - normal * 1e-4;



                auto cor_refr = RayTracer(refratado, P_refratado, iteracao + 1);



                arr[0] += mat.kt.r * cor_refr[0];

                arr[1] += mat.kt.g * cor_refr[1];

                arr[2] += mat.kt.b * cor_refr[2];

            } else {

                Vetor refletido_tir =

                    (direcao - normal * (2.0 * direcao.dot(normal))).normalize();



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



    for(int j = 0; j < cam.vres; j++){

        for(int i = 0; i < cam.hres; i++){



            Vetor ray = cam.getRayDirection((double)i, (double)j);

            auto c = RayTracer(ray, CameraPos, 1);



            image_buffer[j * cam.hres + i] = {

                std::min(255, (int)(255.999 * c[0])),

                std::min(255, (int)(255.999 * c[1])),

                std::min(255, (int)(255.999 * c[2]))

            };

        }

    }



    for (const auto& pixel : image_buffer) {

        std::cout << pixel[0] << ' ' << pixel[1] << ' ' << pixel[2] << '\n';

    }

} 

