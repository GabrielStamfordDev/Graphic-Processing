#include "rayTracing.h"
#include "lighting.h"
#include "geometry.h"

void RayTracer(const CenaProcessada& dados, const Camera& cam, const SceneData& scene){
    cout<<"P3\n"<<cam.hres<<' '<<cam.vres<<"\n255\n";
    for(int j = 0; j < cam.vres; j ++){
        cerr<<"Linha "<<j<<'/'<<cam.vres<<'\r'<<flush;
        for(int i = 0; i < cam.hres; i++){
            Vetor ray_dir = cam.getRayDirection(i, j);
            double closest_t = infinity();
            const ObjectData* hit_obj = nullptr;
            Vetor hit_normal;
            for(const auto& objeto : dados.valid_objects){
                HitResult hr = intersect_object(objeto, cam.C, ray_dir);
                if(hr.t < closest_t){
                    closest_t  = hr.t;
                    hit_obj    = &objeto;
                    hit_normal = hr.normal;
                }
            }

            if(!hit_obj){
                cout << "0 0 0\n";
                continue;
            }

            Ponto P = cam.C + (ray_dir * closest_t);
            auto [cor_r, cor_g, cor_b] = calcular_cor_phong(
                P, hit_normal, ray_dir, *hit_obj, scene, intersect_object);

            
            int r = (int)(255.999 * cor_r);
            int g = (int)(255.999 * cor_g);
            int b = (int)(255.999 * cor_b);
            cout << r << ' ' << g << ' ' << b << '\n';
        }
    }
}