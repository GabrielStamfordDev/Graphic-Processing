#include <unordered_map>
#include <array>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <filesystem>
#include <iostream>
#include "src/Ponto.h"
#include "src/Vetor.h"
#include "Camera.h"
#include "geometry.cpp"
#include "transformacoes.cpp"
#include "utils/scene/sceneParser.cpp"
#include "utils/MeshReader/ObjReader.cpp"
using namespace std;
using hit_color = array<double, 3>;

Matriz4x4 build_transform_matriz(vector<TransformData>& transforma){
    Matriz4x4 M_total = matriz_identidade();
    Matriz4x4 M_atual;
    Matriz4x4 Mx, My, Mz;
    for(const auto& t : transforma){
        M_atual = matriz_identidade();
        if(t.tType == "scaling"){
            M_atual = matriz_escala(t.data.getX(), t.data.getY(), t.data.getZ());
        }
        else if(t.tType == "translation"){
            M_atual = matriz_translacao(t.data.getX(),t.data.getY(),t.data.getZ());
        }
        else if(t.tType == "rotation"){
            Mx = matriz_rotacao_x(t.data.getX());
            My = matriz_rotacao_y(t.data.getY());
            Mz = matriz_rotacao_z(t.data.getZ());
            M_atual = Mz * My * Mx;
        }
        M_total = M_total * M_atual;
    }
    return M_total;
}

double intersect_object(const ObjectData& obj, const Ponto& ray_origin, const Vetor& ray_dir) {
    if(obj.objType == "sphere"){
        double raio = obj.numericData.at("radius");
        return intersect_sphere(ray_origin, ray_dir, obj.relativePos, raio);
    }

    if(obj.objType == "plane"){
        Vetor normal = obj.vetorPointData.at("normal");
        normal = normal.normalize();
        return intersect_plane(ray_origin, ray_dir, obj.relativePos, normal);
    }

    if(obj.objType == "mesh"){
        double closest_t = numeric_limits<double>::infinity();
        for(size_t i = 0; i < obj.mesh_v0.size(); ++i){
            double t = intersect_triangle(ray_origin, ray_dir, 
                                         obj.mesh_v0[i], obj.mesh_v1[i], obj.mesh_v2[i]);
            if(t < closest_t){
                closest_t = t;
            }
        }
        return closest_t;
    }
    return numeric_limits<double>::infinity();
}

int main(int argc, char** argv){
    ios::sync_with_stdio(false);cin.tie(0);
    if(argc < 2){
        cerr<<"Uso: "<<argv[0]<<" cena.josn\n";
        return 1;
    }
    SceneData scene = SceneJsonLoader::loadFile(argv[1]);
    cerr<<"Resolucao: "<<scene.camera.image_width<<" x "<<scene.camera.image_height<<'\n';
    Camera cam(scene.camera);

    bool transformar = true;
    if(argc > 2 && string(argv[2]) == "--no-transform") transformar = false;

    unordered_map<string, unique_ptr<objReader>> loaded_meshes;
    vector<ObjectData> valid_objects;

    for (const auto& src_objeto : scene.objects) {
        ObjectData objeto = src_objeto;
        Matriz4x4 M_total = matriz_identidade();

        if(transformar && !objeto.transforms.empty()){
            M_total = build_transform_matriz(objeto.transforms);
            if(objeto.objType == "sphere"){
                objeto.relativePos = aplicar_matriz_ponto(M_total, objeto.relativePos);
                double sx = sqrt(
                    M_total[0][0] * M_total[0][0] +
                    M_total[1][0] * M_total[1][0] +
                    M_total[2][0] * M_total[2][0]
                );
                double sy = sqrt(
                    M_total[0][1] * M_total[0][1] +
                    M_total[1][1] * M_total[1][1] +
                    M_total[2][1] * M_total[2][1]
                );
                double sz = sqrt(
                    M_total[0][2] * M_total[0][2] +
                    M_total[1][2] * M_total[1][2] +
                    M_total[2][2] * M_total[2][2]
                );
                double s = (sx + sy + sz) / 3.0; //max?
                objeto.numericData.at("radius") *= s;
            }
            else if(objeto.objType == "plane"){
                objeto.relativePos = aplicar_matriz_ponto(M_total, objeto.relativePos);
                Vetor normal = objeto.vetorPointData.at("normal");
                normal = aplicar_matriz_normal(M_total, normal);
                objeto.vetorPointData.at("normal") = normal;
            }
        }

        if (objeto.objType == "mesh"){
            string obj_path = objeto.getProperty("path");
            if(!filesystem::exists(obj_path)){
                cerr<<"Aviso! OBJ nao encontrado: "<<obj_path<<" ignorando Objeto\n";
                continue;
            }
            // Estou considerando que não existe duas referencias para carregar o mesmo objeto
            loaded_meshes[obj_path] = make_unique<objReader>(obj_path);
            objReader& mesh_reader = *loaded_meshes[obj_path];
            auto faces = mesh_reader.getFacePoints();
            if(faces.empty()){
                cerr<<"Aviso! OBJ vazio invalido: "<<obj_path<<" ignorando Objeto\n";
                continue;
            }
            vector<Ponto> v0_list, v1_list, v2_list;
            Ponto v0t, v1t, v2t;
            for(const auto& face_pts : faces){
                if(transformar){
                    v0t = aplicar_matriz_ponto(M_total, face_pts[0]);
                    v1t = aplicar_matriz_ponto(M_total, face_pts[1]);
                    v2t = aplicar_matriz_ponto(M_total, face_pts[2]);
                }
                else{
                    v0t = face_pts[0];
                    v1t = face_pts[1];
                    v2t = face_pts[2];
                }
                v0_list.push_back(v0t);
                v1_list.push_back(v1t);
                v2_list.push_back(v2t);
            }
            objeto.mesh_v0 = move(v0_list);
            objeto.mesh_v1 = move(v1_list);
            objeto.mesh_v2 = move(v2_list);
        }
        valid_objects.push_back(move(objeto));
    }

    scene.objects = valid_objects;

    bool cor_valida; hit_color cor;
    double closest_t; double t;
    Vetor ray_dir;
    int r, g, b;
    cout<<"P3\n"<<cam.hres<<' '<<cam.vres<<"\n255\n";
    for(int j = 0; j < cam.vres; j ++){
        cerr<<"Linha "<<j<<'/'<<cam.vres<<'\r'<<flush;
        for(int i = 0; i < cam.hres; i++){
            ray_dir = cam.getRayDirection(i, j);
            closest_t = numeric_limits<double>::infinity();
            cor_valida = false;
            for(const auto& objeto : scene.objects){
                t = intersect_object(objeto, cam.C, ray_dir);
                if(t < closest_t){
                    closest_t = t;
                    cor[0] = objeto.material.color.r;
                    cor[1] = objeto.material.color.g;
                    cor[2] = objeto.material.color.b;
                    cor_valida = true;
                }
            }

            if(cor_valida){
                r = (int)(255.999 * cor[0]);
                g = (int)(255.999 * cor[1]);
                b = (int)(255.999 * cor[2]);
            }
            else{
                r = 0;
                g = 0;
                b = 0;
            }
            cout<<r<<' '<<g<<' '<<b<<'\n';
        }
    }
    cerr<<"\n Renderizacao concluida!";
}