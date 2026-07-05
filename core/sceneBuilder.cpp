#include "sceneBuilder.h"
#include "../utils/MeshReader/ObjReader.h"
#include "transformacoes.h"
#include <iostream>
#include <filesystem>
#include <cmath>
#include <algorithm>

namespace {
    void mesh_points(objReader& mesh_reader, std::vector<FaceData>& faces_data, ObjectData& objeto, bool transformar, const Matriz4x4& M_Transform){
        vector<Ponto> v0_list, v1_list, v2_list;
        vector<Vetor> n0_list, n1_list, n2_list;
        auto vertices_raw = mesh_reader.getVertices();
        auto normals_raw  = mesh_reader.getNormals();
        bool tem_normais  = !normals_raw.empty();
        for(const auto& face: faces_data){
            Ponto v0_raw = vertices_raw[face.verticeIndice[0]];
            Ponto v1_raw = vertices_raw[face.verticeIndice[1]];
            Ponto v2_raw = vertices_raw[face.verticeIndice[2]];
            Vetor n0_raw, n1_raw, n2_raw;
            if(tem_normais){
                n0_raw = normals_raw[face.normalIndice[0]];
                n1_raw = normals_raw[face.normalIndice[1]];
                n2_raw = normals_raw[face.normalIndice[2]];
            } else {
                Vetor aresta1 = v1_raw - v0_raw;
                Vetor aresta2 = v2_raw - v0_raw;
                Vetor n_face  = aresta1.cross(aresta2).normalize();
                n0_raw = n1_raw = n2_raw = n_face;
            }
            Ponto v0t, v1t, v2t;
            Vetor n0t, n1t, n2t;
            if(transformar && !objeto.transforms.empty()){
                v0t = aplicar_matriz_ponto(M_Transform, v0_raw);
                v1t = aplicar_matriz_ponto(M_Transform, v1_raw);
                v2t = aplicar_matriz_ponto(M_Transform, v2_raw);
                n0t = aplicar_matriz_normal_inv_t(M_Transform, n0_raw);
                n1t = aplicar_matriz_normal_inv_t(M_Transform, n1_raw);
                n2t = aplicar_matriz_normal_inv_t(M_Transform, n2_raw);
            }
            else{
                v0t = v0_raw; n0t = n0_raw;
                v1t = v1_raw; n1t = n1_raw;
                v2t = v2_raw; n2t = n2_raw;
            }
            v0_list.push_back(v0t); n0_list.push_back(n0t);
            v1_list.push_back(v1t); n1_list.push_back(n1t);
            v2_list.push_back(v2t); n2_list.push_back(n2t);
        }
        objeto.mesh_v0 = move(v0_list);
        objeto.mesh_v1 = move(v1_list);
        objeto.mesh_v2 = move(v2_list);
        objeto.mesh_n0 = move(n0_list);
        objeto.mesh_n1 = move(n1_list);
        objeto.mesh_n2 = move(n2_list);
        double min_val = std::numeric_limits<double>::infinity();
        double max_val = -std::numeric_limits<double>::infinity();
        double x1 = min_val, y1 = min_val, z1 = min_val;
        double x2 = max_val, y2 = max_val, z2 = max_val;
        for (size_t i = 0; i < objeto.mesh_v0.size(); ++i) {
            std::array<Ponto, 3> vertices = {objeto.mesh_v0[i], objeto.mesh_v1[i], objeto.mesh_v2[i]};
            for (const auto& v : vertices) {
                x1 = std::min(x1, v.getX());
                y1 = std::min(y1, v.getY());
                z1 = std::min(z1, v.getZ());
                x2 = std::max(x2, v.getX());
                y2 = std::max(y2, v.getY());
                z2 = std::max(z2, v.getZ());
            }
        }
        Ponto minimo(x1,y1,z1);
        Ponto maximo(x2,y2,z2);
        double folga = 1e-4;
        objeto.aabb_min = minimo - Vetor(folga, folga, folga);
        objeto.aabb_max = maximo + Vetor(folga, folga, folga);
        objeto.has_aabb = true;
    }

    void material(ObjectData& objeto, std::vector<FaceData>& faces_data){
        if(objeto.material.name.empty() && !faces_data.empty()){
            const auto& mat_mtl = faces_data[0].material;
            if(mat_mtl.kd.getX() != 0.0 || mat_mtl.kd.getY() != 0.0 || mat_mtl.kd.getZ() != 0.0){
                objeto.material.color.r = mat_mtl.kd.getX();
                objeto.material.color.g = mat_mtl.kd.getY();
                objeto.material.color.b = mat_mtl.kd.getZ();

                objeto.material.ka.r = mat_mtl.ka.getX();
                objeto.material.ka.g = mat_mtl.ka.getY();
                objeto.material.ka.b = mat_mtl.ka.getZ();

                objeto.material.ks.r = mat_mtl.ks.getX();
                objeto.material.ks.g = mat_mtl.ks.getY();
                objeto.material.ks.b = mat_mtl.ks.getZ();

                objeto.material.ns = mat_mtl.ns;
            }
        }
    }

    // Calcula dinamicamente as escalas do raio e da altura para qualquer orientação de eixo inicial
    std::pair<double, double> calcular_escala_cilindro_cone(const Matriz4x4& M_Transform, const Vetor& eixo_base) {
        // Extrai a escala pura embutida nas colunas da matriz
        double sx = std::sqrt(M_Transform[0][0]*M_Transform[0][0] + M_Transform[1][0]*M_Transform[1][0] + M_Transform[2][0]*M_Transform[2][0]);
        double sy = std::sqrt(M_Transform[0][1]*M_Transform[0][1] + M_Transform[1][1]*M_Transform[1][1] + M_Transform[2][1]*M_Transform[2][1]);
        double sz = std::sqrt(M_Transform[0][2]*M_Transform[0][2] + M_Transform[1][2]*M_Transform[1][2] + M_Transform[2][2]*M_Transform[2][2]);

        // Projeta os pesos do eixo inicial nas componentes de escala linear
        double s_altura = std::sqrt(
            (eixo_base.getX() * eixo_base.getX() * sx * sx) +
            (eixo_base.getY() * eixo_base.getY() * sy * sy) +
            (eixo_base.getZ() * eixo_base.getZ() * sz * sz)
        );

        // Deduz a escala média perpendicular (raio) usando invariância de volume (determinante)
        double volume = sx * sy * sz;
        double s_raio = std::sqrt(volume / (s_altura > 1e-6 ? s_altura : 1.0));

        return {s_raio, s_altura};
    }
}

CenaProcessada prepararObjetos(const std::vector<ObjectData>& raw_objects, bool transformar){
    CenaProcessada Resultado;
    for(const auto& object: raw_objects){
        ObjectData objeto = object;
        Matriz4x4 M_Transform;
        if(!object.transforms.empty() && transformar) M_Transform = build_transform_matriz(object.transforms, object.relativePos, object.objType);
        
        if(object.objType == "mesh"){
            string obj_path = objeto.getProperty("path");
            if(!filesystem::exists(obj_path)){
                cerr<<"Aviso! OBJ nao encontrado: "<<obj_path<<" ignorando Objeto\n";
                continue;
            }
            Resultado.loaded_meshes[obj_path] = make_unique<objReader>(obj_path);
            objReader& mesh_reader = *Resultado.loaded_meshes[obj_path];
            auto faces = mesh_reader.getFacePoints();
            if(faces.empty()){
                cerr<<"Aviso! OBJ vazio invalido: "<<obj_path<<" ignorando Objeto\n";
                continue;
            }
            auto faces_data = mesh_reader.getFaces();
            material(objeto, faces_data);
            mesh_points(mesh_reader, faces_data, objeto, transformar, M_Transform);
        }
        else if(object.objType == "plane"){
            if(transformar && !object.transforms.empty()){
                objeto.relativePos = aplicar_matriz_ponto(M_Transform, objeto.relativePos);
                Vetor normal = objeto.vetorPointData.at("normal");
                Matriz4x4 M_Rot = extrair_apenas_rotacao(M_Transform);
                normal = aplicar_matriz_normal(M_Rot, normal);
                objeto.vetorPointData.at("normal") = normal;
            }
        }
        else if(object.objType == "sphere"){
            if(transformar && !object.transforms.empty()){
                objeto.relativePos = aplicar_matriz_ponto(M_Transform, object.relativePos);
                double s = crescimento_raio(M_Transform);
                objeto.numericData.at("radius") *= s;
            }
        }
        else if(object.objType == "cylinder"){
            Vetor eixo_base = objeto.vetorPointData.count("eixo") ? objeto.vetorPointData.at("eixo") : Vetor(0.0, 1.0, 0.0);
            eixo_base = eixo_base.normalize();

            if(transformar && !object.transforms.empty()){
                objeto.relativePos = aplicar_matriz_ponto(M_Transform, object.relativePos);
                
                Matriz4x4 M_Rot = extrair_apenas_rotacao(M_Transform);
                objeto.vetorPointData["eixo"] = aplicar_matriz_vetor(M_Rot, eixo_base).normalize();
                
                // Extração dinâmica e robusta de escala para qualquer eixo arbitrário
                auto [s_raio, s_altura] = calcular_escala_cilindro_cone(M_Transform, eixo_base);
                objeto.numericData.at("radius") *= s_raio;
                objeto.numericData.at("height") *= s_altura;
            } else {
                objeto.vetorPointData["eixo"] = eixo_base;
            }
        }
        else if(object.objType == "cone"){
            Vetor eixo_base = objeto.vetorPointData.count("eixo") ? objeto.vetorPointData.at("eixo") : Vetor(0.0, 1.0, 0.0);
            eixo_base = eixo_base.normalize();

            if(transformar && !object.transforms.empty()){
                objeto.relativePos = aplicar_matriz_ponto(M_Transform, object.relativePos);
                
                Matriz4x4 M_Rot = extrair_apenas_rotacao(M_Transform);
                objeto.vetorPointData["eixo"] = aplicar_matriz_vetor(M_Rot, eixo_base).normalize();
                
                // Extração dinâmica e robusta de escala para qualquer eixo arbitrário
                auto [s_raio, s_altura] = calcular_escala_cilindro_cone(M_Transform, eixo_base);
                objeto.numericData.at("radius") *= s_raio;
                objeto.numericData.at("height") *= s_altura;
            } else {
                objeto.vetorPointData["eixo"] = eixo_base;
            }
        }
        Resultado.valid_objects.push_back(move(objeto));
    }
    return Resultado;
}