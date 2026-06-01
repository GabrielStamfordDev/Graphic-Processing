#include "sceneBuilder.h"
#include "../utils/MeshReader/ObjReader.h"
#include "transformacoes.h"
#include <iostream>
#include <filesystem>

namespace {
    void mesh_points(objReader& mesh_reader, std::vector<FaceData>& faces_data,ObjectData& objeto, bool transformar, const Matriz4x4& M_Transform){
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
        else if(object.objType == "plane" && transformar && !object.transforms.empty()){
            objeto.relativePos = aplicar_matriz_ponto(M_Transform, objeto.relativePos);
            Vetor normal = objeto.vetorPointData.at("normal");
            normal = aplicar_matriz_normal(M_Transform, normal);
            objeto.vetorPointData.at("normal") = normal;
        }
        else if(object.objType == "sphere" && transformar && !object.transforms.empty()){
            objeto.relativePos = aplicar_matriz_ponto(M_Transform, object.relativePos);
            double s = crescimento_raio(M_Transform);
            objeto.numericData.at("radius") *= s;
        }
        Resultado.valid_objects.push_back(move(objeto));
    }
    return Resultado;
}