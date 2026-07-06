#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include "../utils/scene/sceneSchema.hpp"

// Forward Declarations para quebrar dependências circulares
struct HitResult;
class Ponto;
class Vetor;
struct ObjectData;

// Função global segura para infinito
inline double get_inf() { return std::numeric_limits<double>::infinity(); }


std::tuple<double, double, double> intersect_triangle_uvt(const Ponto& origem, const Vetor& direcao, const Ponto& v0, const Ponto& v1, const Ponto& v2);
std::pair<Ponto, Ponto> getBoundingBox(const ObjectData& obj);

// =========================================================================
// 1. MESH OCTREE - Acelerador de triângulos para cada mesh
// =========================================================================
class MeshOctreeNode {
public:
    Ponto c_min, c_max;
    std::vector<size_t> indices_triangulos;  // Índices dos triângulos neste nó
    MeshOctreeNode* filhos[8];
    bool eh_folha;

    MeshOctreeNode(const ObjectData& mesh, const std::vector<size_t>& indices, const Ponto& minimo, const Ponto& maximo, int profundidade);
    ~MeshOctreeNode();
    
    void search_triangle(const ObjectData& obj, const Ponto& origem, const Vetor& direcao, 
                         double& closest_t, int& hit_idx, double& u, double& v);
};

// =========================================================================
// 2. OCTREE GLOBAL - Acelerador para objetos da cena
// =========================================================================
class OctreeNode {
public:
    Ponto c_min, c_max;
    std::vector<const ObjectData*> objetos;
    OctreeNode* filhos[8];
    bool eh_folha;

    OctreeNode(const std::vector<const ObjectData*>& objs_entrada, const Ponto& minimo, const Ponto& maximo, int profundidade);
    ~OctreeNode();
    
    HitResult search(const Ponto& origem, const Vetor& direcao);

private:
    bool intersect_aabb_dist(const Ponto& origem, const Vetor& direcao, const Ponto& min_box, const Ponto& max_box, double& t_entrada) const;

public:
    void search_triangle(const ObjectData& obj, const Ponto& origem, const Vetor& direcao, 
                         double& t, int& hit_idx, double& u, double& v);
};