#include "octree.h"
#include "geometry.h"

// Forward declaration
std::tuple<double, double, double> intersect_triangle_uvt(const Ponto& origem, const Vetor& direcao, const Ponto& v0, const Ponto& v1, const Ponto& v2);

// =========================================================================
// MESH OCTREE NODE - IMPLEMENTAÇÃO
// =========================================================================
MeshOctreeNode::MeshOctreeNode(const ObjectData& mesh, const std::vector<size_t>& indices, const Ponto& minimo, const Ponto& maximo, int profundidade) {
    c_min = minimo; c_max = maximo; eh_folha = true;
    for (int i = 0; i < 8; ++i) filhos[i] = nullptr;

    if (indices.size() <= 4 || profundidade >= 6) {
        indices_triangulos = indices;
        return;
    }

    eh_folha = false;
    double mid_x = (c_min.getX() + c_max.getX()) * 0.5;
    double mid_y = (c_min.getY() + c_max.getY()) * 0.5;
    double mid_z = (c_min.getZ() + c_max.getZ()) * 0.5;

    std::vector<size_t> indices_filhos[8];
    for (size_t idx : indices) {
        const Ponto& v0 = mesh.mesh_v0[idx];
        const Ponto& v1 = mesh.mesh_v1[idx];
        const Ponto& v2 = mesh.mesh_v2[idx];

        double tMinX = std::min({v0.getX(), v1.getX(), v2.getX()});
        double tMaxX = std::max({v0.getX(), v1.getX(), v2.getX()});
        double tMinY = std::min({v0.getY(), v1.getY(), v2.getY()});
        double tMaxY = std::max({v0.getY(), v1.getY(), v2.getY()});
        double tMinZ = std::min({v0.getZ(), v1.getZ(), v2.getZ()});
        double tMaxZ = std::max({v0.getZ(), v1.getZ(), v2.getZ()});

        int octante = -1;
        for (int i = 0; i < 8; ++i) {
            double f_min_x = (i & 1) ? mid_x : c_min.getX();
            double f_max_x = (i & 1) ? c_max.getX() : mid_x;
            double f_min_y = (i & 2) ? mid_y : c_min.getY();
            double f_max_y = (i & 2) ? c_max.getY() : mid_y;
            double f_min_z = (i & 4) ? mid_z : c_min.getZ();
            double f_max_z = (i & 4) ? c_max.getZ() : mid_z;

            if (tMinX >= f_min_x && tMaxX <= f_max_x &&
                tMinY >= f_min_y && tMaxY <= f_max_y &&
                tMinZ >= f_min_z && tMaxZ <= f_max_z) {
                octante = i; break;
            }
        }

        if (octante != -1) indices_filhos[octante].push_back(idx);
        else this->indices_triangulos.push_back(idx); // Fica no pai
    }

    for (int i = 0; i < 8; ++i) {
        if (!indices_filhos[i].empty()) {
            Ponto f_min((i & 1) ? mid_x : c_min.getX(), (i & 2) ? mid_y : c_min.getY(), (i & 4) ? mid_z : c_min.getZ());
            Ponto f_max((i & 1) ? c_max.getX() : mid_x, (i & 2) ? c_max.getY() : mid_y, (i & 4) ? c_max.getZ() : mid_z);
            filhos[i] = new MeshOctreeNode(mesh, indices_filhos[i], f_min, f_max, profundidade + 1);
        }
    }
}

MeshOctreeNode::~MeshOctreeNode() {
    for (int i = 0; i < 8; ++i) if (filhos[i]) delete filhos[i];
}

void MeshOctreeNode::search_triangle(const ObjectData& obj, const Ponto& origem, const Vetor& direcao, 
                                     double& closest_t, int& hit_idx, double& u, double& v) {
    
    // Teste simples AABB
    double tmin = -get_inf(), tmax = get_inf();
    for (int i = 0; i < 3; ++i) {
        double o = (i == 0) ? origem.getX() : (i == 1 ? origem.getY() : origem.getZ());
        double d = (i == 0) ? direcao.getX() : (i == 1 ? direcao.getY() : direcao.getZ());
        double min_b = (i == 0) ? c_min.getX() : (i == 1 ? c_min.getY() : c_min.getZ());
        double max_b = (i == 0) ? c_max.getX() : (i == 1 ? c_max.getY() : c_max.getZ());
        if (std::abs(d) > 1e-9) {
            double t1 = (min_b - o) / d; double t2 = (max_b - o) / d;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1); tmax = std::min(tmax, t2);
        } else if (o < min_b || o > max_b) return;
    }
    if (tmin > tmax || tmax < 0.0 || tmin >= closest_t) return;

    // Teste triângulos do nó atual (incluindo os "do pai")
    for (size_t idx : indices_triangulos) {
        auto [t, u_tri, v_tri] = intersect_triangle_uvt(origem, direcao, obj.mesh_v0[idx], obj.mesh_v1[idx], obj.mesh_v2[idx]);
        if (t > 1e-4 && t < closest_t) {
            closest_t = t; hit_idx = (int)idx; u = u_tri; v = v_tri;
        }
    }

    if (!eh_folha) {
        for (int i = 0; i < 8; ++i) if (filhos[i]) filhos[i]->search_triangle(obj, origem, direcao, closest_t, hit_idx, u, v);
    }
}

// =========================================================================
// OCTREE GLOBAL - IMPLEMENTAÇÃO
// =========================================================================
OctreeNode::OctreeNode(const std::vector<const ObjectData*>& objs_entrada, const Ponto& minimo, const Ponto& maximo, int profundidade) {
    c_min = minimo; c_max = maximo; eh_folha = true;
    for (int i = 0; i < 8; ++i) filhos[i] = nullptr;

    std::vector<const ObjectData*> objs_filtrados, planos;
    for (const auto& obj : objs_entrada) {
        if (obj->objType == "plane") planos.push_back(obj);
        else {
            auto [oMin, oMax] = getBoundingBox(*obj);
            if (!(oMax.getX() < c_min.getX() || oMin.getX() > c_max.getX() || oMax.getY() < c_min.getY() || oMin.getY() > c_max.getY() || oMax.getZ() < c_min.getZ() || oMin.getZ() > c_max.getZ()))
                objs_filtrados.push_back(obj);
        }
    }

    if (objs_filtrados.size() <= 2 || profundidade >= 6) {
        objetos = objs_filtrados;
        objetos.insert(objetos.end(), planos.begin(), planos.end());
        return;
    }

    eh_folha = false;
    double mid_x = (c_min.getX() + c_max.getX()) * 0.5, mid_y = (c_min.getY() + c_max.getY()) * 0.5, mid_z = (c_min.getZ() + c_max.getZ()) * 0.5;

    std::vector<const ObjectData*> objs_filhos[8];
    for (const auto& obj : objs_filtrados) {
        auto [oMin, oMax] = getBoundingBox(*obj);
        int octante = -1;
        for (int i = 0; i < 8; ++i) {
            double f_min_x = (i & 1) ? mid_x : c_min.getX(), f_max_x = (i & 1) ? c_max.getX() : mid_x;
            double f_min_y = (i & 2) ? mid_y : c_min.getY(), f_max_y = (i & 2) ? c_max.getY() : mid_y;
            double f_min_z = (i & 4) ? mid_z : c_min.getZ(), f_max_z = (i & 4) ? c_max.getZ() : mid_z;
            if (oMin.getX() >= f_min_x && oMax.getX() <= f_max_x && oMin.getY() >= f_min_y && oMax.getY() <= f_max_y && oMin.getZ() >= f_min_z && oMax.getZ() <= f_max_z) {
                octante = i; break;
            }
        }
        if (octante != -1) objs_filhos[octante].push_back(obj);
        else objetos.push_back(obj);
    }

    for (int i = 0; i < 8; ++i) {
        if (!objs_filhos[i].empty()) {
            Ponto f_min((i & 1) ? mid_x : c_min.getX(), (i & 2) ? mid_y : c_min.getY(), (i & 4) ? mid_z : c_min.getZ());
            Ponto f_max((i & 1) ? c_max.getX() : mid_x, (i & 2) ? c_max.getY() : mid_y, (i & 4) ? c_max.getZ() : mid_z);
            filhos[i] = new OctreeNode(objs_filhos[i], f_min, f_max, profundidade + 1);
        }
    }
    objetos.insert(objetos.end(), planos.begin(), planos.end());
}

OctreeNode::~OctreeNode() { for (int i = 0; i < 8; ++i) if (filhos[i]) delete filhos[i]; }

HitResult OctreeNode::search(const Ponto& origem, const Vetor& direcao) {
    double t_nulo = 0.0;
    if (!intersect_aabb_dist(origem, direcao, c_min, c_max, t_nulo)) return { get_inf(), Vetor(), nullptr };
    
    HitResult closest_hr = { get_inf(), Vetor(), nullptr };
    for (const auto* obj : objetos) {
        HitResult hr = intersect_object(*obj, origem, direcao);
        if (hr.hit() && hr.t < closest_hr.t) closest_hr = hr;
    }
    if (!eh_folha) {
        for (int i = 0; i < 8; ++i) if (filhos[i]) {
            HitResult hr = filhos[i]->search(origem, direcao);
            if (hr.hit() && hr.t < closest_hr.t) closest_hr = hr;
        }
    }
    return closest_hr;
}

bool OctreeNode::intersect_aabb_dist(const Ponto& origem, const Vetor& direcao, const Ponto& min_box, const Ponto& max_box, double& t_entrada) const {
    double tmin = -get_inf(), tmax = get_inf();
    for (int i = 0; i < 3; ++i) {
        double o = (i == 0 ? origem.getX() : (i == 1 ? origem.getY() : origem.getZ()));
        double d = (i == 0 ? direcao.getX() : (i == 1 ? direcao.getY() : direcao.getZ()));
        double min_b = (i == 0 ? min_box.getX() : (i == 1 ? min_box.getY() : min_box.getZ()));
        double max_b = (i == 0 ? max_box.getX() : (i == 1 ? max_box.getY() : max_box.getZ()));
        if (std::abs(d) > 1e-9) {
            double t1 = (min_b - o) / d; double t2 = (max_b - o) / d;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1); tmax = std::min(tmax, t2);
        } else if (o < min_b || o > max_b) return false;
    }
    t_entrada = tmin; return tmin <= tmax && tmax >= 0.0;
}