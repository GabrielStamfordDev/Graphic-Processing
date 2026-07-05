#pragma once
#include <cmath>
#include <limits>
#include <tuple>
#include "../src/Ponto.h"
#include "../src/Vetor.h"
#include "hitResult.h"
#include "../utils/scene/sceneSchema.hpp" 

double infinity(){
    return std::numeric_limits<double>::infinity();
}

namespace{
    constexpr double kEpsilon = 1e-8;
    constexpr double kMinT = 0.001;
}

bool intersect_aabb(const Ponto& ray_origin, const Vetor& ray_dir, const Ponto& aabb_min, const Ponto& aabb_max) {
    double tx1 = (aabb_min.getX() - ray_origin.getX()) / ray_dir.getX();
    double tx2 = (aabb_max.getX() - ray_origin.getX()) / ray_dir.getX();
    double tmin = std::min(tx1, tx2);
    double tmax = std::max(tx1, tx2);

    double ty1 = (aabb_min.getY() - ray_origin.getY()) / ray_dir.getY();
    double ty2 = (aabb_max.getY() - ray_origin.getY()) / ray_dir.getY();
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    double tz1 = (aabb_min.getZ() - ray_origin.getZ()) / ray_dir.getZ();
    double tz2 = (aabb_max.getZ() - ray_origin.getZ()) / ray_dir.getZ();
    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    return tmax >= tmin && tmax > 0.0;
}

std::pair<double, Vetor> intersect_sphere(const Ponto& origem, const Vetor& direcao, const Ponto& centro, double raio){
    Vetor v = origem - centro;
    double v_dot_d = v.dot(direcao);
    double v_dot_v = v.dot(v);
    double r2 = raio * raio;
    double discriminant = (v_dot_d * v_dot_d) - (v_dot_v - r2);
    if(discriminant < 0.0) return {infinity(), Vetor()};

    double sqrt_disc = std::sqrt(discriminant);
    double t1 = -v_dot_d - sqrt_disc;
    double t2 = -v_dot_d + sqrt_disc;
    double t = infinity();
    
    if(t1 > kMinT) t = t1;
    else if(t2 > kMinT) t = t2;
    
    if(t == infinity()) return {infinity(), Vetor()};

    Ponto P = origem + (direcao * t);
    Vetor normal = (P - centro).normalize();
    return {t, normal};
}

std::pair<double, Vetor> intersect_plane(const Ponto& origem, const Vetor& direcao, const Ponto& p0, const Vetor& normal) {
    double denom = direcao.dot(normal);
    if(std::abs(denom) > 1e-6){
        Vetor p0_origem = p0 - origem;
        double t = p0_origem.dot(normal)/denom;
        if(t > kMinT) return {t, normal};
    }
    return {infinity(), Vetor()};
}

std::pair<double, Vetor> intersect_cylinder(const Ponto& origem, const Vetor& direcao, const Ponto& centro, const Vetor& eixo, double raio, double altura) {
    double closest_t = infinity();
    Vetor normal_hit;

    Vetor D = direcao;
    Vetor V = eixo;
    Vetor X = origem - centro;

    double D_dot_V = D.dot(V);
    double X_dot_V = X.dot(V);

    // 1. Corpo lateral cilindro genérico orientado por V
    double a = D.dot(D) - (D_dot_V * D_dot_V);
    double b = 2.0 * (D.dot(X) - (D_dot_V * X_dot_V));
    double c = X.dot(X) - (X_dot_V * X_dot_V) - (raio * raio);

    if (std::abs(a) > 1e-6) {
        double discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0.0) {
            double sqrt_disc = std::sqrt(discriminant);
            double t1 = (-b - sqrt_disc) / (2.0 * a);
            double t2 = (-b + sqrt_disc) / (2.0 * a);

            for (double t : {t1, t2}) {
                if (t > kMinT && t < closest_t) {
                    Ponto P = origem + (D * t);
                    double projecao = (P - centro).dot(V);
                    if (projecao >= 0.0 && projecao <= altura) {
                        closest_t = t;
                        Ponto ponto_no_eixo = centro + (V * projecao);
                        normal_hit = (P - ponto_no_eixo).normalize();
                    }
                }
            }
        }
    }

    // 2. Tampas planas orientadas perpendicularmente a V
    // Tampa de baixo (plano passando por centro com normal -V)
    double denom = D.dot(V);
    if (std::abs(denom) > 1e-6) {
        double t_inf = -X.dot(V) / denom;
        if (t_inf > kMinT && t_inf < closest_t) {
            Ponto P = origem + (D * t_inf);
            Vetor dist_base = P - centro;
            if (dist_base.dot(dist_base) - (dist_base.dot(V) * dist_base.dot(V)) <= raio * raio) {
                closest_t = t_inf;
                normal_hit = V * -1.0;
            }
        }

        // Tampa de cima (plano passando por centro + V*altura com normal V)
        Ponto centro_superior = centro + (V * altura);
        double t_sup = (centro_superior - origem).dot(V) / denom;
        if (t_sup > kMinT && t_sup < closest_t) {
            Ponto P = origem + (D * t_sup);
            Vetor dist_topo = P - centro_superior;
            if (dist_topo.dot(dist_topo) - (dist_topo.dot(V) * dist_topo.dot(V)) <= raio * raio) {
                closest_t = t_sup;
                normal_hit = V;
            }
        }
    }

    return {closest_t, normal_hit};
}

std::pair<double, Vetor> intersect_cone(const Ponto& origem, const Vetor& direcao, const Ponto& centro, const Vetor& eixo, double raio, double altura) {
    double closest_t = infinity();
    Vetor normal_hit;

    Vetor D = direcao;
    Vetor V = eixo;
    Ponto A_apex = centro + (V * altura);
    Vetor X = origem - A_apex;

    double cos_sq_theta = (altura * altura) / (raio * raio + altura * altura);

    double D_dot_V = D.dot(V);
    double X_dot_V = X.dot(V);

    // 1. Corpo cônico genérico
    double a = (D_dot_V * D_dot_V) - D.dot(D) * cos_sq_theta;
    double b = 2.0 * ((D_dot_V * X_dot_V) - D.dot(X) * cos_sq_theta);
    double c = (X_dot_V * X_dot_V) - X.dot(X) * cos_sq_theta;

    double discriminant = b * b - 4.0 * a * c;
    if (discriminant >= 0.0) {
        double sqrt_disc = std::sqrt(discriminant);
        double t1 = (-b - sqrt_disc) / (2.0 * a);
        double t2 = (-b + sqrt_disc) / (2.0 * a);

        for (double t : {t1, t2}) {
            if (t > kMinT && t < closest_t) {
                Ponto P = origem + (D * t);
                double projecao = (P - centro).dot(V);
                if (projecao >= 0.0 && projecao <= altura) {
                    closest_t = t;
                    Vetor r_vetor = P - A_apex;
                    Vetor ortogonal_ao_eixo = r_vetor - V * r_vetor.dot(V);
                    Vetor direcao_normal = ortogonal_ao_eixo.normalize();
                    double tan_theta = raio / altura;
                    normal_hit = (direcao_normal + V * tan_theta).normalize();
                }
            }
        }
    }

    // 2. Base do Cone plana (plano passando por centro com normal -V)
    double denom = D.dot(V);
    if (std::abs(denom) > 1e-6) {
        double t_base = (centro - origem).dot(V) / denom;
        if (t_base > kMinT && t_base < closest_t) {
            Ponto P = origem + (D * t_base);
            Vetor dist_base = P - centro;
            if (dist_base.dot(dist_base) - (dist_base.dot(V) * dist_base.dot(V)) <= raio * raio) {
                closest_t = t_base;
                normal_hit = V * -1.0;
            }
        }
    }

    return {closest_t, normal_hit};
}

std::tuple<double, double, double> intersect_triangle_uvt(const Ponto& origem, const Vetor& direcao, const Ponto& v0, const Ponto& v1, const Ponto& v2){
    Vetor aresta1 = v1 - v0;
    Vetor aresta2 = v2 - v0;
    Vetor h = direcao.cross(aresta2);
    double a = aresta1.dot(h);
    if(std::abs(a) < kEpsilon) return {infinity(), 0.0, 0.0};

    double det = 1.0 / a;
    Vetor s = origem - v0;
    double alfa = det * s.dot(h);
    if(alfa < 0.0 || alfa > 1.0) return {infinity(), 0.0, 0.0};

    Vetor q = s.cross(aresta1);
    double beta = det * direcao.dot(q);
    if(beta < 0.0 || (alfa + beta) > 1.0) return {infinity(), 0.0, 0.0};

    double t = det * aresta2.dot(q);
    if(t > kMinT) return {t, alfa, beta};

    return {infinity(), 0.0, 0.0};
}

double intersect_triangle(const Ponto& origem, const Vetor& direcao, const Ponto& v0, const Ponto& v1, const Ponto& v2) {
    Vetor aresta1 = v1 - v0;
    Vetor aresta2 = v2 - v0;
    Vetor h = direcao.cross(aresta2);
    double a = aresta1.dot(h);
    if(std::abs(a) < kEpsilon) return infinity();

    double det = 1.0 / a;
    Vetor s = origem - v0;
    double alfa = det * s.dot(h);
    if(alfa < 0.0 || alfa > 1.0) return infinity();

    Vetor q = s.cross(aresta1);
    double beta = det * direcao.dot(q);
    if(beta < 0.0 || (alfa + beta) > 1.0) return infinity();

    double t = det * aresta2.dot(q);
    if(t > kMinT) return t;

    return infinity();
}

HitResult intersect_object(const ObjectData& obj, const Ponto& ray_origin, const Vetor& ray_dir){
    if(obj.objType == "sphere"){
        double raio = obj.numericData.at("radius");
        auto [t, normal] = intersect_sphere(ray_origin, ray_dir, obj.relativePos, raio);
        return {t, normal};
    }

    if(obj.objType == "plane"){
        Vetor normal = obj.vetorPointData.at("normal").normalize();
        auto [t, n]  = intersect_plane(ray_origin, ray_dir, obj.relativePos, normal);
        return {t, n};
    }

    if(obj.objType == "cylinder"){
        double raio = obj.numericData.at("radius");
        double altura = obj.numericData.at("height");
        Vetor eixo = obj.vetorPointData.at("eixo");
        auto [t, normal] = intersect_cylinder(ray_origin, ray_dir, obj.relativePos, eixo, raio, altura);
        return {t, normal};
    }

    if(obj.objType == "cone"){
        double raio = obj.numericData.at("radius");
        double altura = obj.numericData.at("height");
        Vetor eixo = obj.vetorPointData.at("eixo");
        auto [t, normal] = intersect_cone(ray_origin, ray_dir, obj.relativePos, eixo, raio, altura);
        return {t, normal};
    }

    if(obj.objType == "mesh"){
        if (obj.has_aabb) {
            if (!intersect_aabb(ray_origin, ray_dir, obj.aabb_min, obj.aabb_max)) {
                return {infinity(), Vetor()}; 
            }
        }
        
        double closest_t = infinity();
        int    hit_idx   = -1;
        double    hit_u = 0.0, hit_v = 0.0;
        for(size_t i = 0; i < obj.mesh_v0.size(); ++i){
            auto [t, u, v] = intersect_triangle_uvt(
                ray_origin, ray_dir,
                obj.mesh_v0[i], obj.mesh_v1[i], obj.mesh_v2[i]);
            if(t > 1e-4 && t < closest_t){
                closest_t = t;
                hit_idx   = (int)i;
                hit_u     = u;
                hit_v     = v;
            }
        }
        if(hit_idx >= 0){
            const Vetor& n0 = obj.mesh_n0[hit_idx];
            const Vetor& n1 = obj.mesh_n1[hit_idx];
            const Vetor& n2 = obj.mesh_n2[hit_idx];
            double peso_n0  = 1.0 - hit_u - hit_v;
            Vetor  normal_interp = (n0 * peso_n0) + (n1 * hit_u) + (n2 * hit_v);
            return {closest_t, normal_interp.normalize()};
        }
        return {infinity(), Vetor()};
    }
    return {infinity(), Vetor()};
}