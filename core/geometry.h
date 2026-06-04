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

    if(obj.objType == "mesh"){
        double closest_t = infinity();
        int      hit_idx   = -1;
        double   hit_u = 0.0, hit_v = 0.0;
        for(size_t i = 0; i < obj.mesh_v0.size(); ++i){
            auto [t, u, v] = intersect_triangle_uvt(
                ray_origin, ray_dir,
                obj.mesh_v0[i], obj.mesh_v1[i], obj.mesh_v2[i]);
            if(t < closest_t){
                closest_t = t;
                hit_idx   = (int)i;
                hit_u     = u;
                hit_v     = v;
            }
        }
        if(hit_idx >= 0){
            // Interpolação baricêntrica das normais por vértice (Phong shading)
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