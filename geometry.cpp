#pragma once
#include <cmath>
#include <limits>
#include "src/Ponto.h"
#include "src/Vetor.h"

namespace{
    constexpr double kEpsilon = 1e-8;
    constexpr double kMinT = 0.001;
    double infinity(){
        return std::numeric_limits<double>::infinity();
    }
}

double intersect_sphere(const Ponto& origem, const Vetor& direcao, const Ponto& centro, double raio){
	Vetor v = origem - centro;
	double v_dot_d = v.dot(direcao);
	double v_dot_v = v.dot(v);
	double r2 = raio * raio;
	double discriminant = (v_dot_d * v_dot_d) - (v_dot_v - r2);
	if(discriminant < 0.0) return infinity();

	double sqrt_disc = std::sqrt(discriminant);
	double t1 = -v_dot_d - sqrt_disc;
	double t2 = -v_dot_d + sqrt_disc;
	if(t1 > kMinT) return t1;
	if(t2 > kMinT) return t2;

	return infinity();
}

double intersect_plane(const Ponto& origem, const Vetor& direcao, const Ponto& p0, const Vetor& normal) {
	double denom = direcao.dot(normal);
	if(std::abs(denom) > 1e-6){
		Vetor p0_origem = p0 - origem;
		double t = p0_origem.dot(normal)/denom;
		if(t > kMinT) return t;
	}
	return infinity();
}

double intersect_triangle(const Ponto& origem, const Vetor& direcao, const Ponto& v0, const Ponto& v1, const Ponto& v2) {
	Vetor aresta1 = v1 - v0;
	Vetor aresta2 = v2 - v0;
	Vetor h = direcao.cross(aresta2);
	double a = aresta1.dot(h);
	if(std::abs(a) < kEpsilon) return infinity();

	double f = 1.0 / a;
	Vetor s = origem - v0;
	double u = f * s.dot(h);
	if(u < 0.0 || u > 1.0) return infinity();

	Vetor q = s.cross(aresta1);
	double v = f * direcao.dot(q);
	if(v < 0.0 || (u + v) > 1.0) return infinity();

	double t = f * aresta2.dot(q);
	if(t > kMinT) return t;

	return infinity();
}