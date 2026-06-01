#pragma once
#include <array>
#include "../src/Ponto.h"
#include "../src/Vetor.h"
#include "../utils/Scene/sceneSchema.hpp"

using Matriz4x4 = std::array<std::array<double, 4>, 4>;

Matriz4x4 matriz_identidade();
Matriz4x4 operator*(const Matriz4x4& A, const Matriz4x4& B);
Matriz4x4 matriz_translacao(double dx, double dy, double dz);
Matriz4x4 matriz_escala(double sx, double sy, double sz);
Matriz4x4 matriz_rotacao_x(double angulo_graus);
Matriz4x4 matriz_rotacao_y(double angulo_graus);
Matriz4x4 matriz_rotacao_z(double angulo_graus);
Matriz4x4 matriz_inversa(const Matriz4x4& m);
Matriz4x4 build_transform_matriz(const vector<TransformData>& A,const Ponto& B, const string& C);
Vetor aplicar_matriz_normal_inv_t(const Matriz4x4& M, const Vetor& n);

Ponto aplicar_matriz_ponto(const Matriz4x4& matriz, const Ponto& ponto);
Vetor aplicar_matriz_vetor(const Matriz4x4& matriz, const Vetor& vetor);
Vetor aplicar_matriz_normal(const Matriz4x4& matriz, const Vetor& normal);

double crescimento_raio(const Matriz4x4& matriz);