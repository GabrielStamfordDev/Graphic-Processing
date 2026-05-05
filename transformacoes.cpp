#include "transformacoes.h"
#include <cmath>

namespace {
    constexpr double kPi = 3.141592653589793238462643383279502884;
    double graus_para_radianos(double angulo_graus){
        return angulo_graus * (kPi / 180.0);
    }
}

Matriz4x4 operator*(const Matriz4x4& A, const Matriz4x4& B) {
    Matriz4x4 C{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            C[i][j] = 0.0;
            for (int k = 0; k < 4; ++k)
                C[i][j] += A[i][k] * B[k][j];
        }
    }
    return C;
}

Matriz4x4 matriz_translacao(double dx, double dy, double dz) {
	Matriz4x4 matriz{};
	matriz[0] = {1.0, 0.0, 0.0, dx};
	matriz[1] = {0.0, 1.0, 0.0, dy};
	matriz[2] = {0.0, 0.0, 1.0, dz};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_escala(double sx, double sy, double sz) {
	Matriz4x4 matriz{};
	matriz[0] = {sx,  0.0, 0.0, 0.0};
	matriz[1] = {0.0, sy,  0.0, 0.0};
	matriz[2] = {0.0, 0.0, sz,  0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_rotacao_x(double angulo_graus) {
	double rad = graus_para_radianos(angulo_graus);
	double c = std::cos(rad);
	double s = std::sin(rad);
	Matriz4x4 matriz{};
	matriz[0] = {1.0, 0.0, 0.0, 0.0};
	matriz[1] = {0.0, c,   -s,  0.0};
	matriz[2] = {0.0, s,    c,  0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_rotacao_y(double angulo_graus) {
	double rad = graus_para_radianos(angulo_graus);
	double c = std::cos(rad);
	double s = std::sin(rad);
	Matriz4x4 matriz{};
	matriz[0] = { c,  0.0, s,   0.0};
	matriz[1] = {0.0, 1.0, 0.0, 0.0};
	matriz[2] = {-s,  0.0, c,   0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_rotacao_z(double angulo_graus) {
	double rad = graus_para_radianos(angulo_graus);
	double c = std::cos(rad);
	double s = std::sin(rad);
	Matriz4x4 matriz{};
	matriz[0] = {c,   -s,  0.0, 0.0};
	matriz[1] = {s,    c,  0.0, 0.0};
	matriz[2] = {0.0, 0.0, 1.0, 0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_identidade(){
	Matriz4x4 matriz{};
	matriz[0] = {1.0, 0.0, 0.0, 0.0};
	matriz[1] = {0.0, 1.0, 0.0, 0.0};
	matriz[2] = {0.0, 0.0, 1.0, 0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Ponto aplicar_matriz_ponto(const Matriz4x4& matriz, const Ponto& ponto) {
	double x = matriz[0][0] * ponto.getX() + matriz[0][1] * ponto.getY() + matriz[0][2] * ponto.getZ() + matriz[0][3];
	double y = matriz[1][0] * ponto.getX() + matriz[1][1] * ponto.getY() + matriz[1][2] * ponto.getZ() + matriz[1][3];
	double z = matriz[2][0] * ponto.getX() + matriz[2][1] * ponto.getY() + matriz[2][2] * ponto.getZ() + matriz[2][3];
	return Ponto(x, y, z);
}

Vetor aplicar_matriz_vetor(const Matriz4x4& matriz, const Vetor& vetor) {
	double x = matriz[0][0] * vetor.getX() + matriz[0][1] * vetor.getY() + matriz[0][2] * vetor.getZ();
	double y = matriz[1][0] * vetor.getX() + matriz[1][1] * vetor.getY() + matriz[1][2] * vetor.getZ();
	double z = matriz[2][0] * vetor.getX() + matriz[2][1] * vetor.getY() + matriz[2][2] * vetor.getZ();
	return Vetor(x, y, z);
}
