#include "transformacoes.h"
#include <cmath>

namespace{
    constexpr double kPi = 3.141592653589793238462643383279502884;
    double graus_para_radianos(double angulo_graus){
        return angulo_graus * (kPi / 180.0);
    }
}

Matriz4x4 operator*(const Matriz4x4& A, const Matriz4x4& B){
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


Matriz4x4 matriz_translacao(double dx, double dy, double dz){
	Matriz4x4 matriz{};
	matriz[0] = {1.0, 0.0, 0.0, dx};
	matriz[1] = {0.0, 1.0, 0.0, dy};
	matriz[2] = {0.0, 0.0, 1.0, dz};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_escala(double sx, double sy, double sz){
	Matriz4x4 matriz{};
	matriz[0] = {sx,  0.0, 0.0, 0.0};
	matriz[1] = {0.0, sy,  0.0, 0.0};
	matriz[2] = {0.0, 0.0, sz,  0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_rotacao_x(double rad){
	double c = std::cos(rad);
	double s = std::sin(rad);
	Matriz4x4 matriz{};
	matriz[0] = {1.0, 0.0, 0.0, 0.0};
	matriz[1] = {0.0, c,   -s,  0.0};
	matriz[2] = {0.0, s,    c,  0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_rotacao_y(double rad){
	double c = std::cos(rad);
	double s = std::sin(rad);
	Matriz4x4 matriz{};
	matriz[0] = { c,  0.0, s,   0.0};
	matriz[1] = {0.0, 1.0, 0.0, 0.0};
	matriz[2] = {-s,  0.0, c,   0.0};
	matriz[3] = {0.0, 0.0, 0.0, 1.0};
	return matriz;
}

Matriz4x4 matriz_rotacao_z(double rad){
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

Vetor aplicar_matriz_normal(const Matriz4x4& matriz, const Vetor& normal) {
	double a = matriz[0][0];
	double b = matriz[0][1];
	double c = matriz[0][2];
	double d = matriz[1][0];
	double e = matriz[1][1];
	double f = matriz[1][2];
	double g = matriz[2][0];
	double h = matriz[2][1];
	double i = matriz[2][2];

	double det = a * (e * i - f * h)
	           - b * (d * i - f * g)
	           + c * (d * h - e * g);

	if (std::abs(det) == 0.0) {
		return normal.normalize();
	}

	double inv00 =  (e * i - f * h) / det;
	double inv01 =  (c * h - b * i) / det;
	double inv02 =  (b * f - c * e) / det;
	double inv10 =  (f * g - d * i) / det;
	double inv11 =  (a * i - c * g) / det;
	double inv12 =  (c * d - a * f) / det;
	double inv20 =  (d * h - e * g) / det;
	double inv21 =  (b * g - a * h) / det;
	double inv22 =  (a * e - b * d) / det;

	double x = inv00 * normal.getX() + inv10 * normal.getY() + inv20 * normal.getZ();
	double y = inv01 * normal.getX() + inv11 * normal.getY() + inv21 * normal.getZ();
	double z = inv02 * normal.getX() + inv12 * normal.getY() + inv22 * normal.getZ();

	return Vetor(x, y, z).normalize();
}

double crescimento_raio(const Matriz4x4& m){
	double sx = sqrt(
		m[0][0] * m[0][0] +
		m[1][0] * m[1][0] +
		m[2][0] * m[2][0]
	);
	double sy = sqrt(
		m[0][1] * m[0][1] +
		m[1][1] * m[1][1] +
		m[2][1] * m[2][1]
	);
	double sz = sqrt(
		m[0][2] * m[0][2] +
		m[1][2] * m[1][2] +
		m[2][2] * m[2][2]
	);
	double s = (sx + sy + sz) / 3.0; //formalismo
	return s;
}

Matriz4x4 matriz_inversa(const Matriz4x4& m) {
    // Expansão de cofatores 4x4
    // Cada elemento [i][j] da inversa = cofator(j,i) / determinante
    double inv[4][4];

    inv[0][0] =  (m[1][1]*(m[2][2]*m[3][3]-m[2][3]*m[3][2]) - m[1][2]*(m[2][1]*m[3][3]-m[2][3]*m[3][1]) + m[1][3]*(m[2][1]*m[3][2]-m[2][2]*m[3][1]));
    inv[1][0] = -(m[1][0]*(m[2][2]*m[3][3]-m[2][3]*m[3][2]) - m[1][2]*(m[2][0]*m[3][3]-m[2][3]*m[3][0]) + m[1][3]*(m[2][0]*m[3][2]-m[2][2]*m[3][0]));
    inv[2][0] =  (m[1][0]*(m[2][1]*m[3][3]-m[2][3]*m[3][1]) - m[1][1]*(m[2][0]*m[3][3]-m[2][3]*m[3][0]) + m[1][3]*(m[2][0]*m[3][1]-m[2][1]*m[3][0]));
    inv[3][0] = -(m[1][0]*(m[2][1]*m[3][2]-m[2][2]*m[3][1]) - m[1][1]*(m[2][0]*m[3][2]-m[2][2]*m[3][0]) + m[1][2]*(m[2][0]*m[3][1]-m[2][1]*m[3][0]));

    double det = m[0][0]*inv[0][0] + m[0][1]*inv[1][0] + m[0][2]*inv[2][0] + m[0][3]*inv[3][0];

    // Matriz singular — retorna identidade como fallback seguro
    if(std::abs(det) < 1e-12) return matriz_identidade();

    double inv_det = 1.0 / det;

    inv[0][1] = -(m[0][1]*(m[2][2]*m[3][3]-m[2][3]*m[3][2]) - m[0][2]*(m[2][1]*m[3][3]-m[2][3]*m[3][1]) + m[0][3]*(m[2][1]*m[3][2]-m[2][2]*m[3][1]));
    inv[1][1] =  (m[0][0]*(m[2][2]*m[3][3]-m[2][3]*m[3][2]) - m[0][2]*(m[2][0]*m[3][3]-m[2][3]*m[3][0]) + m[0][3]*(m[2][0]*m[3][2]-m[2][2]*m[3][0]));
    inv[2][1] = -(m[0][0]*(m[2][1]*m[3][3]-m[2][3]*m[3][1]) - m[0][1]*(m[2][0]*m[3][3]-m[2][3]*m[3][0]) + m[0][3]*(m[2][0]*m[3][1]-m[2][1]*m[3][0]));
    inv[3][1] =  (m[0][0]*(m[2][1]*m[3][2]-m[2][2]*m[3][1]) - m[0][1]*(m[2][0]*m[3][2]-m[2][2]*m[3][0]) + m[0][2]*(m[2][0]*m[3][1]-m[2][1]*m[3][0]));

    inv[0][2] =  (m[0][1]*(m[1][2]*m[3][3]-m[1][3]*m[3][2]) - m[0][2]*(m[1][1]*m[3][3]-m[1][3]*m[3][1]) + m[0][3]*(m[1][1]*m[3][2]-m[1][2]*m[3][1]));
    inv[1][2] = -(m[0][0]*(m[1][2]*m[3][3]-m[1][3]*m[3][2]) - m[0][2]*(m[1][0]*m[3][3]-m[1][3]*m[3][0]) + m[0][3]*(m[1][0]*m[3][2]-m[1][2]*m[3][0]));
    inv[2][2] =  (m[0][0]*(m[1][1]*m[3][3]-m[1][3]*m[3][1]) - m[0][1]*(m[1][0]*m[3][3]-m[1][3]*m[3][0]) + m[0][3]*(m[1][0]*m[3][1]-m[1][1]*m[3][0]));
    inv[3][2] = -(m[0][0]*(m[1][1]*m[3][2]-m[1][2]*m[3][1]) - m[0][1]*(m[1][0]*m[3][2]-m[1][2]*m[3][0]) + m[0][2]*(m[1][0]*m[3][1]-m[1][1]*m[3][0]));

    inv[0][3] = -(m[0][1]*(m[1][2]*m[2][3]-m[1][3]*m[2][2]) - m[0][2]*(m[1][1]*m[2][3]-m[1][3]*m[2][1]) + m[0][3]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]));
    inv[1][3] =  (m[0][0]*(m[1][2]*m[2][3]-m[1][3]*m[2][2]) - m[0][2]*(m[1][0]*m[2][3]-m[1][3]*m[2][0]) + m[0][3]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]));
    inv[2][3] = -(m[0][0]*(m[1][1]*m[2][3]-m[1][3]*m[2][1]) - m[0][1]*(m[1][0]*m[2][3]-m[1][3]*m[2][0]) + m[0][3]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]));
    inv[3][3] =  (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]));

    Matriz4x4 result = matriz_identidade();
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            result[i][j] = inv[i][j] * inv_det;

    return result;
}

Matriz4x4 build_transform_matriz(const vector<TransformData>& transforma,const Ponto& relativePos, const string& tipo){
    Matriz4x4 M_total = matriz_identidade();
    Matriz4x4 M_atual;
    Matriz4x4 Mx, My, Mz, Mpinv, Mp, ME;
    Ponto pivo_acumulado;

    if(tipo == "mesh") pivo_acumulado = Ponto(0, 0, 0);
    else pivo_acumulado = relativePos;

    for(const auto& t : transforma){
        M_atual = matriz_identidade();
        if(t.tType == "scaling" && tipo != "plane"){
            Mpinv = matriz_translacao(-pivo_acumulado.getX(),-pivo_acumulado.getY(),-pivo_acumulado.getZ());
            Mp = matriz_translacao(pivo_acumulado.getX(),pivo_acumulado.getY(),pivo_acumulado.getZ());
            ME = matriz_escala(t.data.getX(), t.data.getY(), t.data.getZ());
            M_atual = Mp * ME * Mpinv;
        }
        else if(t.tType == "translation"){
            M_atual = matriz_translacao(t.data.getX(),t.data.getY(),t.data.getZ());
            pivo_acumulado = aplicar_matriz_ponto(M_atual, pivo_acumulado);
        }
        else if(t.tType == "rotation" && tipo != "sphere"){
            Mx = matriz_rotacao_x(t.data.getX());
            My = matriz_rotacao_y(t.data.getY());
            Mz = matriz_rotacao_z(t.data.getZ());
            Mpinv = matriz_translacao(-pivo_acumulado.getX(),-pivo_acumulado.getY(),-pivo_acumulado.getZ());
            Mp = matriz_translacao(pivo_acumulado.getX(),pivo_acumulado.getY(),pivo_acumulado.getZ());
            M_atual =  Mp * Mz * My * Mx * Mpinv;
        }
        M_total = M_atual * M_total;
    }

    if(tipo == "mesh"){
        Mp = matriz_translacao(relativePos.getX(),relativePos.getY(),relativePos.getZ());
        M_total = Mp * M_total;
    }

    return M_total;
}

Vetor aplicar_matriz_normal_inv_t(const Matriz4x4& M, const Vetor& n){
    Matriz4x4 inv = matriz_inversa(M);
    double x = inv[0][0]*n.getX() + inv[1][0]*n.getY() + inv[2][0]*n.getZ();
    double y = inv[0][1]*n.getX() + inv[1][1]*n.getY() + inv[2][1]*n.getZ();
    double z = inv[0][2]*n.getX() + inv[1][2]*n.getY() + inv[2][2]*n.getZ();
    return Vetor(x, y, z).normalize();
}