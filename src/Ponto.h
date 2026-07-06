#ifndef PONTOHEADER
#define PONTOHEADER

#include <iostream>
#include "Vetor.h"

class Ponto {
public:
    Ponto(double x = 0, double y = 0, double z = 0)
        : x(x), y(y), z(z) {}

    // Ponto + Vetor → Ponto (translação)
    Ponto operator+(const Vetor& v) const {
        return Ponto(x + v.getX(), y + v.getY(), z + v.getZ());
    }

    Ponto& operator+=(const Vetor& v) {
        x += v.getX();
        y += v.getY();
        z += v.getZ();
        return *this;
    }

    // Ponto - Vetor → Ponto (recuo)
    Ponto operator-(const Vetor& v) const {
        return Ponto(x - v.getX(), y - v.getY(), z - v.getZ());
    }

    Ponto& operator-=(const Vetor& v) {
        x -= v.getX();
        y -= v.getY();
        z -= v.getZ();
        return *this;
    }

    // Ponto - Ponto → Vetor (direção entre pontos)
    Vetor operator-(const Ponto& p) const {
        return Vetor(x - p.getX(), y - p.getY(), z - p.getZ());
    }

    bool operator==(const Ponto& p) const {
        return x == p.x && y == p.y && z == p.z;
    }

    bool operator!=(const Ponto& p) const {
        return !(*this == p);
    }
    // Função auxiliar de deslocamento que respeita sua estrutura atual
    Ponto deslocarPonto(const Ponto& p, double valor) {
        // Como você já tem Vetor(x, y, z), usamos ele como intermediário
        // Ponto + Vetor = Ponto (isso você JÁ tem implementado)
        return p + Vetor(valor, valor, valor);
    }

    Ponto subtrairDeslocamento(const Ponto& p, double valor) {
        // Ponto - Vetor = Ponto (isso você JÁ tem implementado)
        return p - Vetor(valor, valor, valor);
    }

    // impressão
    friend std::ostream& operator<<(std::ostream& os, const Ponto& p) {
        return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    }

    // getters (mantidos para compatibilidade com seu código atual)
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

private:
    double x, y, z;
};

#endif