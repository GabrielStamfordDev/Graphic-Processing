#ifndef VETORHEADER
#define VETORHEADER

#include <iostream>
#include <cmath>

class Vetor {
public:
    Vetor(double x = 0, double y = 0, double z = 0)
        : x(x), y(y), z(z) {}

    // Vetor + Vetor
    Vetor operator+(const Vetor& v) const {
        return Vetor(x + v.x, y + v.y, z + v.z);
    }

    // Vetor - Vetor
    Vetor operator-(const Vetor& v) const {
        return Vetor(x - v.x, y - v.y, z - v.z);
    }

    // Escalar * Vetor
    Vetor operator*(double s) const {
        return Vetor(x * s, y * s, z * s);
    }

    friend Vetor operator*(double s, const Vetor& v) {
        return Vetor(v.x * s, v.y * s, v.z * s);
    }

    // Produto escalar
    double dot(const Vetor& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Produto vetorial
    Vetor cross(const Vetor& v) const {
        return Vetor(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    double length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vetor normalize() const {
        double len = length();
        if (len == 0) return Vetor(0, 0, 0);
        return Vetor(x / len, y / len, z / len);
    }

    friend std::ostream& operator<<(std::ostream& os, const Vetor &v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }

    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

private:
    double x, y, z;
};

#endif