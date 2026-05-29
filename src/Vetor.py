import numpy as np

class Vetor:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self._data = np.array([x, y, z], dtype=np.float64)

    @property
    def x(self): return self._data[0]

    @property
    def y(self): return self._data[1]

    @property
    def z(self): return self._data[2]

    def magnitude(self) -> float:
        return np.linalg.norm(self._data)

    def normalize(self) -> 'Vetor':
        mag = self.magnitude()
        if mag == 0:
            return Vetor(0, 0, 0)
        return Vetor(*(self._data / mag))

    def dot(self, other: 'Vetor') -> float:
        if not hasattr(other, "_data"):
            raise TypeError("dot apenas entre Vetores")
        return float(np.dot(self._data, other._data))

    def cross(self, other: 'Vetor') -> 'Vetor':
        if not hasattr(other, "_data"):
            raise TypeError("cross apenas entre Vetores")
        return Vetor(*np.cross(self._data, other._data))

    def to_array(self):
        return self._data.copy()

    def __iter__(self):
        return iter(self._data)

    # ============================================================
    # OPERAÇÕES (mantidas, só tornadas mais seguras)
    # ============================================================

    def __add__(self, other):
        if hasattr(other, "_data"):
            return Vetor(*(self._data + other._data))
        raise TypeError("Vetor só soma com Vetor ou tipo compatível")

    def __sub__(self, other):
        if hasattr(other, "_data"):
            return Vetor(*(self._data - other._data))
        raise TypeError("Vetor só subtrai com Vetor ou tipo compatível")

    
    def __mul__(self, other):

        # Vetor * escalar
        if isinstance(other, (int, float, np.number)):
            return Vetor(*(self._data * other))

        # Vetor * Vetor (componente a componente)
        if isinstance(other, Vetor):
            return Vetor(*(self._data * other._data))

        raise TypeError(
            "Multiplicação suportada apenas para escalar ou Vetor."
        )



    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __truediv__(self, scalar):
        if isinstance(scalar, (int, float, np.number)):
            if scalar == 0:
                raise ZeroDivisionError("Divisão por zero")
            return Vetor(*(self._data / scalar))
        raise TypeError("Vetor só divide por escalar")

    def __neg__(self):
        return Vetor(*(-self._data))

    # ============================================================
    # UTILITÁRIOS
    # ============================================================

    def clamp(self, minv=0.0, maxv=1.0):
        return Vetor(
            np.clip(self.x, minv, maxv),
            np.clip(self.y, minv, maxv),
            np.clip(self.z, minv, maxv)
        )

    def almost_equal(self, other: 'Vetor', eps=1e-6):
        if not hasattr(other, "_data"):
            return False
        return np.allclose(self._data, other._data, atol=eps)

    def __repr__(self):
        return f"Vetor({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"