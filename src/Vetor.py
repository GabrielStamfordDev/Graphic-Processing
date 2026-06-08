import numpy as np

class Vetor:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        # Armazena os dados internamente como array do numpy
        self._data = np.array([x, y, z], dtype=np.float64)

    # Propriedades para garantir compatibilidade com o parser
    @property
    def x(self): return self._data[0]
    
    @x.setter
    def x(self, value): self._data[0] = value

    @property
    def y(self): return self._data[1]
    
    @y.setter
    def y(self, value): self._data[1] = value

    @property
    def z(self): return self._data[2]
    
    @z.setter
    def z(self, value): self._data[2] = value

    def magnitude(self) -> float:
        """Retorna o comprimento do vetor."""
        return np.linalg.norm(self._data)

    def normalize(self) -> 'Vetor':
        """Retorna um novo Vetor com mesma direção e tamanho 1."""
        mag = self.magnitude()
        if mag == 0:
            return Vetor(0, 0, 0)
        res = self._data / mag
        return Vetor(*res)

    def dot(self, other: 'Vetor') -> float:
        """Produto Escalar."""
        return float(np.dot(self._data, other._data))

    def cross(self, other: 'Vetor') -> 'Vetor':
        """Produto Vetorial (essencial para definir a base da câmera)."""
        res = np.cross(self._data, other._data)
        return Vetor(*res)

    def __add__(self, other):
        if isinstance(other, Vetor):
            return Vetor(*(self._data + other._data))
        raise TypeError("Vetores só somam com Vetores.")

    def __sub__(self, other):
        if isinstance(other, Vetor):
            return Vetor(*(self._data - other._data))
        raise TypeError("Vetores só subtraem com Vetores.")

    def __mul__(self, scalar: float) -> 'Vetor':
        """Multiplicação por escalar (Vetor * numero)."""
        return Vetor(*(self._data * scalar))

    def __rmul__(self, scalar: float) -> 'Vetor':
        """Multiplicação reversa (numero * Vetor)."""
        return self.__mul__(scalar)

    def __neg__(self):
        """Inverte o vetor (-Vetor)."""
        return Vetor(*(-self._data))

    def __repr__(self):
        return f"Vetor({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"