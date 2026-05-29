import numpy as np
from src.Vetor import Vetor


class Ponto:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self._data = np.array([x, y, z], dtype=np.float64)

    # ============================================================
    # ACESSO
    # ============================================================

    @property
    def x(self): return self._data[0]

    @property
    def y(self): return self._data[1]

    @property
    def z(self): return self._data[2]

    # ============================================================
    # OPERAÇÕES PRINCIPAIS
    # ============================================================

    def __add__(self, other):
        """
        Ponto + Vetor = Ponto
        """
        if isinstance(other, Vetor):
            return Ponto(*(self._data + other._data))

        if hasattr(other, "_data"):
            return Ponto(*(self._data + other._data))

        raise TypeError("Ponto só pode ser somado a um Vetor.")

    def __sub__(self, other):
        """
        Ponto - Ponto = Vetor
        Ponto - Vetor = Ponto
        """
        if isinstance(other, Ponto):
            return Vetor(*(self._data - other._data))

        if isinstance(other, Vetor):
            return Ponto(*(self._data - other._data))

        if hasattr(other, "_data"):
            return Vetor(*(self._data - other._data))

        raise TypeError("Subtração inválida para Ponto.")

    # ============================================================
    # UTILITÁRIOS
    # ============================================================

    def to_array(self):
        return self._data.copy()

    def __iter__(self):
        return iter(self._data)

    def almost_equal(self, other: 'Ponto', eps=1e-6):
        if not hasattr(other, "_data"):
            return False
        return np.allclose(self._data, other._data, atol=eps)

    # ============================================================
    # REPRESENTAÇÃO
    # ============================================================

    def __repr__(self):
        return f"Ponto({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"