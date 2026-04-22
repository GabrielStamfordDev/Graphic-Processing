import numpy as np
from src.Ponto import Ponto
from src.Vetor import Vetor

def matriz_translacao(dx: float, dy: float, dz: float) -> np.ndarray:
    return np.array([
        [1.0, 0.0, 0.0,  dx],
        [0.0, 1.0, 0.0,  dy],
        [0.0, 0.0, 1.0,  dz],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)

def matriz_escala(sx: float, sy: float, sz: float) -> np.ndarray:
    return np.array([
        [ sx, 0.0, 0.0, 0.0],
        [0.0,  sy, 0.0, 0.0],
        [0.0, 0.0,  sz, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=np.float64)

def matriz_rotacao_x(angulo_graus: float) -> np.ndarray:
    rad = np.radians(angulo_graus)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [1.0, 0.0,  0.0, 0.0],
        [0.0,   c,   -s, 0.0],
        [0.0,   s,    c, 0.0],
        [0.0, 0.0,  0.0, 1.0]
    ], dtype=np.float64)

def matriz_rotacao_y(angulo_graus: float) -> np.ndarray:
    rad = np.radians(angulo_graus)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [  c, 0.0,    s, 0.0],
        [0.0, 1.0,  0.0, 0.0],
        [ -s, 0.0,    c, 0.0],
        [0.0, 0.0,  0.0, 1.0]
    ], dtype=np.float64)

def matriz_rotacao_z(angulo_graus: float) -> np.ndarray:
    rad = np.radians(angulo_graus)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [  c,  -s,  0.0, 0.0],
        [  s,   c,  0.0, 0.0],
        [0.0, 0.0,  1.0, 0.0],
        [0.0, 0.0,  0.0, 1.0]
    ], dtype=np.float64)

def aplicar_matriz_ponto(matriz: np.ndarray, ponto: Ponto) -> Ponto:
    """Aplica uma matriz 4x4 a um Ponto (x, y, z, 1)."""
    p_homogeneo = np.array([ponto.x, ponto.y, ponto.z, 1.0], dtype=np.float64)
    p_transformado = matriz @ p_homogeneo
    return Ponto(p_transformado[0], p_transformado[1], p_transformado[2])
