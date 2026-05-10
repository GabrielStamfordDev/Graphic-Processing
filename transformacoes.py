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


def aplicar_matriz_vetor(matriz: np.ndarray, vetor: Vetor) -> Vetor:
    """
    Aplica uma matriz 4x4 a um Vetor (x, y, z, 0).
    Como a 4ª componente é 0, a translação é automaticamente ignorada,
    mas a rotação e a escala são aplicadas.
    """
    # Note o 0.0 no final, que caracteriza matematicamente um vetor direcional
    v_homogeneo = np.array([vetor.x, vetor.y, vetor.z, 0.0], dtype=np.float64)
    
    # Multiplicação da matriz pelo vetor
    v_transformado = matriz @ v_homogeneo
    
    # Retorna como nossa classe Vetor
    return Vetor(v_transformado[0], v_transformado[1], v_transformado[2])
