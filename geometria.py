import numpy as np
from src.Ponto import Ponto
from src.Vetor import Vetor


def intersect_sphere(origem: Ponto, direcao: Vetor, centro: Ponto, raio: float) -> float:
    """
    Calcula a interseção entre um raio e uma esfera.
    """
    v = origem - centro
    
    v_dot_d = v.dot(direcao)
    v_dot_v = v.dot(v)
    r2 = raio * raio
    
    discriminant = (v_dot_d ** 2) - (v_dot_v - r2)
    
    if discriminant < 0:
        return float('inf')
        
    sqrt_disc = np.sqrt(discriminant)
    
    t1 = -v_dot_d - sqrt_disc
    t2 = -v_dot_d + sqrt_disc
    
    if t1 > 0.001: return t1
    if t2 > 0.001: return t2
    
    return float('inf')


def intersect_plane(origem: Ponto, direcao: Vetor, p0: Ponto, normal: Vetor) -> float:
    """
    Calcula a interseção entre um raio e um plano.
    """
    denom = direcao.dot(normal)
    
    if abs(denom) > 1e-6:
        p0_origem = p0 - origem
        t = p0_origem.dot(normal) / denom
        
        if t > 0.001:
            return t
            
    return float('inf')


def intersect_triangles_numpy(origem: Ponto, direcao: Vetor,
                               v0: np.ndarray, v1: np.ndarray, v2: np.ndarray) -> float:
    """
    Möller–Trumbore vetorizado — testa N triângulos de uma vez com numpy.
    v0, v1, v2 são arrays (N, 3) pré-computados no pré-processamento.
    Retorna o menor t válido ou inf se não houver interseção.
    """
    EPSILON = 1e-8
    orig = np.array([origem.x, origem.y, origem.z])
    dire = np.array([direcao.x, direcao.y, direcao.z])

    aresta1 = v1 - v0          # (N, 3)
    aresta2 = v2 - v0          # (N, 3)

    h = np.cross(dire, aresta2)                         # (N, 3)
    a = np.einsum('ij,ij->i', aresta1, h)               # (N,)

    mask = np.abs(a) > EPSILON
    f = np.where(mask, 1.0 / np.where(mask, a, 1), 0)

    s = orig - v0                                       # (N, 3)
    u = f * np.einsum('ij,ij->i', s, h)                # (N,)
    mask &= (u >= 0.0) & (u <= 1.0)

    q = np.cross(s, aresta1)                            # (N, 3)
    v = f * (q @ dire)                                  # (N,)  ← corrigido
    mask &= (v >= 0.0) & ((u + v) <= 1.0)

    t = f * (aresta2 @ dire)                            # (N,)  ← corrigido
    mask &= (t > 0.001)

    if not np.any(mask):
        return float('inf')

    return float(np.min(t[mask]))