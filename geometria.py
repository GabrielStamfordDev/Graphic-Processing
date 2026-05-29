import numpy as np
from src.Ponto import Ponto
from src.Vetor import Vetor

def intersect_sphere(origem: Ponto, direcao: Vetor, centro: Ponto, raio: float) -> tuple[float, Vetor]:
    """
    Retorna a distância t e o Vetor Normal no ponto de interseção.
    """
    v = origem - centro
    v_dot_d = v.dot(direcao)
    v_dot_v = v.dot(v)
    r2 = raio * raio
    
    discriminant = (v_dot_d ** 2) - (v_dot_v - r2)
    
    if discriminant < 0:
        return float('inf'), None
        
    sqrt_disc = np.sqrt(discriminant)
    
    t1 = -v_dot_d - sqrt_disc
    t2 = -v_dot_d + sqrt_disc
    
    t = float('inf')
    if t1 > 0.001: 
        t = t1
    elif t2 > 0.001: 
        t = t2
        
    if t != float('inf'):
        # Calcula o ponto exato no espaço
        P = origem + (direcao * t)
        # A normal de uma esfera é o vetor que sai do centro e vai até P
        normal = (P - centro).normalize()
        return t, normal
        
    return float('inf'), None


def intersect_plane(origem: Ponto, direcao: Vetor, p0: Ponto, normal: Vetor) -> tuple[float, Vetor]:
    """
    Retorna a distância t e a Normal do plano.
    """
    denom = direcao.dot(normal)
    
    if abs(denom) > 1e-6:
        p0_origem = p0 - origem
        t = p0_origem.dot(normal) / denom
        
        if t > 0.001:
            # Se o raio atingir o plano por "trás", invertemos a normal para a luz funcionar
            n_final = normal if denom < 0 else normal * (-1)
            return t, n_final
            
    return float('inf'), None


def intersect_triangles_numpy(origem: Ponto, direcao: Vetor,
                           v0: np.ndarray, v1: np.ndarray, v2: np.ndarray) -> tuple[float, int, float, float]:
    """
    Retorna (t, indice_do_triangulo, u, v)
    """
    EPSILON = 1e-8
    orig = np.array([origem.x, origem.y, origem.z])
    dire = np.array([direcao.x, direcao.y, direcao.z])

    aresta1 = v1 - v0
    aresta2 = v2 - v0

    h = np.cross(dire, aresta2)
    a = np.einsum('ij,ij->i', aresta1, h)

    mask = np.abs(a) > EPSILON
    f = np.where(mask, 1.0 / np.where(mask, a, 1), 0)

    s = orig - v0
    u = f * np.einsum('ij,ij->i', s, h)
    mask &= (u >= 0.0) & (u <= 1.0)

    q = np.cross(s, aresta1)
    v = f * (q @ dire)
    mask &= (v >= 0.0) & ((u + v) <= 1.0)

    t = f * np.einsum('ij,ij->i', aresta2, q)
    mask &= (t > 0.001)

    if not np.any(mask):
        return float('inf'), -1, 0.0, 0.0

    # 1. Filtramos apenas as respostas válidas
    valid_t = t[mask]
    
    # 2. Achamos o índice relativo do menor T dentro dos válidos
    min_idx_in_valid = np.argmin(valid_t)
    
    # 3. Descobrimos qual era o índice original desse triângulo na lista inteira (N)
    orig_indices = np.arange(len(t))[mask]
    hit_idx = orig_indices[min_idx_in_valid]
    
    # 4. Pegamos os valores vencedores
    hit_t = valid_t[min_idx_in_valid]
    hit_u = u[mask][min_idx_in_valid]
    hit_v = v[mask][min_idx_in_valid]

    return float(hit_t), int(hit_idx), float(hit_u), float(hit_v)