import numpy as np
from src.Ponto import Ponto
from src.Vetor import Vetor


def intersect_sphere(origem: Ponto, direcao: Vetor, centro: Ponto, raio: float) -> float:
    """
    Calcula a interseção entre um raio e uma esfera.

    O raio é definido por origem e direção, enquanto a esfera é definida
    por centro e raio. A interseção é obtida resolvendo uma equação
    quadrática derivada da substituição da equação paramétrica do raio
    na equação implícita da esfera.

    O discriminante determina a existência de interseção:
    - Se negativo, não há interseção
    - Se não negativo, existem até duas soluções (entrada e saída)

    Retorna o menor valor de t positivo maior que um pequeno epsilon (0.001),
    garantindo que a interseção esteja à frente da câmera e evitando problemas
    numéricos como self-intersection (shadow acne).

    Caso não haja interseção válida, retorna infinito.
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

    O plano é definido por um ponto p0 e um vetor normal. A interseção
    é obtida substituindo a equação paramétrica do raio na equação do plano.

    O denominador (produto escalar entre direção do raio e normal do plano)
    indica a relação angular:
    - Se próximo de zero, o raio é paralelo ao plano e não há interseção útil
    - Caso contrário, calcula-se t como a distância ao ponto de interseção

    Retorna o valor de t se ele for positivo e maior que um pequeno epsilon (0.001),
    garantindo que a interseção esteja à frente da câmera.

    Caso não haja interseção válida, retorna infinito.
    """
    denom = direcao.dot(normal)
    
    if abs(denom) > 1e-6:
        p0_origem = p0 - origem
        t = p0_origem.dot(normal) / denom
        
        if t > 0.001:
            return t
            
    return float('inf')