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


def intersect_triangle(origem: Ponto, direcao: Vetor, v0: Ponto, v1: Ponto, v2: Ponto) -> float:
    """
    Calcula a interseção entre um raio e um triângulo usando o Teste de Áreas.
    """
    # 1. Encontra o vetor normal (NÃO unitário) e a Área do triângulo original
    aresta1 = v1 - v0
    aresta2 = v2 - v0
    vetor_cruzado_abc = aresta1.cross(aresta2)
    
    area_ABC = vetor_cruzado_abc.magnitude() / 2.0
    
    # 2. Interseção do Raio com o Plano (usando a normal não-unitária)
    denom = direcao.dot(vetor_cruzado_abc)
    if abs(denom) < 1e-8:
        return float('inf') # Raio paralelo ao triângulo
        
    p0_origem = v0 - origem
    t = p0_origem.dot(vetor_cruzado_abc) / denom
    
    if t < 0.001:
        return float('inf') # Triângulo está atrás da câmera
        
    # 3. Descobre o Ponto P exato da interseção no plano
    p = origem + (direcao * t)
    
    # 4. Calcula a área dos 3 sub-triângulos usando sua fórmula (vetores saindo dos vértices originais)
    area_PAB = (v1 - v0).cross(p - v0).magnitude() / 2.0
    area_PBC = (v2 - v1).cross(p - v1).magnitude() / 2.0
    area_PCA = (v0 - v2).cross(p - v2).magnitude() / 2.0
    
    # 5. Compara se a soma das áreas é igual à área original
    soma_areas = area_PAB + area_PBC + area_PCA
    
    if abs(soma_areas - area_ABC) > 1e-5:
        return float('inf') # Ponto P está FORA do triângulo
        
    # 6. Coeficientes baricêntricos
    alpha = area_PBC / area_ABC
    beta = area_PCA / area_ABC
    gamma = area_PAB / area_ABC
    
    return t
