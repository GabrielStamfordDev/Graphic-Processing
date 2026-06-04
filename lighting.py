# src/iluminacao.py
import numpy as np
from src.Ponto import Ponto
from src.Vetor import Vetor


# ============================================================
# CONSTANTES
# ============================================================
EPSILON = 1e-4


# ============================================================
# SOMBRAS
# ============================================================
def checar_sombra(
    ponto_deslocado: Ponto,
    L_normalizado: Vetor,
    distancia_luz: float,
    objetos: list,
    intersect_func
) -> bool:
    """
    Dispara um shadow ray a partir de um ponto ligeiramente deslocado.
    
    Retorna True se existir qualquer objeto entre:
        0 < t < distancia_luz
    """

    for obj_sombra in objetos:
        t_sombra, _ = intersect_func(
            obj_sombra,
            ponto_deslocado,
            L_normalizado
        )

        if EPSILON < t_sombra < distancia_luz:
            return True

    return False


# ============================================================
# PHONG
# ============================================================
def calcular_cor_phong(
    P: Ponto,
    N: Vetor,
    ray_dir: Vetor,
    hit_obj,
    scene_data,
    intersect_func
) -> tuple[float, float, float]:

    Ia = scene_data.global_light.color
    mat = hit_obj.material

    # =========================================================
    # NORMALIZAÇÃO GLOBAL
    # =========================================================

    N = N.normalize()
    #ray_dir = ray_dir.normalize()

    # Corrige normal para ficar contra o raio incidente
    '''if ray_dir.dot(N) > 0:
        N = N * (-1.0)'''

    # Vetor de visão
    V = (scene_data.camera.lookfrom - P).normalize()

    # =========================================================
    # COMPONENTE AMBIENTE
    # =========================================================

    cor_r = mat.ka.r * Ia.r
    cor_g = mat.ka.g * Ia.g
    cor_b = mat.ka.b * Ia.b

    # =========================================================
    # LUZES
    # =========================================================

    for luz in scene_data.light_list:

        vetor_luz = luz.pos - P

        distancia_luz = vetor_luz.magnitude()

        # Vetor unitário direção da luz
        L = vetor_luz.normalize()

        # =====================================================
        # SHADOW RAY
        # =====================================================

        P_sombra = P + (N * EPSILON)

        if checar_sombra(
            P_sombra,
            L,
            distancia_luz,
            scene_data.objects,
            intersect_func
        ):
            continue

        # =====================================================
        # DIFUSA
        # =====================================================

        L_dot_N = N.dot(L)

        if L_dot_N <= 0:
            continue

        cor_r += (
            mat.color.r *
            L_dot_N *
            luz.color.r
        )

        cor_g += (
            mat.color.g *
            L_dot_N *
            luz.color.g
        )

        cor_b += (
            mat.color.b *
            L_dot_N *
            luz.color.b
        )

        # =====================================================
        # ESPECULAR
        # =====================================================

        # R = 2(N·L)N - L

        R = ((N * (2.0 * L_dot_N)) - L).normalize()

        R_dot_V = R.dot(V)

        if R_dot_V > 0:

            spec = R_dot_V ** mat.ns

            cor_r += (
                mat.ks.r *
                spec *
                luz.color.r
            )

            cor_g += (
                mat.ks.g *
                spec *
                luz.color.g
            )

            cor_b += (
                mat.ks.b *
                spec *
                luz.color.b
            )

    # =========================================================
    # CLAMP
    # =========================================================

    cor_r = min(1.0, max(0.0, cor_r))
    cor_g = min(1.0, max(0.0, cor_g))
    cor_b = min(1.0, max(0.0, cor_b))

    return cor_r, cor_g, cor_b
