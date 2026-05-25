import sys
import numpy as np
from pathlib import Path
from src.Ponto import Ponto
from src.Vetor import Vetor
from utils.Scene.sceneParser import SceneJsonLoader
from utils.MeshReader.ObjReader import ObjReader

from camera import Camera

from geometria import (
    intersect_sphere,
    intersect_plane,
    intersect_triangles_numpy
)

from transformacoes import (
    matriz_translacao,
    matriz_escala,
    matriz_rotacao_x,
    matriz_rotacao_y,
    matriz_rotacao_z,
    aplicar_matriz_ponto,
    aplicar_matriz_vetor
)


# ============================================================
# INTERSEÇÃO
# ============================================================

def intersect_object(obj, ray_origin, ray_dir):
    """
    Retorna a dupla (t, Normal). 
    Se não bater em nada, retorna (inf, None).
    """
    if obj.obj_type == "sphere":
        return intersect_sphere(ray_origin, ray_dir, obj.relative_pos, obj.get_num("radius"))

    elif obj.obj_type == "plane":
        normal = obj.get_vetor("normal").normalize()
        return intersect_plane(ray_origin, ray_dir, obj.relative_pos, normal)

    elif obj.obj_type == "mesh":
        # Recebe os novos retornos da função vetorizada do geometria.py
        t, idx, u, v = intersect_triangles_numpy(
            ray_origin, ray_dir, obj.np_v0, obj.np_v1, obj.np_v2
        )
        
        if t != float("inf"):
            # 1. Busca as normais originais daquele triângulo específico
            n0 = Vetor(*obj.np_n0[idx])
            n1 = Vetor(*obj.np_n1[idx])
            n2 = Vetor(*obj.np_n2[idx])
            
            # 2. Faz a Interpolação Baricêntrica para Phong Shading!
            peso_n0 = 1.0 - u - v
            normal_interp = (n0 * peso_n0) + (n1 * u) + (n2 * v)
            
            # 3. Retorna a distância e a normal recém normalizada
            return t, normal_interp.normalize()

    return float("inf"), None


def aplicar_matriz_normal(M, n: Vetor) -> Vetor:
    """
    Aplica a INVERSA TRANSPOSTA da matriz de transformação a um vetor normal,
    garantindo que ele continue perpendicular à superfície mesmo após escalas não-uniformes.
    """
    M_inv = np.linalg.inv(M)
    M_inv_T = M_inv.T
    
    v_homogeneo = np.array([n.x, n.y, n.z, 0.0], dtype=np.float64)
    res = M_inv_T @ v_homogeneo
    
    return Vetor(res[0], res[1], res[2]).normalize()


# ============================================================
# MATRIZ DE ROTAÇÃO
# ============================================================

def build_rotation_matrix(rx, ry, rz):

    Mx = matriz_rotacao_x(rx)
    My = matriz_rotacao_y(ry)
    Mz = matriz_rotacao_z(rz)

    return Mz @ My @ Mx


# ============================================================
# ESFERA
# ============================================================

def processar_esfera(obj, apply_transform):
    """
    ESFERA

    - rotação: ignorada
    - escala: multiplica raio
    - translação: aplicada no centro
    """

    if not apply_transform:
        return

    centro = obj.relative_pos

    raio = obj.get_num("radius")

    for t in obj.transforms:

        if t.t_type == "rotation":
            pass

        elif t.t_type == "scaling":

            sx = t.data.x
            sy = t.data.y
            sz = t.data.z

            if not (
                np.isclose(sx, sy)
                and np.isclose(sx, sz)
            ):
                raise ValueError(
                    "Esfera aceita apenas escala uniforme."
                )

            raio *= sx

        elif t.t_type == "translation":

            M = matriz_translacao(
                t.data.x,
                t.data.y,
                t.data.z
            )

            centro = aplicar_matriz_ponto(
                M,
                centro
            )

    obj.relative_pos = centro
    obj.numeric_data["radius"] = raio


# ============================================================
# PLANO
# ============================================================

def processar_plano(obj, apply_transform):
    """
    PLANO

    - rotação: aplicada na normal
    - escala: ignorada
    - translação: aplicada no relative_pos
    """

    if not apply_transform:
        return

    ponto = obj.relative_pos

    normal = obj.get_vetor(
        "normal"
    ).normalize()

    for t in obj.transforms:

        if t.t_type == "rotation":

            M_rot = build_rotation_matrix(
                t.data.x,
                t.data.y,
                t.data.z
            )

            normal = aplicar_matriz_vetor(
                M_rot,
                normal
            ).normalize()

        elif t.t_type == "scaling":
            pass

        elif t.t_type == "translation":

            M = matriz_translacao(
                t.data.x,
                t.data.y,
                t.data.z
            )

            ponto = aplicar_matriz_ponto(
                M,
                ponto
            )

    obj.relative_pos = ponto

    obj.vetor_point_data["normal"] = normal


# ============================================================
# MALHA
# ============================================================

def processar_malha(
    obj,
    apply_transform,
    loaded_meshes
):
    """
    MALHA

    REGRAS:

    --------------------------------------------------------
    relativePos do JSON:
        IGNORADO

    A malha sempre nasce em:
        (0,0,0)

    --------------------------------------------------------
    translation:
        afeta apenas relative_pos

    --------------------------------------------------------
    rotation/scaling:
        afetam vértices

        se relative_pos != origem:
            usa relative_pos como pivô

    --------------------------------------------------------
    final:
        aplica relative_pos em todos os vértices
    """

    obj_path = obj.get_property("path")

    # ========================================================
    # OBJ inexistente
    # ========================================================

    if not Path(obj_path).exists():

        print(
            f"[AVISO] OBJ não encontrado: {obj_path}",
            file=sys.stderr
        )

        return False

    # ========================================================
    # CACHE
    # ========================================================

    if obj_path not in loaded_meshes:

        print(
            f"Carregando: {obj_path}",
            file=sys.stderr
        )

        loaded_meshes[obj_path] = ObjReader(
            obj_path
        )

    mesh_reader = loaded_meshes[obj_path]

    faces = mesh_reader.get_face_points()

    # ========================================================
    # OBJ inválido
    # ========================================================

    if not faces:

        print(
            f"[AVISO] OBJ vazio: {obj_path}",
            file=sys.stderr
        )

        return False

    # ========================================================
    # MATRIZ FINAL
    # ========================================================

    M_total = np.eye(4, dtype=np.float64)

    # ========================================================
    # IMPORTANTE:
    #
    # A malha sempre começa na origem.
    # ========================================================

    relative_pos = Ponto(
        0.0,
        0.0,
        0.0
    )

    # ========================================================
    # TRANSFORMAÇÕES
    # ========================================================

    if apply_transform:

        for t in obj.transforms:

            # ------------------------------------------------
            # TRANSLAÇÃO
            # ------------------------------------------------

            if t.t_type == "translation":

                M_translate = matriz_translacao(
                    t.data.x,
                    t.data.y,
                    t.data.z
                )

                relative_pos = aplicar_matriz_ponto(
                    M_translate,
                    relative_pos
                )

            # ------------------------------------------------
            # ROTAÇÃO
            # ------------------------------------------------

            elif t.t_type == "rotation":

                M_rot = build_rotation_matrix(
                    t.data.x,
                    t.data.y,
                    t.data.z
                )

                # pivô na origem
                if (
                    np.isclose(relative_pos.x, 0.0)
                    and np.isclose(relative_pos.y, 0.0)
                    and np.isclose(relative_pos.z, 0.0)
                ):

                    M_atual = M_rot

                # pivô no relative_pos
                else:

                    T_neg = matriz_translacao(
                        -relative_pos.x,
                        -relative_pos.y,
                        -relative_pos.z
                    )

                    T_pos = matriz_translacao(
                        relative_pos.x,
                        relative_pos.y,
                        relative_pos.z
                    )

                    M_atual = (
                        T_pos
                        @ M_rot
                        @ T_neg
                    )

                M_total = M_atual @ M_total

            # ------------------------------------------------
            # ESCALA
            # ------------------------------------------------

            elif t.t_type == "scaling":

                M_scale = matriz_escala(
                    t.data.x,
                    t.data.y,
                    t.data.z
                )

                # pivô na origem
                if (
                    np.isclose(relative_pos.x, 0.0)
                    and np.isclose(relative_pos.y, 0.0)
                    and np.isclose(relative_pos.z, 0.0)
                ):

                    M_atual = M_scale

                # pivô no relative_pos
                else:

                    T_neg = matriz_translacao(
                        -relative_pos.x,
                        -relative_pos.y,
                        -relative_pos.z
                    )

                    T_pos = matriz_translacao(
                        relative_pos.x,
                        relative_pos.y,
                        relative_pos.z
                    )

                    M_atual = (
                        T_pos
                        @ M_scale
                        @ T_neg
                    )

                M_total = M_atual @ M_total

    # ========================================================
    # POSIÇÃO INICIAL DO JSON
    # ========================================================

    M_inicial = matriz_translacao(
        obj.relative_pos.x,
        obj.relative_pos.y,
        obj.relative_pos.z
    )

    # ========================================================
    # POSIÇÃO FINAL NO MUNDO
    # ========================================================

    M_relative = matriz_translacao(
        relative_pos.x,
        relative_pos.y,
        relative_pos.z
    )

    M_total = M_relative @ M_inicial @ M_total

    # ========================================================
    # APLICAÇÃO NOS VÉRTICES E NORMAIS
    # ========================================================

    v0_list, v1_list, v2_list = [], [], []
    n0_list, n1_list, n2_list = [], [], []

    vertices_raw = mesh_reader.get_vertices()
    normals_raw = mesh_reader.get_normals()
    faces_data = mesh_reader.get_faces()

# ========================================================
    # INTEGRAÇÃO DO MATERIAL (.MTL) vs (.JSON)
    # ========================================================
    
    # REGRA: O .json é o chefe. Só puxamos o .mtl se o .json 
    # NÃO tiver especificado um material (nome vazio).
    if obj.material.name == "" and len(faces_data) > 0:
        mat_mtl = faces_data[0].material
        
        # CHECAGEM DE SEGURANÇA: Só sobrescrevemos se o .mtl realmente
        # tiver entregue uma cor (Kd) diferente de (0,0,0). Isso evita que
        # um .mtl ausente ou quebrado destrua a renderização.
        if mat_mtl.kd.x != 0.0 or mat_mtl.kd.y != 0.0 or mat_mtl.kd.z != 0.0:
            
            # Transferindo as cores originais do arquivo 3D
            obj.material.color.r = mat_mtl.kd.x
            obj.material.color.g = mat_mtl.kd.y
            obj.material.color.b = mat_mtl.kd.z
            
            obj.material.ka.r = mat_mtl.ka.x
            obj.material.ka.g = mat_mtl.ka.y
            obj.material.ka.b = mat_mtl.ka.z
            
            obj.material.ks.r = mat_mtl.ks.x
            obj.material.ks.g = mat_mtl.ks.y
            obj.material.ks.b = mat_mtl.ks.z
            
            obj.material.ns = mat_mtl.ns

    for face in faces_data:
        # Pega os vértices originais
        v0_raw = vertices_raw[face.vertice_indice[0]]
        v1_raw = vertices_raw[face.vertice_indice[1]]
        v2_raw = vertices_raw[face.vertice_indice[2]]

        # Pega as normais originais (com fallback caso o OBJ não tenha normais)
        if normals_raw:
            n0_raw = normals_raw[face.normal_indice[0]]
            n1_raw = normals_raw[face.normal_indice[1]]
            n2_raw = normals_raw[face.normal_indice[2]]
        else:
            # Fallback: Cria uma normal de face padrão se o OBJ for muito simples
            aresta1 = v1_raw - v0_raw
            aresta2 = v2_raw - v0_raw
            n_face = aresta1.cross(aresta2).normalize()
            n0_raw = n1_raw = n2_raw = n_face

        if apply_transform:
            # Transforma Vértices
            v0t = aplicar_matriz_ponto(M_total, v0_raw)
            v1t = aplicar_matriz_ponto(M_total, v1_raw)
            v2t = aplicar_matriz_ponto(M_total, v2_raw)
            # Transforma Normais (Inversa Transposta)
            n0t = aplicar_matriz_normal(M_total, n0_raw)
            n1t = aplicar_matriz_normal(M_total, n1_raw)
            n2t = aplicar_matriz_normal(M_total, n2_raw)
        else:
            v0t, v1t, v2t = v0_raw, v1_raw, v2_raw
            n0t, n1t, n2t = n0_raw, n1_raw, n2_raw

        # Guarda Vértices
        v0_list.append([v0t.x, v0t.y, v0t.z])
        v1_list.append([v1t.x, v1t.y, v1t.z])
        v2_list.append([v2t.x, v2t.y, v2t.z])
        
        # Guarda Normais
        n0_list.append([n0t.x, n0t.y, n0t.z])
        n1_list.append([n1t.x, n1t.y, n1t.z])
        n2_list.append([n2t.x, n2t.y, n2t.z])

    # Associa os arrays ao objeto
    obj.np_v0 = np.array(v0_list, dtype=np.float64)
    obj.np_v1 = np.array(v1_list, dtype=np.float64)
    obj.np_v2 = np.array(v2_list, dtype=np.float64)

    obj.np_n0 = np.array(n0_list, dtype=np.float64)
    obj.np_n1 = np.array(n1_list, dtype=np.float64)
    obj.np_n2 = np.array(n2_list, dtype=np.float64)

    print(f"  → {len(faces_data)} triângulos carregados e transformados.", file=sys.stderr)

    return True

# ============================================================
# MAIN
# ============================================================

def main():

    if len(sys.argv) < 2:

        print(
            "Use: python main.py <cena.json> [--no-transform]",
            file=sys.stderr
        )

        sys.exit(1)

    scene_file = sys.argv[1]

    apply_transform = True

    if (
        len(sys.argv) > 2
        and sys.argv[2] == "--no-transform"
    ):
        apply_transform = False

    # ========================================================
    # LOAD DA CENA
    # ========================================================

    scene_data = SceneJsonLoader.load_file(
        scene_file
    )

    cam = Camera(scene_data.camera)

    # ========================================================
    # PRÉ-PROCESSAMENTO
    # ========================================================

    loaded_meshes = {}

    valid_objects = []

    for obj in scene_data.objects:

        if obj.obj_type == "sphere":

            processar_esfera(
                obj,
                apply_transform
            )

        elif obj.obj_type == "plane":

            processar_plano(
                obj,
                apply_transform
            )

        elif obj.obj_type == "mesh":

            ok = processar_malha(
                obj,
                apply_transform,
                loaded_meshes
            )

            if not ok:
                continue

        valid_objects.append(obj)

    scene_data.objects = valid_objects

# ========================================================
    # RENDER (PHONG SHADING CORRIGIDO)
    # ========================================================

    print(f"P3\n{cam.hres} {cam.vres}\n255")

    # Cor da luz ambiente global
    Ia = scene_data.global_light.color

    for j in range(cam.vres):
        print(f"Linha {j}/{cam.vres}", file=sys.stderr, end='\r')

        for i in range(cam.hres):
            ray_dir = cam.get_ray_direction(i, j)

            closest_t = float("inf")
            hit_obj = None
            hit_normal = None

            # 1. ENCONTRA O OBJETO MAIS PRÓXIMO
            for obj in scene_data.objects:
                t, normal = intersect_object(obj, cam.C, ray_dir)
                
                if t < closest_t:
                    closest_t = t
                    hit_obj = obj
                    hit_normal = normal

            # Se não bateu em nada, fundo preto
            if hit_obj is None:
                print("0 0 0")
                continue

            # 2. PREPARATIVOS PARA A EQUAÇÃO DE PHONG
            P = cam.C + (ray_dir * closest_t)
            
            N = hit_normal
            # Garante que a normal aponta contra o raio
            if ray_dir.dot(N) > 0:
                N = N * (-1.0)

            V = (cam.C - P).normalize()

            mat = hit_obj.material
            
            # 3. COMPONENTE AMBIENTE (I = ka * Ia)
            cor_r = mat.ka.r * Ia.r
            cor_g = mat.ka.g * Ia.g
            cor_b = mat.ka.b * Ia.b

            # 4. SOMA DAS LUZES PONTUAIS
            for luz in scene_data.light_list:
                vetor_luz = luz.pos - P
                distancia_luz = vetor_luz.magnitude()
                L = vetor_luz.normalize()

                # --- RAY TRACING DE SOMBRAS ---
                # Epsilon (1e-4) para evitar que o objeto faça sombra nele mesmo
                P_sombra = P + (N * 1e-4)
                
                em_sombra = False
                for obj_sombra in scene_data.objects:
                    # Não precisamos da normal para checar sombra, ignoramos com "_"
                    t_sombra, _ = intersect_object(obj_sombra, P_sombra, L)
                    
                    if 0.001 < t_sombra < distancia_luz:
                        em_sombra = True
                        break
                
                if em_sombra:
                    continue # Pula Difusa e Especular dessa luz

                # --- LUZ DIFUSA ---
                L_dot_N = L.dot(N)
                if L_dot_N > 0:
                    cor_r += mat.color.r * L_dot_N * luz.color.r
                    cor_g += mat.color.g * L_dot_N * luz.color.g
                    cor_b += mat.color.b * L_dot_N * luz.color.b

                    # --- LUZ ESPECULAR ---
                    R = (N * (2.0 * L_dot_N)) - L
                    R_dot_V = R.dot(V)
                    
                    if R_dot_V > 0:
                        spec = R_dot_V ** mat.ns
                        cor_r += mat.ks.r * spec * luz.color.r
                        cor_g += mat.ks.g * spec * luz.color.g
                        cor_b += mat.ks.b * spec * luz.color.b

            # 5. O SEGREDO QUE FALTAVA (Conversão e Clamping corretos!)
            # Agora multiplicamos por 255.999 ANTES de converter para Inteiro.
            r_final = min(255, max(0, int(255.999 * cor_r)))
            g_final = min(255, max(0, int(255.999 * cor_g)))
            b_final = min(255, max(0, int(255.999 * cor_b)))

            print(f"{r_final} {g_final} {b_final}")

    print("\nRenderização concluída!", file=sys.stderr)


if __name__ == "__main__":
    main()
