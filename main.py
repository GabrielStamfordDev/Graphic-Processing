import sys
import numpy as np
from pathlib import Path

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

from src.Ponto import Ponto

EPSILON = 1e-6


# ============================================================
# INTERSEÇÃO
# ============================================================

def intersect_object(obj, ray_origin, ray_dir):

    if obj.obj_type == "sphere":

        return intersect_sphere(
            ray_origin,
            ray_dir,
            obj.relative_pos,
            obj.get_num("radius")
        )

    elif obj.obj_type == "plane":

        normal = obj.get_vetor(
            "normal"
        ).normalize()

        return intersect_plane(
            ray_origin,
            ray_dir,
            obj.relative_pos,
            normal
        )

    elif obj.obj_type == "mesh":

        return intersect_triangles_numpy(
            ray_origin,
            ray_dir,
            obj.np_v0,
            obj.np_v1,
            obj.np_v2
        )

    return float("inf")


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
    # relativePos do JSON é IGNORADO para mesh.
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
    # POSIÇÃO FINAL NO MUNDO
    # ========================================================

    M_relative = matriz_translacao(
        relative_pos.x,
        relative_pos.y,
        relative_pos.z
    )

    M_total = M_relative @ M_total

    # ========================================================
    # APLICAÇÃO NOS VÉRTICES
    # ========================================================

    v0_list = []
    v1_list = []
    v2_list = []

    for face_pts in faces:

        if apply_transform:

            v0t = aplicar_matriz_ponto(
                M_total,
                face_pts[0]
            )

            v1t = aplicar_matriz_ponto(
                M_total,
                face_pts[1]
            )

            v2t = aplicar_matriz_ponto(
                M_total,
                face_pts[2]
            )

        else:

            v0t = face_pts[0]
            v1t = face_pts[1]
            v2t = face_pts[2]

        v0_list.append(
            [v0t.x, v0t.y, v0t.z]
        )

        v1_list.append(
            [v1t.x, v1t.y, v1t.z]
        )

        v2_list.append(
            [v2t.x, v2t.y, v2t.z]
        )

    obj.np_v0 = np.array(
        v0_list,
        dtype=np.float64
    )

    obj.np_v1 = np.array(
        v1_list,
        dtype=np.float64
    )

    obj.np_v2 = np.array(
        v2_list,
        dtype=np.float64
    )

    print(
        f"  → {len(faces)} triângulos.",
        file=sys.stderr
    )

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
    # RENDER
    # ========================================================

    print(f"P3\n{cam.hres} {cam.vres}\n255")

    for j in range(cam.vres):

        print(
            f"Linha {j}/{cam.vres}",
            file=sys.stderr,
            end='\r'
        )

        for i in range(cam.hres):

            ray_dir = cam.get_ray_direction(
                i,
                j
            )

            closest_t = float("inf")

            hit_color = None

            for obj in scene_data.objects:

                t = intersect_object(
                    obj,
                    cam.C,
                    ray_dir
                )

                if EPSILON < t < closest_t:

                    closest_t = t
                    hit_color = obj.material.color

            if hit_color is not None:

                r = int(255.999 * hit_color.r)
                g = int(255.999 * hit_color.g)
                b = int(255.999 * hit_color.b)

            else:

                r, g, b = 0, 0, 0

            print(f"{r} {g} {b}")

    print(
        "\nRenderização concluída!",
        file=sys.stderr
    )


if __name__ == "__main__":
    main()