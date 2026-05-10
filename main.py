import sys
import numpy as np
from pathlib import Path

from utils.Scene.sceneParser import SceneJsonLoader
from utils.MeshReader.ObjReader import ObjReader
from camera import Camera
from geometria import intersect_sphere, intersect_plane, intersect_triangles_numpy
from transformacoes import (
    matriz_translacao, matriz_escala,
    matriz_rotacao_x, matriz_rotacao_y, matriz_rotacao_z,
    aplicar_matriz_ponto
)


def intersect_object(obj, ray_origin, ray_dir):
    if obj.obj_type == "sphere":
        return intersect_sphere(ray_origin, ray_dir, obj.relative_pos, obj.get_num("radius"))

    if obj.obj_type == "plane":
        normal = obj.get_vetor("normal").normalize()
        return intersect_plane(ray_origin, ray_dir, obj.relative_pos, normal)

    if obj.obj_type == "mesh":
        return intersect_triangles_numpy(ray_origin, ray_dir, obj.np_v0, obj.np_v1, obj.np_v2)

    return float('inf')


def main():
    if len(sys.argv) < 2:
        print("Use: python main.py <cena.json> [--no-transform]", file=sys.stderr)
        sys.exit(1)

    scene_file = sys.argv[1]

    apply_transform = True
    if len(sys.argv) > 2 and sys.argv[2] == "--no-transform":
        apply_transform = False

    scene_data = SceneJsonLoader.load_file(scene_file)
    cam = Camera(scene_data.camera)

    # --- PRÉ-CARREGAMENTO ---
    loaded_meshes = {}
    valid_objects = []

    for obj in scene_data.objects:

        if obj.obj_type == "mesh":
            obj_path = obj.get_property("path")

            # 🔴 ignora arquivo inexistente
            if not Path(obj_path).exists():
                print(f"[AVISO] OBJ não encontrado: {obj_path} → ignorando objeto", file=sys.stderr)
                continue

            if obj_path not in loaded_meshes:
                print(f"Carregando: {obj_path}...", file=sys.stderr)
                loaded_meshes[obj_path] = ObjReader(obj_path)

            mesh_reader = loaded_meshes[obj_path]
            faces = mesh_reader.get_face_points()

            # 🔴 ignora OBJ inválido/vazio
            if not faces:
                print(f"[AVISO] OBJ vazio ou inválido: {obj_path} → ignorando", file=sys.stderr)
                continue

            v0_list, v1_list, v2_list = [], [], []

            if apply_transform:
                M_total = np.eye(4)

                for t in obj.transforms:
                    M_atual = np.eye(4)

                    if t.t_type == "scaling":
                        M_atual = matriz_escala(t.data.x, t.data.y, t.data.z)

                    elif t.t_type == "translation":
                        M_atual = matriz_translacao(t.data.x, t.data.y, t.data.z)

                    elif t.t_type == "rotation":
                        Mx = matriz_rotacao_x(t.data.x)
                        My = matriz_rotacao_y(t.data.y)
                        Mz = matriz_rotacao_z(t.data.z)
                        M_atual = Mz @ My @ Mx

                    M_total = M_atual @ M_total

            for face_pts in faces:
                if apply_transform:
                    v0t = aplicar_matriz_ponto(M_total, face_pts[0])
                    v1t = aplicar_matriz_ponto(M_total, face_pts[1])
                    v2t = aplicar_matriz_ponto(M_total, face_pts[2])
                else:
                    v0t = face_pts[0]
                    v1t = face_pts[1]
                    v2t = face_pts[2]

                v0_list.append([v0t.x, v0t.y, v0t.z])
                v1_list.append([v1t.x, v1t.y, v1t.z])
                v2_list.append([v2t.x, v2t.y, v2t.z])

            obj.np_v0 = np.array(v0_list, dtype=np.float64)
            obj.np_v1 = np.array(v1_list, dtype=np.float64)
            obj.np_v2 = np.array(v2_list, dtype=np.float64)

            print(f"  → {len(faces)} triângulos.", file=sys.stderr)

        # 🔴 só adiciona objetos válidos
        valid_objects.append(obj)

    # 🔴 substitui lista original
    scene_data.objects = valid_objects

    # --- RENDER ---
    print(f"P3\n{cam.hres} {cam.vres}\n255")

    for j in range(cam.vres):
        print(f"Linha {j}/{cam.vres}", file=sys.stderr, end='\r')

        for i in range(cam.hres):
            ray_dir = cam.get_ray_direction(i, j)

            closest_t = float('inf')
            hit_color = None

            for obj in scene_data.objects:
                t = intersect_object(obj, cam.C, ray_dir)

                if t < closest_t:
                    closest_t = t
                    hit_color = obj.material.color

            if hit_color is not None:
                r = int(255.999 * hit_color.r)
                g = int(255.999 * hit_color.g)
                b = int(255.999 * hit_color.b)
            else:
                r, g, b = 0, 0, 0

            print(f"{r} {g} {b}")

    print("\nRenderização concluída!", file=sys.stderr)


if __name__ == "__main__":
    main()