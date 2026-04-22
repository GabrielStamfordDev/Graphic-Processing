import sys
import numpy as np
from utils.Scene.sceneParser import SceneJsonLoader
from utils.MeshReader.ObjReader import ObjReader
from camera import Camera
from geometria import intersect_sphere, intersect_plane, intersect_triangle
from transformacoes import (
    matriz_translacao, matriz_escala, 
    matriz_rotacao_x, matriz_rotacao_y, matriz_rotacao_z, 
    aplicar_matriz_ponto
)

def main():
    if len(sys.argv) < 2:
        print("Use: python main.py <caminho_para_cena.json>", file=sys.stderr)
        sys.exit(1)

    scene_file = sys.argv[1]
    scene_data = SceneJsonLoader.load_file(scene_file)
    cam = Camera(scene_data.camera)
    
    # --- PRÉ-CARREGAMENTO E TRANSFORMAÇÃO DE MALHAS ---
    loaded_meshes = {}
    for obj in scene_data.objects:
        if obj.obj_type == "mesh":
            obj_path = obj.get_property("path") # Acessa a propriedade "path" de monkeyScene.txt
            
            # Carrega o ObjReader cru apenas uma vez por arquivo
            if obj_path not in loaded_meshes:
                print(f"Carregando arquivo 3D bruto: {obj_path}...", file=sys.stderr)
                loaded_meshes[obj_path] = ObjReader(obj_path)
            
            mesh_reader = loaded_meshes[obj_path]
            
            # 1. Constrói a Matriz 4x4 Final (Começando com a Matriz Identidade)
            M_total = np.eye(4)
            
            for t in obj.transforms:
                M_atual = np.eye(4)
                # O parser mapeia o vetor (factors, vector, angle) sempre para t.data
                if t.t_type == "scaling":
                    M_atual = matriz_escala(t.data.x, t.data.y, t.data.z)
                elif t.t_type == "translation":
                    M_atual = matriz_translacao(t.data.x, t.data.y, t.data.z)
                elif t.t_type == "rotation":
                    Mx = matriz_rotacao_x(t.data.x)
                    My = matriz_rotacao_y(t.data.y)
                    Mz = matriz_rotacao_z(t.data.z)
                    M_atual = Mz @ My @ Mx  # Ordem padrão de Euler (Z * Y * X)
                
                # Multiplicação matricial (Transformação mais recente é aplicada por último)
                M_total = M_atual @ M_total 

            # 2. Aplica a Matriz M_total em todos os triângulos e guarda neste objeto
            triangulos_transformados = []
            for face_pts in mesh_reader.get_face_points():
                v0_trans = aplicar_matriz_ponto(M_total, face_pts[0])
                v1_trans = aplicar_matriz_ponto(M_total, face_pts[1])
                v2_trans = aplicar_matriz_ponto(M_total, face_pts[2])
                triangulos_transformados.append((v0_trans, v1_trans, v2_trans))
            
            # Grudamos a lista de triângulos "prontos para renderizar" dentro da instância do objeto
            obj.faces_transformadas = triangulos_transformados
    # ------------------------------------------------

    print(f"P3\n{cam.hres} {cam.vres}\n255")
    
    for j in range(cam.vres):
        print(f"Renderizando linha {j}/{cam.vres}...", file=sys.stderr, end='\r')
        
        for i in range(cam.hres):
            ray_dir = cam.get_ray_direction(i, j)
            
            closest_t = float('inf')
            hit_color = None
            
            for obj in scene_data.objects:
                if obj.obj_type == "sphere":
                    # (Mesmo código da esfera)
                    t = intersect_sphere(cam.C, ray_dir, obj.relative_pos, obj.get_num("radius"))
                    if t < closest_t:
                        closest_t = t
                        hit_color = obj.material.color
                        
                elif obj.obj_type == "plane":
                    # (Mesmo código do plano)
                    normal = obj.get_vetor("normal").normalize()
                    t = intersect_plane(cam.C, ray_dir, obj.relative_pos, normal)
                    if t < closest_t:
                        closest_t = t
                        hit_color = obj.material.color
                        
                elif obj.obj_type == "mesh":
                    # Agora iteramos pela lista de triângulos JÁ TRANSFORMADOS exclusivos deste objeto!
                    for v0, v1, v2 in obj.faces_transformadas:
                        t = intersect_triangle(cam.C, ray_dir, v0, v1, v2)
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
            
    print("\nRenderização concluída com sucesso!", file=sys.stderr)

if __name__ == "__main__":
    main()
