import sys
from utils.Scene.sceneParser import SceneJsonLoader
from camera import Camera
from geometria import intersect_sphere, intersect_plane

def main():
    if len(sys.argv) < 2:
        print("Use: python main.py <caminho_para_cena.json> > out.ppm", file=sys.stderr)
        sys.exit(1)

    scene_file = sys.argv[1]
    scene_data = SceneJsonLoader.load_file(scene_file)
    cam = Camera(scene_data.camera)
    
    print(f"P3\n{cam.hres} {cam.vres}\n255")
    
    # Loop principal de Ray Casting
    for j in range(cam.vres):
        print(f"Renderizando linha {j}/{cam.vres}...", file=sys.stderr, end='\r')
        
        for i in range(cam.hres):
            ray_dir = cam.get_ray_direction(i, j)
            
            # Variáveis para rastrear o objeto mais próximo
            closest_t = float('inf')
            hit_color = None
            
            # Testa a interseção com TODOS os objetos da cena
            for obj in scene_data.objects:
                t = float('inf')
                
                if obj.obj_type == "sphere":
                    # O parser mapeia o 'center' do JSON para relative_pos
                    centro = obj.relative_pos
                    raio = obj.get_num("radius")
                    t = intersect_sphere(cam.C, ray_dir, centro, raio)
                    
                elif obj.obj_type == "plane":
                    # O parser mapeia o 'point_on_plane' para relative_pos
                    ponto_plano = obj.relative_pos
                    normal = obj.get_vetor("normal").normalize()
                    t = intersect_plane(cam.C, ray_dir, ponto_plano, normal)
                
                # Se achamos uma interseção mais próxima, atualizamos a cor
                if t < closest_t:
                    closest_t = t
                    hit_color = obj.material.color
            
            # --- Definição da Cor do Pixel ---
            if hit_color is not None:
                # O objeto foi atingido. As cores no JSON vão de 0.0 a 1.0.
                # Transformamos para a escala PPM (0 a 255)
                r = int(255.999 * hit_color.r)
                g = int(255.999 * hit_color.g)
                b = int(255.999 * hit_color.b)  
            else:
                # O raio não atingiu nada, pinta de Preto (Fundo)
                r, g, b = 0, 0, 0
                
            print(f"{r} {g} {b}")
            
    print("\nRenderização concluída com sucesso!", file=sys.stderr)

if __name__ == "__main__":
    main()
