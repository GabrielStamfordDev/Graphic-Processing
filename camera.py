from src.Ponto import Ponto
from src.Vetor import Vetor
from utils.Scene.sceneSchema import CameraData


class Camera:
    """
    Representa uma câmera pinhole para ray casting.

    A câmera é definida pelos parâmetros fornecidos no arquivo de cena:
    - Posição da câmera (lookfrom)
    - Ponto para onde a câmera olha (lookat)
    - Vetor "para cima" (up_vector)
    - Distância até o plano de projeção (screen_distance)
    - Resolução da imagem (image_width, image_height)

    A partir desses dados, é construída uma base ortonormal (U, V, W):
    - W aponta no sentido oposto à direção de visão (de M para C)
    - U representa o eixo horizontal (direita da câmera)
    - V representa o eixo vertical (cima da câmera)

    Com essa base, define-se um plano de imagem (tela) localizado a uma
    distância d da câmera. A tela possui largura normalizada igual a 1.0
    e altura proporcional à resolução.

    Cada pixel da imagem é mapeado para um ponto nesse plano, e a direção
    de um raio é obtida a partir do vetor que liga a câmera a esse ponto.

    Esse modelo permite converter coordenadas discretas de pixel (i, j)
    em direções contínuas no espaço 3D.
    """

    def __init__(self, cam_data: CameraData):
        self.C = cam_data.lookfrom
        self.M = cam_data.lookat
        self.Vup = cam_data.up_vector
        self.d = cam_data.screen_distance
        
        self.hres = cam_data.image_width
        self.vres = cam_data.image_height
        
        direcao_w = self.C - self.M
        self.W = direcao_w.normalize()
        
        direcao_u = self.Vup.cross(self.W)
        self.U = direcao_u.normalize()
        
        self.V = self.W.cross(self.U)
        
        self.pixel_size = 1.0 / self.hres
        screen_width = 1.0
        screen_height = self.vres * self.pixel_size
        
        self.screen_center = self.C - (self.W * self.d)
        
        self.upper_left = self.screen_center - (self.U * (screen_width / 2.0)) + (self.V * (screen_height / 2.0))
        
    def get_ray_direction(self, i: int, j: int) -> Vetor:
        """
        Calcula a direção do raio correspondente ao pixel (i, j).

        O ponto central do pixel é obtido deslocando-se a partir do canto
        superior esquerdo da tela ao longo dos eixos U (horizontal) e V (vertical),
        considerando o tamanho de cada pixel.

        O deslocamento usa (i + 0.5) e (j + 0.5) para amostrar o centro do pixel,
        evitando viés de amostragem nas bordas.

        A direção do raio é então o vetor que liga a posição da câmera ao ponto
        calculado na tela, sendo normalizado antes do retorno.
        """

        """
Representação do mapeamento de pixels para o plano de projeção:

        V (up)
        ↑
        │
        │        (i,j)
        │      ┌───────┐
        │      │   •   │  ← centro do pixel (i+0.5, j+0.5)
        │      │       │
        │      └───────┘
        │          ↑
        │          │ Δy = (j+0.5)*pixel_size
        │
        └──────────────→ U (right)
                   │
                   │ Δx = (i+0.5)*pixel_size
                   ↓

   upper_left  ┌───────────────────────────┐
               │  .   .   .   .   .   .    │
               │                           │
               │  .   .   •   .   .   .    │
               │                           │
               │  .   .   .   .   .   .    │
               └───────────────────────────┘
                        screen plane

                ↘
                  ↘   raio
                    ↘
                      C (câmera)
"""
        deslocamento_x = self.U * ((i + 0.5) * self.pixel_size)
        deslocamento_y = self.V * ((j + 0.5) * self.pixel_size)
        
        pixel_center = self.upper_left + deslocamento_x - deslocamento_y
        
        direcao = pixel_center - self.C
        return direcao.normalize()