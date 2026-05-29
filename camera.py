import numpy as np
from src.Ponto import Ponto
from src.Vetor import Vetor
from utils.Scene.sceneSchema import CameraData

class Camera:
    def __init__(self, cam_data: CameraData):
        # GARANTIA: Converte entradas da cena para Ponto/Vetor caso venham do NumPy
        self.C = self._to_ponto(cam_data.lookfrom)
        self.M = self._to_ponto(cam_data.lookat)
        self.Vup = self._to_vetor(cam_data.up_vector)
        self.d = float(cam_data.screen_distance)
        
        self.hres = cam_data.image_width
        self.vres = cam_data.image_height
        
        # --- Construção da Base Ortonormal (U, V, W) ---
        # W aponta para trás (da cena para a câmera)
        direcao_w = self.C - self.M
        self.W = direcao_w.normalize()
        
        # U é o eixo horizontal (direita)
        direcao_u = self.Vup.cross(self.W)
        self.U = direcao_u.normalize()
        
        # V é o eixo vertical (cima)
        self.V = self.W.cross(self.U)
        
        # --- Configuração do Plano de Imagem ---
        # Largura da tela fixa em 1.0, altura proporcional
        self.pixel_size = 1.0 / self.hres
        screen_width = 1.0
        screen_height = self.vres * self.pixel_size
        
        # Centro da tela projetada a distância d
        self.screen_center = self.C - (self.W * self.d)
        
        # Canto superior esquerdo da tela (Ponto de partida para o rastreio)
        # Deslocamos metade da largura para a esquerda (-U) e metade da altura para cima (+V)
        self.upper_left = self.screen_center - (self.U * (screen_width / 2.0)) + (self.V * (screen_height / 2.0))

    def _to_ponto(self, p):
        """Helper para blindagem contra NumPy."""
        if isinstance(p, np.ndarray):
            return Ponto(p[0], p[1], p[2])
        return p

    def _to_vetor(self, v):
        """Helper para blindagem contra NumPy."""
        if isinstance(v, np.ndarray):
            return Vetor(v[0], v[1], v[2])
        return v

    def get_ray_direction(self, i: int, j: int) -> Vetor:
        """
        Calcula a direção do raio para o pixel (i, j).
        Garante o retorno de um objeto da classe Vetor.
        """
        # (i + 0.5) amostra o centro do pixel para evitar aliasing de borda
        deslocamento_x = self.U * ((i + 0.5) * self.pixel_size)
        deslocamento_y = self.V * ((j + 0.5) * self.pixel_size)
        
        # Ponto no mundo 3D correspondente ao pixel na tela
        pixel_center = self.upper_left + deslocamento_x - deslocamento_y
        
        # Direção: do centro da câmera (C) para o ponto na tela
        direcao = pixel_center - self.C
        return direcao.normalize()