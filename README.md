# PG-Ray-Tracing

Repositório base para o projeto da disciplina de Processamento Gráfico.

## Entregas

O projeto consiste de 4 entregas mais uma entrega extra, sendo elas:
1. [Raycasting com esferas e planos](#1-raycasting-com-esferas-e-planos)
2. [Raycasting com malhas de triâgulos;](#2-raycasting-com-malhas-de-triângulos)
3. [Raytracing não recursivo;](#3-raytracing-não-recursivo)
4. [Raytracing recursivo;](#4-raytracing-recursivo)
5. [Features extras individuais](#5-features-individuais)

## 1. Raycasting com Esferas e Planos
Para um Raycaster básico, os grupos precisarão:

### Definir tipos para pontos e vetores.
O grupo pode usar bibliotecas para trabalhar com pontos e vetores. Mas também é possível definir as estruturas de dados por meio de structs/classes/records e as operações por meio de funções (ou métodos).

### Definir a câmera
    
  Ela pode ser uma classe composta por:
  
  - Um ponto, que definirá sua localização no mundo $\small \longrightarrow C({c_1}, {c_2}, {c_3}) \; onde,  \;{c_1}, \;{c_2}, \;{c_3} \in \mathbb{R};$
  - Um outro ponto no mundo que será para onde a câmera aponta (ela sempre aponta para o centro da tela) $\small \longrightarrow M(x, y, z) \; onde, \; x, \; y, \; z \in \mathbb{R};$
  - Um vetor que parte da câmera e aponta para cima $\small \longrightarrow \; {V_{up}}({v_1}, {v_2}, {v_3}) \;onde, \;{v_1}, \;{v_2}, \;{v_3} \in \mathbb{R} \; e \; \;{v_{up}} \neq (vetor \;nulo);$
  - Três vetores ortonormais $\small \longrightarrow W({w_1}, {w_2}, {w_3}), \; V({v_1}, {v_2}, {v_3}) e \; U({u_1}, {u_2}, {u_3}) \; onde \; {w_1}, \; {w_2}, \; {w_3}, \; {v_1}, \; {v_2}, \; {v_3}, \; {u_1}, \; {u_2}, \; {u_3} \in \mathbb{R}.$  Por convenção, um desses vetores deve ter a mesma direção que (M − C), mas sentido oposto; o ponto da mira e o ponto da localização da câmera, respectivamente;
  - A distância entre a câmera e a tela $\small \longrightarrow d \in \mathbb{R_+}^*$
  - Um número que definirá a altura da tela $\small \longrightarrow {v_{res}} \in \mathbb{Z_+}^*;$
  - Um número que definirá a largura da tela $\small \longrightarrow {h_{res}} \in \mathbb{Z_+}^*.$
    
> ***OBS:***  Utilizamos a resolução padrão dos pixels como 1x1. Isso quer dizer que, por exemplo, se olharmos apenas para a resolução horizontal da tela, ela será dividida em pixels de tamanho $\small \frac{1}{{h_{res}}}$ Neste projeto não será útil trabalhar com variantes dessa resolução.
>
> ***OBS:*** O ângulo de visão da câmera, no geral, é uma consequência direta da definição da resolução da tela e da distância que a câmera se encontra dela.
  
  - Com todos esses atributos o grupo deverá construir uma câmera móvel, ou seja, podendo ser posicionada em qualquer lugar e apontada para diferentes localizações. Ao mesmo tempo, esses atributos permitirão mapear todos os pixels da tela, que poderão ser encontrados pela soma de uma combinação de vetores da base da câmera pela localização da câmera.

### Testar interseções
  - Esferas
      - Um ponto que determina seu centro $\small \longrightarrow \; {C_{\varepsilon}}(x, y, z) \; onde \; x, \; y, \; z \in \mathbb{R};$
      - O raio $\longrightarrow {r} \in \mathbb{R_+}^*;$
      - Cor RGB normalizada $\small \longrightarrow {O_d} \in [0,1]^3$
  - Planos
      - Um ponto pertencente ao plano $\small \longrightarrow {P_p}(x, y, z) \; onde \; x, \; y, \; z \in \mathbb{R}$
      - Um vetor normal ao plano $\small \longrightarrow {V}({v_1}, {v_2}, {v_3}) \; onde \; {v_1}, \; {v_2}, \; {v_3} \in \mathbb{R}$
      - Cor RGB normalizada $\small \longrightarrow {O_d} \in [0, 1]^3$

OBS: Será necessário renderizar a cena com os objetos (esferas e planos).

## 2. Raycasting com Malhas de Triângulos

### Nessa fase os grupos implementarão a interseção com malhas de triângulos, ela é definida da seguinte forma:

- Número de triângulos $\small \longrightarrow {n_{\bigtriangleup}} \in \mathbb{N}$
- Número total de vértices $\small \longrightarrow {n_{\circ}} \in \mathbb{N}, {n_{\circ}} \geqslant 3$
- Uma lista de vértices (pontos) $\small \longrightarrow tamanho: {n_{\circ}}$
- Uma lista com triplas de índices de vértices (cada tripla possui os índices dos vértices (na lista de vértices) que fazem parte de um triângulo) $\small \longrightarrow tamanho: {n_{\bigtriangleup}}$
- Uma lista com normais de triângulos (vetores) $\small \longrightarrow tamanho: {n_{\bigtriangleup}}$
- Uma lista com normais dos vértices; cada elemento da lista é um vetor que é a média das normais dos triângulos que compartilham o correspondente vértice $\small \longrightarrow tamanho: {n_{\circ}}$
- Cor RGB normalizada $\small \longrightarrow {O_d} \in [0, 1]^3$
- Ela PRECISARÀ receber inputs de arquivos .OBJ, seja utilizando o projeto base da disciplina ou um desenvolvido pelo aluno.

Além disso, os grupos também implementarão transformações afins. Elas são definidas da seguinte forma:

- Para isso, o grupo precisa desenvolver uma maneira de construir matrizes de floats ou doubles, podem ser arrays por exemplo;
- Além disso, deve ser possível aplicar matrizes a pontos e vetores;
- Não é preciso fazer uma animação para mostrar o efeito da transformação, vocês podem por exemplo, fazer com que o programa renderize duas imagens, uma antes da aplicação de uma transformação afim e outra depois.

## 3. Raytracing não Recursivo

### Nessa etapa, cada grupo precisa implementar o modelo de iluminação de Phong (sem recursão) e sombras.

- Todos os objetos possuem os seguintes atributos para determinar seu material:
    - Coeficiente difuso $\small \longrightarrow {k_d} \in [0, 1]^3$
    - Coeficiente especular $\small \longrightarrow {k_s} \in [0, 1]^3$
    - Coeficiente ambiental $\small \longrightarrow {k_a} \in [0, 1]^3$
    - Coeficiente de reflexão $\small \longrightarrow {k_r} \in [0, 1]^3$
    - Coeficiente de transmissão $\small \longrightarrow {k_t} \in [0, 1]^3$
    - Coeficiente de rugosidade $\small \longrightarrow \eta > 0$
- Definir as fontes de luz
    
    **Ela pode ser uma classe com os seguintes atributos:**
    
    - Luzes
        - Cada luz é um ponto, que determina sua localização $\small \longrightarrow \; {l}(x, y, z) \; onde, \; x, \; y, \; z \in \mathbb{R}$
        - Intensidade da luz, uma cor RGB $\small \longrightarrow \; {I_{L_n}} \in \; [0, 255]^3 \; onde \; 1 \leqslant \; {n} \leqslant \; {m}.$
    - Ambiente
        - Uma cor ambiente, funciona como um filtro $\small \longrightarrow \; {I_a} \in \;[0, 255]^3.$
- Implementar o modelo de iluminação de Phong;
    - O programa continuará executando o Ray-Casting, mas, agora, ao invés de retornar apenas a cor dos objetos interceptados para cada pixel; ele irá calcular a cor de cada um deles de acordo com o modelo de iluminação de Phong;
    - Por hora, a parte (kr · Ir + kt · It) da equação de Phong deve ser ignorada, já que as reflexões e refrações fazem parte da segunda entrega.

- Alguns dos parâmetros da equação de Phong acima, não foram exemplificados em nenhum objeto, esses são os seguintes:
    - O vetor que parte do ponto da superfície do objeto onde a interseção ocorreu e aponta para a luz  $\small {I_{L_n}} \longrightarrow \; {L_n}({ln_1}, {ln_2}, {ln_3}) \; onde, \; {ln_1}, \; {ln_2}, \; {ln_3} \in \mathbb{R} \; e \; 1 \leqslant {n} \leqslant {m};$
    - A normal no ponto da superfície do objeto onde a interseção ocorreu $\small \longrightarrow \; N({n_1}, {n_2}, {n_3}) \; onde, \; {n_1}, \; {n_2}, \; {n_3} \in \mathbb{R};$
    - O vetor de reflexão em relação a luz $\small {I_{L_n}} \; no  \; ponto \; da \;superfície \; onde \; a \;interseção \; ocorreu \; \longrightarrow  \; {R_n}({rn_1}, {rn_2}, {rn_3}) \; onde \; {rn_1}, \; {rn_2}, \; {rn_3} \in \mathbb \; {R} \; e \; 1 \leqslant \; {n} \leqslant \; {m};$
    - O vetor que aponta para o espectador. Para os raios primários, no Ray-Casting por exemplo, esse espectador é a câmera. O espectador muda quando estamos tratando reflexões e refrações $\small \longrightarrow \; V({v_1}, {v_2}, {v_3}) \; onde, \; {v_1}, \; {v_2}, \; {v_3} \in \mathbb\; {R}$
    - A cor RGB retornada pelo raio refletido $\small \longrightarrow \; {I_r} \in \; [0,255]^3$
    - A cor RGB retornada pelo raio refratado $\small \longrightarrow {I_t} \in [0,255]^3$

## 4. Raytracing Recursivo

### Nessa etapa, cada grupo precisa adicionar a iluminação recursiva (refrações e reflexões) ao modelo de Phong.

- Aqui vocês precisam definir os índices de refração dos objetos na cena $\small \longrightarrow IOR \in \mathbb \; {R}, \; IOR \geqslant \; 1;$
- Considerem o IOR do ar como sendo 1;
    - Implementem da forma como vocês preferirem, podem incluir na entrada do programa ou defini-los na main;
- Para todo objeto com propriedades reflexivas ou transparência é preciso “lançar” raios secundários;
- Como nosso ray-tracer já sabe fazer o ray-casting, que é o “lançamento de raios”; vamos apenas precisar fazer uma chamada recursiva ao ray-casting para que ele “lance” raios a partir dos pontos de interseção com objetos onde a reflexão ou refração devem ocorrer;
- Aqui iremos somar a cor calculada pelo modelo de Phong a essa cor secundária oriunda dos raios refletidos e refratados.

## 5. Features Individuais

⚠️ Cada aluno deve escolher uma feature extra, que será implementada individualmente. Pessoas que fizeram parte do mesmo grupo nas entregas passadas não podem escolher uma mesma feature.

| **Nível de dificuldade**  | **Tempo estimado de implementação** | **Nota máxima** |
| --- | --- | --- |
| Fácil | 1 dia | 4 pontos |
| Média | 3 a 4 dias | 7 pontos  |
| Difícil | 7 a 10 dias | 10 pontos |

### Features fáceis

- Anti-aliasing (supersampling)
    - Lança um raio pra cada corner de um pixel e pega a média pra ser a cor daquele pixel.
- Cones e Cilindros
- Paraboloide
- Textura em planos
    - A posição do ponto vai definir onde vai pegar o pixel correspondente na imagem usada de base pra textura.
- Textura em esferas
    - Usar sistema de coordenadas esfericas.
- Textura procedural
    - Não se baseia em um arquivo, mas sim em uma regra.
- Tone maping
- Bumping mapping (via randomização de norma)

### Features médias

- Soft shadows
    - Fonte de luz extensa, implementada com uma quantidade prescrita de pontos igualmente espaçados ao longo da fonte ou aleatoriamente definidos.
- Renderizar um toro como uma malha de triângulos
    - Veja como se faz a renderização de um toro a  partir de um retângulo e gere triângulos a partir daí
- Textura sólida
- Iluminação de raios paralelos saindo de um retângulo
    - Como se fosse uma janela aberta em um dia ensolarado. Se assemelha a uma fonte extensa, mas os raios de luz que partem dele têm direção predefinida.

### Features difíceis

- Renderizar uma superfície de Bézier
- Renderizar uma superfície de revolução com a curva geratriz sendo uma curva de Bézier
- Octrees (subdivisão de bounding-box)
- Relief mapping
- Binary Space Partitioning (BSP)
