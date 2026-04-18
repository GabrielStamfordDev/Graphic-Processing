import subprocess
import sys
import os

SCENE_DEFAULT = "utils/input/sampleScene.json"
OUTPUT_PPM = "output.ppm"
OUTPUT_PNG = "output.png"


def ensure_pillow():
    """
    Garante que a biblioteca Pillow esteja disponível no ambiente.
    Caso não esteja instalada, realiza a instalação automaticamente
    utilizando o mesmo interpretador Python em execução.
    """
    try:
        from PIL import Image
        return Image
    except ImportError:
        print("Instalando dependência necessária (Pillow)...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pillow"])
        from PIL import Image
        return Image


def resolve_scene():
    """
    Determina qual arquivo de cena será utilizado.

    - Se um argumento for passado via linha de comando, ele será usado.
    - Caso contrário, utiliza a cena padrão definida no script.

    Retorna o caminho válido da cena ou encerra o programa em caso de erro.
    """
    scene = sys.argv[1] if len(sys.argv) > 1 else SCENE_DEFAULT

    if not os.path.exists(scene):
        print(f"Erro: arquivo de cena não encontrado -> {scene}")
        sys.exit(1)

    return scene


def render_scene(scene):
    """
    Executa o ray tracer principal (main.py), redirecionando a saída
    padrão para um arquivo PPM.

    O stderr é mantido no terminal para exibir progresso e mensagens
    de execução.
    """
    print(f"Iniciando renderização da cena: {scene}")

    with open(OUTPUT_PPM, "w") as f:
        subprocess.run(
            [sys.executable, "main.py", scene],
            stdout=f,
            stderr=sys.stderr
        )


def convert_to_png(Image):
    """
    Converte o arquivo PPM gerado para o formato PNG utilizando Pillow.
    """
    print("Convertendo imagem para PNG...")

    img = Image.open(OUTPUT_PPM)
    img.save(OUTPUT_PNG)


def main():
    Image = ensure_pillow()
    scene = resolve_scene()
    render_scene(scene)
    convert_to_png(Image)

    print("Processo finalizado com sucesso.")
    print(f"Arquivo gerado: {OUTPUT_PNG}")


if __name__ == "__main__":
    main()