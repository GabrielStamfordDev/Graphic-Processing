import subprocess
import sys
import os
import time

SCENE_DEFAULT = "utils/input/sampleScene.json"

OUTPUT = {
    "ppm": "output.ppm",
    "png": "output.png",
}


def ensure_pillow():
    try:
        from PIL import Image
        return Image
    except ImportError:
        print("Instalando Pillow...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pillow"])
        from PIL import Image
        return Image


def resolve_scene():
    scene = None
    use_cpp = False

    for arg in sys.argv[1:]:
        if arg == "--cpp":
            use_cpp = True
        elif not arg.startswith("--") and scene is None:
            scene = arg

    scene = scene or SCENE_DEFAULT

    if not os.path.exists(scene):
        print(f"Erro: cena não encontrada -> {scene}")
        sys.exit(1)

    return scene, use_cpp


def render(scene, output_ppm, use_cpp=False):
    modo = "C++ (main.exe)" if use_cpp else "Python (main.py)"
    print(f"Renderizando ({modo}) -> {output_ppm}")

    if use_cpp:
        cmd = ["main.exe", scene]
    else:
        cmd = [sys.executable, "main.py", scene]

    # Inicia a contagem do tempo antes de disparar o processo
    inicio = time.perf_counter()

    with open(output_ppm, "w") as f:
        subprocess.run(cmd, stdout=f, stderr=sys.stderr)

    # Finaliza a contagem após o término do processo
    fim = time.perf_counter()
    tempo_gasto = fim - inicio

    print(f"⏱️  Tempo de renderização: {tempo_gasto:.4f} segundos.\n")


def convert(Image, input_ppm, output_png):
    print(f"Convertendo {input_ppm} -> {output_png}")
    img = Image.open(input_ppm)
    img.save(output_png)


def main():
    Image = ensure_pillow()
    scene, use_cpp = resolve_scene()

    # 🔷 SOMENTE AFTER
    render(scene, OUTPUT["ppm"], use_cpp)
    convert(Image, OUTPUT["ppm"], OUTPUT["png"])

    print("\n✔ Processo concluído")
    print(f"→ {OUTPUT['png']}")


if __name__ == "__main__":
    main()