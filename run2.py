import subprocess
import sys
import os

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
    scene = sys.argv[1] if len(sys.argv) > 1 else SCENE_DEFAULT

    if not os.path.exists(scene):
        print(f"Erro: cena não encontrada -> {scene}")
        sys.exit(1)

    return scene


def render(scene, output_ppm):
    print(f"Renderizando (com transformações) -> {output_ppm}")

    cmd = [sys.executable, "main.py", scene]

    with open(output_ppm, "w") as f:
        subprocess.run(cmd, stdout=f, stderr=sys.stderr)


def convert(Image, input_ppm, output_png):
    print(f"Convertendo {input_ppm} -> {output_png}")
    img = Image.open(input_ppm)
    img.save(output_png)


def main():
    Image = ensure_pillow()
    scene = resolve_scene()

    # 🔷 SOMENTE AFTER
    render(scene, OUTPUT["ppm"])
    convert(Image, OUTPUT["ppm"], OUTPUT["png"])

    print("\n✔ Processo concluído")
    print(f"→ {OUTPUT['png']}")


if __name__ == "__main__":
    main()