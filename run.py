import subprocess
import sys
import os

SCENE_DEFAULT = "utils/input/sampleScene.json"

OUTPUTS = {
    "before": {
        "ppm": "before.ppm",
        "png": "before.png",
        "flag": "--no-transform"
    },
    "after": {
        "ppm": "after.ppm",
        "png": "after.png",
        "flag": None
    }
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


def render(scene, output_ppm, flag=None):
    print(f"Renderizando -> {output_ppm}")

    cmd = [sys.executable, "main.py", scene]

    if flag:
        cmd.append(flag)

    with open(output_ppm, "w") as f:
        subprocess.run(cmd, stdout=f, stderr=sys.stderr)


def convert(Image, input_ppm, output_png):
    print(f"Convertendo {input_ppm} -> {output_png}")
    img = Image.open(input_ppm)
    img.save(output_png)


def main():
    Image = ensure_pillow()
    scene = resolve_scene()

    # 🔷 BEFORE (sem transformação)
    render(scene, OUTPUTS["before"]["ppm"], OUTPUTS["before"]["flag"])
    convert(Image, OUTPUTS["before"]["ppm"], OUTPUTS["before"]["png"])

    # 🔷 AFTER (com transformação)
    render(scene, OUTPUTS["after"]["ppm"], OUTPUTS["after"]["flag"])
    convert(Image, OUTPUTS["after"]["ppm"], OUTPUTS["after"]["png"])

    print("\n✔ Processo concluído")
    print(f"→ {OUTPUTS['before']['png']}")
    print(f"→ {OUTPUTS['after']['png']}")


if __name__ == "__main__":
    main()