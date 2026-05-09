import subprocess
import sys
import os

SCENE_DIR = "utils/input"
RESULTS_DIR = "resultados"

OUTPUTS = {
    "before": {
        "suffix": "before",
        "flag": "--no-transform"
    },
    "after": {
        "suffix": "after",
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


def get_input_files():
    """
    Busca arquivos:
    input1.json até input12.json
    """
    files = []

    for i in range(10, 13):
        path = os.path.join(SCENE_DIR, f"input{i}.json")

        if not os.path.exists(path):
            print(f"[AVISO] Arquivo não encontrado: {path}")
            continue

        files.append(path)

    return files


def ensure_results_dir():
    os.makedirs(RESULTS_DIR, exist_ok=True)


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


def process_scene(Image, scene_path):
    scene_name = os.path.splitext(os.path.basename(scene_path))[0]

    print(f"\n==============================")
    print(f"Processando cena: {scene_name}")
    print(f"==============================")

    for mode_name, config in OUTPUTS.items():

        ppm_path = os.path.join(
            RESULTS_DIR,
            f"{scene_name}_{config['suffix']}.ppm"
        )

        png_path = os.path.join(
            RESULTS_DIR,
            f"{scene_name}_{config['suffix']}.png"
        )

        render(scene_path, ppm_path, config["flag"])
        convert(Image, ppm_path, png_path)


def main():
    Image = ensure_pillow()

    ensure_results_dir()

    input_files = get_input_files()

    if not input_files:
        print("Nenhum arquivo input encontrado.")
        sys.exit(1)

    for scene in input_files:
        process_scene(Image, scene)

    print("\n✔ Todos os renders concluídos")
    print(f"Resultados salvos em: {RESULTS_DIR}")


if __name__ == "__main__":
    main()