#!/usr/bin/env python3
import os
import sys

os.chdir(r'c:\Users\giova\Graphic-Processing')

# Simular uma renderização com o comando
print("Testando caso6.json (teste simples)...")
os.system('py run2.py utils/input/caso6.json --cpp > test_caso6.log 2>&1')
print("✓ caso6.json concluído, salvo em test_caso6.log")

print("\nTestando monkeyScene.json (com múltiplos meshes transformados)...")
os.system('py run2.py utils/input/monkeyScene.json --cpp > test_monkey.log 2>&1')
print("✓ monkeyScene.json concluído, salvo em test_monkey.log")

print("\n=== Verificando output.ppm ===")
if os.path.exists('output.ppm'):
    size = os.path.getsize('output.ppm')
    print(f"output.ppm: {size} bytes")
else:
    print("output.ppm não encontrado!")
