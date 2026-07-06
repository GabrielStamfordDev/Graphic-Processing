#!/usr/bin/env python3
import subprocess
import sys

result = subprocess.run(['.\\main.exe', 'utils/input/monkeyScene.json'], 
                       capture_output=True, text=True)
print("STDERR OUTPUT:")
print(result.stderr[:2000])  # Print first 2000 chars of stderr
print("\n... (truncated)")
