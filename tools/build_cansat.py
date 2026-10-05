import os
import subprocess
import sys

def main():
    workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    build_dir = os.path.join(workspace, "build")

    extra_paths = [
        r"C:\Espressif\tools\ninja\1.12.1",
        r"C:\Espressif\tools\cmake\3.30.2\bin",
        r"C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64",
        r"C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin",
        r"C:\Espressif\python_env\idf5.5_py3.11_env\Scripts",
    ]

    env = os.environ.copy()
    env["PATH"] = ";".join(extra_paths) + ";" + env.get("PATH", "")
    env["IDF_PATH"] = r"C:\Espressif\frameworks\esp-idf-v5.5.5"

    ninja_exe = r"C:\Espressif\tools\ninja\1.12.1\ninja.exe"
    print("=" * 60)
    print(" BUILDING CANSAT FIRMWARE WITH NINJA...")
    print("=" * 60)
    res = subprocess.run([ninja_exe], cwd=build_dir, env=env)
    return res.returncode

if __name__ == '__main__':
    sys.exit(main())
