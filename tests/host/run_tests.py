"""
Host-side flight-software tests (no hardware needed).

  test_attitude : AttitudeReference - mount detection, level trim, conventions,
                  exact Euler rebuild through gimbal lock, tare, flight behaviour
  test_mission  : VerticalKF + MissionSupervisor + ReturnGuidance + SteerController
                  - rocket and drone-carrier flights with baro spikes, ejection pulses,
                    4 g accel clipping; aborted carrier descent; pad handling;
                    mid-air reset; return-to-launch closed loop with wind / GNSS lag

Usage:  python tests/host/run_tests.py          (needs g++ on PATH, e.g. MSYS2 mingw64)
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
INC = ["-I" + os.path.join(HERE, "stub"),
       "-I" + os.path.join(ROOT, "components", "nav", "include"),
       "-I" + os.path.join(ROOT, "components", "control", "include"),
       "-I" + os.path.join(ROOT, "components", "drivers", "include")]

TESTS = {
    "test_attitude": ["test_attitude.cpp", os.path.join(ROOT, "components", "drivers", "src", "imu_attitude.cpp")],
    "test_mission":  ["test_mission.cpp"],
}


def main():
    gxx = shutil.which("g++") or (r"C:\msys64\mingw64\bin\g++.exe" if os.path.exists(r"C:\msys64\mingw64\bin\g++.exe") else None)
    if not gxx:
        print("g++ not found (install MSYS2 mingw64 or add g++ to PATH)")
        return 2
    env = dict(os.environ)
    env["PATH"] = os.path.dirname(gxx) + os.pathsep + env.get("PATH", "")
    build = os.path.join(HERE, "build")
    os.makedirs(build, exist_ok=True)
    failed = []
    for name, srcs in TESTS.items():
        exe = os.path.join(build, name + (".exe" if os.name == "nt" else ""))
        cmd = [gxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unused-parameter", *INC,
               *[s if os.path.isabs(s) else os.path.join(HERE, s) for s in srcs], "-o", exe]
        print(f"\n##### building {name}")
        if subprocess.run(cmd, env=env).returncode != 0:
            failed.append(name)
            continue
        print(f"##### running {name}")
        if subprocess.run([exe], env=env).returncode != 0:
            failed.append(name)
    print("\n" + ("ALL HOST TEST SUITES PASSED" if not failed else f"FAILED: {', '.join(failed)}"))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
