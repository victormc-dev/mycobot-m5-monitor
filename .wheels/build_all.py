# coding=utf-8
"""一键构建 v1.4.0：主程序 onedir → payload.zip → 单文件 Setup + Portable zip。"""
from __future__ import annotations

import hashlib
import os
import subprocess
import sys
import zipfile
from pathlib import Path

WHEELS = Path(__file__).resolve().parent
ROOT = WHEELS.parent
PY = Path(sys.executable)
DIST = WHEELS / "dist"
RELEASE = WHEELS / "release"
VERSION = "1.4.0"
SEP = os.pathsep


def run(cmd, cwd):
    print("RUN", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run([str(c) for c in cmd], cwd=str(cwd), check=True)


def build_main():
    cmd = [
        PY, "-m", "PyInstaller", "--noconfirm", "--clean",
        "--distpath", str(DIST), "--workpath", str(WHEELS / "build"),
        "--specpath", str(WHEELS),
        "--onedir", "--windowed", "--name", "MyCobotMonitor",
        "--paths", str(ROOT / "vendor"),
        "--add-data", str(ROOT / "vendor") + SEP + "vendor",
        "--hidden-import", "pymycobot.mycobot280",
        "--hidden-import", "pymycobot.mycobot320",
        "--hidden-import", "pymycobot.ultraArmP340",
        "--hidden-import", "pymycobot.ultraArm",
        str(ROOT / "joint_monitor.py"),
    ]
    run(cmd, ROOT)


def payload():
    run([PY, str(WHEELS / "make_payload.py")], WHEELS)


def build_setup():
    exe = "MyCobotMonitorSetup-v{}".format(VERSION)
    cmd = [
        PY, "-m", "PyInstaller", "--noconfirm", "--clean",
        "--onefile", "--name", exe,
        "--add-data", str(WHEELS / "payload.zip") + SEP + ".",
        str(WHEELS / "installer.py"),
    ]
    run(cmd, WHEELS)
    return DIST / (exe + ".exe")


def build_portable():
    RELEASE.mkdir(parents=True, exist_ok=True)
    src = DIST / "MyCobotMonitor"
    out = RELEASE / "MyCobotMonitorPortable-v{}.zip".format(VERSION)
    files = sorted(p for p in src.rglob("*") if p.is_file())
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in files:
            z.write(p, "MyCobotMonitor/" + p.relative_to(src).as_posix())
        z.writestr("MyCobotMonitor/PORTABLE_VERSION.txt", VERSION + "\n")
    print("PORTABLE_OK files={} size={}".format(len(files), out.stat().st_size))
    return out


def main():
    build_main()
    payload()
    setup = build_setup()
    portable = build_portable()
    target = RELEASE / setup.name
    target.write_bytes(setup.read_bytes())
    for p in (target, portable):
        print("ARTIFACT {} {} {}".format(
            p.name, p.stat().st_size, hashlib.sha256(p.read_bytes()).hexdigest()))
    print("BUILD_ALL_OK")


if __name__ == "__main__":
    main()
