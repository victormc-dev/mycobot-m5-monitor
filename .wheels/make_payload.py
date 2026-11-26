# coding=utf-8
"""把 PyInstaller onedir 产物显式打进 payload.zip。

不用 shutil.make_archive：它会漏文件（上次只打了 288/1100，丢了 tk.tcl，
导致安装版 EXE 启动时 TclError）。这里逐文件写入并断言关键资源齐全。
"""
from __future__ import annotations

import hashlib
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "dist" / "MyCobotMonitor"
OUT = ROOT / "payload.zip"

MUST_HAVE = (
    "MyCobotMonitor.exe",
    "_internal/_tk_data/tk.tcl",
    "_internal/_tcl_data/init.tcl",
    "_internal/_tk_data/pkgIndex.tcl",
    "_internal/vendor/pymycobot/ultraArmP340.py",
    "_internal/vendor/pymycobot/mycobot280.py",
    "_internal/vendor/pymycobot/mycobot320.py",
)

DLL_HINTS = ("python3", "dll")


def build() -> Path:
    if not SRC.is_dir():
        raise RuntimeError("缺少 onedir 产物: {}".format(SRC))
    files = sorted(p for p in SRC.rglob("*") if p.is_file())
    if not files:
        raise RuntimeError("产物目录为空")
    rels = {p.relative_to(SRC).as_posix() for p in files}
    missing = [m for m in MUST_HAVE if m not in rels]
    if missing:
        raise RuntimeError("关键资源缺失: {}".format(missing))
    if not any(n.lower().endswith(".dll") and DLL_HINTS[0] in n.lower() for n in rels):
        raise RuntimeError("未找到 python3*.dll")

    if OUT.exists():
        OUT.unlink()
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in files:
            z.write(p, p.relative_to(SRC).as_posix())
        z.writestr("PAYLOAD_VERSION.txt", "1.4.0\n")

    with zipfile.ZipFile(OUT) as z:
        names = set(z.namelist())
        still = [m for m in MUST_HAVE if m not in names]
        if still:
            raise RuntimeError("写入后校验失败: {}".format(still))
        n = len(names)
    digest = hashlib.sha256(OUT.read_bytes()).hexdigest()
    print("PAYLOAD_OK files={} size={} sha256={}".format(n, OUT.stat().st_size, digest))
    return OUT


if __name__ == "__main__":
    build()
