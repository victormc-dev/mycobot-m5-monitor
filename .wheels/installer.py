# coding=utf-8
"""MyCobot Monitor 安装器（PyInstaller onefile）。

用法：
    Setup.exe                 图形向导
    Setup.exe --silent        静默安装
    Setup.exe --uninstall     卸载（先复制自身到 TEMP 再执行，避免占用）
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import winreg
import zipfile
from pathlib import Path

VERSION = "1.4.0"
APP = "MyCobotMonitor"
DISPLAY = "MyCobot Monitor {} (280/320/P340)".format(VERSION)
LOCAL = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local"))
APPDATA = Path(os.environ.get("APPDATA", Path.home() / "AppData" / "Roaming"))
INSTALL_DIR = LOCAL / APP
EXE_NAME = "MyCobotMonitor.exe"
UNINSTALL_KEY = r"Software\Microsoft\Windows\CurrentVersion\Uninstall\MyCobotMonitor"
START_DIR = APPDATA / "Microsoft" / "Windows" / "Start Menu" / "Programs" / "MyCobot Monitor"
DESKTOP = Path(os.environ.get("USERPROFILE", Path.home())) / "Desktop"
CREATE_NO_WINDOW = 0x08000000


def payload_path() -> Path:
    if getattr(sys, "frozen", False):
        return Path(getattr(sys, "_MEIPASS")) / "payload.zip"
    return Path(__file__).resolve().parent / "payload.zip"


def say(text: str) -> None:
    try:
        print(text, flush=True)
    except Exception:
        pass


def _shortcut(lnk: Path, target: Path) -> None:
    lnk.parent.mkdir(parents=True, exist_ok=True)
    ps = (
        "$ErrorActionPreference='Stop';"
        "$ws=New-Object -ComObject WScript.Shell;"
        "$sc=$ws.CreateShortcut('{}');"
        "$sc.TargetPath='{}';"
        "$sc.WorkingDirectory='{}';"
        "$sc.IconLocation='{},0';"
        "$sc.Save()"
    ).format(str(lnk), str(target), str(target.parent), str(target))
    res = subprocess.run(
        ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
        capture_output=True, text=True, creationflags=CREATE_NO_WINDOW,
    )
    if res.returncode != 0:
        raise RuntimeError("快捷方式创建失败: {}".format(res.stderr.strip()[:200]))


def _write_reg(exe: Path) -> None:
    with winreg.CreateKey(winreg.HKEY_CURRENT_USER, UNINSTALL_KEY) as k:
        winreg.SetValueEx(k, "DisplayName", 0, winreg.REG_SZ, DISPLAY)
        winreg.SetValueEx(k, "DisplayVersion", 0, winreg.REG_SZ, VERSION)
        winreg.SetValueEx(k, "Publisher", 0, winreg.REG_SZ, "victormc-dev")
        winreg.SetValueEx(k, "InstallLocation", 0, winreg.REG_SZ, str(INSTALL_DIR))
        winreg.SetValueEx(k, "UninstallString", 0, winreg.REG_SZ,
                          '"{}" --uninstall'.format(sys.executable))
        winreg.SetValueEx(k, "QuietUninstallString", 0, winreg.REG_SZ,
                          '"{}" --uninstall'.format(sys.executable))
        winreg.SetValueEx(k, "NoModify", 0, winreg.REG_DWORD, 1)
        winreg.SetValueEx(k, "NoRepair", 0, winreg.REG_DWORD, 1)


def install(desktop: bool = True) -> int:
    zpath = payload_path()
    if not zpath.is_file():
        raise RuntimeError("安装载荷缺失: {}".format(zpath))
    INSTALL_DIR.mkdir(parents=True, exist_ok=True)
    # 覆盖式升级：只写载荷内的文件，不删除用户可能保存的其他文件
    with zipfile.ZipFile(zpath) as z:
        names = [n for n in z.namelist() if not n.endswith("/") and n != "PAYLOAD_VERSION.txt"]
        for name in names:
            dest = INSTALL_DIR / Path(name.replace("/", os.sep))
            dest.parent.mkdir(parents=True, exist_ok=True)
            with z.open(name) as src, open(dest, "wb") as out:
                shutil.copyfileobj(src, out)
    exe = INSTALL_DIR / EXE_NAME
    if not exe.is_file():
        raise RuntimeError("安装后未找到 {}".format(exe))
    _shortcut(START_DIR / "MyCobot Monitor.lnk", exe)
    if desktop:
        _shortcut(DESKTOP / "MyCobot Monitor.lnk", exe)
    _write_reg(exe)
    say("INSTALL_OK dir={} files={}".format(INSTALL_DIR, len(names)))
    return 0


def _remove_reg() -> None:
    try:
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, UNINSTALL_KEY):
            pass
        winreg.DeleteKey(winreg.HKEY_CURRENT_USER, UNINSTALL_KEY)
    except OSError:
        pass


def uninstall_impl() -> int:
    _remove_reg()
    for lnk in (START_DIR / "MyCobot Monitor.lnk", DESKTOP / "MyCobot Monitor.lnk"):
        try:
            lnk.unlink()
        except OSError:
            pass
    zpath = payload_path()
    if zpath.is_file():
        with zipfile.ZipFile(zpath) as z:
            names = [n for n in z.namelist() if not n.endswith("/") and n != "PAYLOAD_VERSION.txt"]
        for name in names:
            try:
                (INSTALL_DIR / Path(name.replace("/", os.sep))).unlink()
            except OSError:
                pass
        # 只删空目录，绝不递归删除其他内容
        dirs = sorted({(INSTALL_DIR / Path(n.replace("/", os.sep))).parent for n in names},
                      key=lambda p: len(p.parts), reverse=True)
        for d in dirs:
            try:
                d.rmdir()
            except OSError:
                pass
    say("UNINSTALL_OK")
    return 0


def uninstall() -> int:
    if not getattr(sys, "frozen", False):
        return uninstall_impl()
    tmp = Path(tempfile.gettempdir()) / "MyCobotMonitorUninstall" / EXE_NAME
    tmp.parent.mkdir(parents=True, exist_ok=True)
    try:
        shutil.copy2(sys.executable, tmp)
    except Exception as exc:
        raise RuntimeError("无法复制到 TEMP: {}".format(exc))
    # onefile 的 payload 会被带过去；自删用延迟命令
    subprocess.Popen(
        'cmd /c (ping -n 3 127.0.0.1 >nul & del /q "{}")'.format(tmp),
        shell=True, creationflags=CREATE_NO_WINDOW,
    )
    res = subprocess.run([str(tmp), "--uninstall-impl"],
                         capture_output=True, text=True, creationflags=CREATE_NO_WINDOW)
    say(res.stdout.strip())
    if res.returncode != 0:
        say(res.stderr.strip())
    return res.returncode


def _gui() -> int:
    import tkinter as tk
    from tkinter import messagebox

    root = tk.Tk()
    root.title("MyCobot Monitor {} 安装".format(VERSION))
    root.resizable(False, False)
    tk.Label(root, text="MyCobot Monitor {}".format(VERSION),
             font=("Segoe UI", 16, "bold")).pack(padx=24, pady=(20, 4), anchor="w")
    tk.Label(root, text="支持 myCobot 280 M5 / 320 M5 / ultraArm P340",
             font=("Segoe UI", 10)).pack(padx=24, anchor="w")
    tk.Label(root, text="安装位置：", font=("Segoe UI", 10, "bold")).pack(
        padx=24, pady=(18, 2), anchor="w")
    tk.Label(root, text=str(INSTALL_DIR), font=("Consolas", 9),
             fg="#475569").pack(padx=24, anchor="w")
    desk = tk.BooleanVar(value=True)
    tk.Checkbutton(root, text="创建桌面快捷方式", variable=desk,
                   font=("Segoe UI", 10)).pack(padx=24, pady=10, anchor="w")

    def do_install():
        btn.config(state="disabled", text="安装中…")
        root.update()
        try:
            install(bool(desk.get()))
            messagebox.showinfo("完成", "MyCobot Monitor {} 安装完成。".format(VERSION))
            root.destroy()
        except Exception as exc:
            messagebox.showerror("安装失败", str(exc))
            btn.config(state="normal", text="安装")

    btn = tk.Button(root, text="安装", width=14, command=do_install,
                    bg="#2563EB", fg="white", relief="flat")
    btn.pack(padx=24, pady=(4, 20), anchor="w")
    root.mainloop()
    return 0


def main() -> int:
    args = set(sys.argv[1:])
    if "--uninstall-impl" in args:
        return uninstall_impl()
    if "--uninstall" in args:
        return uninstall()
    if "--silent" in args:
        return install(True)
    return _gui()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        say("SETUP_ERROR {}: {}".format(type(exc).__name__, exc))
        sys.exit(1)
