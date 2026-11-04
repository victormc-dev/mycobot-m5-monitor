# coding=utf-8
"""依赖引导。

优先级：
1. 已通过 pip / .venv 安装的 pymycobot；
2. 否则回退到项目内的离线 vendor/ 目录（wheel 已解包，可直接 import）。

所有示例只需在 import pymycobot 之前 `import bootstrap` 即可。
"""

import sys
from pathlib import Path

VENDOR = Path(__file__).resolve().parent / "vendor"


def _ensure() -> str:
    try:
        import pymycobot  # noqa: F401
        return "site-packages / .venv"
    except ImportError:
        pass

    if not VENDOR.is_dir():
        raise RuntimeError(
            "pymycobot 未安装，且离线目录 vendor/ 不存在。请运行 setup.ps1。"
        )

    sys.path.insert(0, str(VENDOR))
    import pymycobot  # noqa: F401
    return "vendor/ (offline)"


SOURCE = _ensure()

__all__ = ["SOURCE", "VENDOR"]
