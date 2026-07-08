from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path


def ensure_linkerbot_sdk() -> None:
    try:
        importlib.import_module("linkerbot")
        return
    except ModuleNotFoundError:
        pass

    candidates: list[Path] = []
    env_path = os.environ.get("LINKERBOT_SDK_PATH")
    if env_path:
        root = Path(env_path).expanduser()
        candidates.extend([root, root / "src"])

    cwd = Path.cwd()
    candidates.extend(
        [
            cwd / "vendor" / "linkerbot-python-sdk" / "src",
            cwd / "linkerbot-python-sdk" / "src",
        ]
    )

    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / "vendor" / "linkerbot-python-sdk" / "src")

    for candidate in candidates:
        if (candidate / "linkerbot").is_dir():
            sys.path.insert(0, str(candidate))
            importlib.import_module("linkerbot")
            return

    raise ModuleNotFoundError(
        "Cannot import linkerbot. Install linkerbot-py[kinetix] or set "
        "LINKERBOT_SDK_PATH to the linkerbot-python-sdk checkout."
    )
