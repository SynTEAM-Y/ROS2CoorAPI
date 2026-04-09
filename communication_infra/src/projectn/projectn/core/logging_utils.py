# projectn/core/logging_utils.py
import os, sys, logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

PROJECTN_LOG_DIR = Path(os.environ.get("PROJECTN_LOG_DIR", str(Path.home() / "ProjectN" / "logs")))
PROJECTN_LOG_DIR.mkdir(parents=True, exist_ok=True)

_cache = {}

def get_node_logger(node_name: str, level=logging.INFO, to_console=True):
    """
    Logger named 'projectn.<node_name>' with:
      • Rotating file  ~/ProjectN/logs/<node_name>.log
      • Optional console stream
    """
    key = f"projectn.{node_name}"
    if key in _cache:
        return _cache[key]

    log = logging.getLogger(key)
    log.setLevel(level)
    log.propagate = False

    fh = RotatingFileHandler(PROJECTN_LOG_DIR / f"{node_name}.log",
                             maxBytes=2_000_000, backupCount=3, encoding="utf-8")
    ffmt = logging.Formatter("%(asctime)s.%(msecs)03d %(levelname)s [%(name)s] %(message)s", datefmt="%H:%M:%S")
    fh.setFormatter(ffmt)
    log.addHandler(fh)

    if to_console:
        ch = logging.StreamHandler(sys.stdout)
        cfmt = logging.Formatter("%(asctime)s %(levelname)s [%(name)s] %(message)s", datefmt="%H:%M:%S")
        ch.setFormatter(cfmt)
        log.addHandler(ch)

    _cache[key] = log
    return log
