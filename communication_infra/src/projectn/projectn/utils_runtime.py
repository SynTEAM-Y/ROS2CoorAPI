#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Runtime IO Helper
- Single shared runtime dir: ~/ProjectN/Runtime/
- events.jsonl : append-only line-delimited JSON (all hops + internals)
- state.json   : merged grouped snapshot (nodes, agents, queues, topology)
- Safe cross-process writes via fcntl.flock (POSIX).
"""

from __future__ import annotations
import json, os, time, errno
from pathlib import Path
from typing import Any, Dict, Optional

RUNTIME_BASE = Path(os.path.expanduser(os.environ.get("PROJECTN_DIR", "~/ProjectN"))) / "Runtime"
EVENTS = RUNTIME_BASE / "events.jsonl"
STATE  = RUNTIME_BASE / "state.json"
LOCK   = RUNTIME_BASE / "lock"

try:
    import fcntl
    _HAS_FCNTL = True
except Exception:
    _HAS_FCNTL = False  # Fallback: best-effort (still OK for dev)

def _now() -> float:
    return time.time()

def _ensure_dirs():
    RUNTIME_BASE.mkdir(parents=True, exist_ok=True)

def _compact(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))

def _read_json(path: Path, default: Any) -> Any:
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        pass
    return default

def _deep_merge(dst: Dict, src: Dict) -> Dict:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst

class RuntimeOut:
    """
    Create once per process.
      - fresh=True → root wipes runtime dir on this run
      - node_name is used to group node snapshots in state.json
    """
    def __init__(self, node_name: str, fresh: bool=False):
        self.node = node_name
        _ensure_dirs()
        if fresh:
            # wipe only files we own
            for p in [EVENTS, STATE, LOCK]:
                try:
                    if p.exists(): p.unlink()
                except Exception:
                    pass
            _ensure_dirs()

    # --------------- low-level lock ---------------
    def _locked(self):
        class _L:
            def __init__(self): pass
            def __enter__(self_inner):
                _ensure_dirs()
                self._lock_fd = open(LOCK, "a+")
                if _HAS_FCNTL:
                    fcntl.flock(self._lock_fd.fileno(), fcntl.LOCK_EX)
                return self
            def __exit__(self_inner, exc_type, exc, tb):
                try:
                    if _HAS_FCNTL:
                        fcntl.flock(self._lock_fd.fileno(), fcntl.LOCK_UN)
                    self._lock_fd.close()
                except Exception:
                    pass
        return _L()

    # --------------- public API -------------------
    def event(self, kind: str, **payload):
        """Append a single event line into events.jsonl"""
        rec = {"ts": _now(), "kind": kind, "node": self.node}
        rec.update(payload)
        line = _compact(rec)
        try:
            with self._locked():
                with open(EVENTS, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
        except Exception:
            pass

    def merge_state(self, patch: Dict[str, Any]):
        """Merge a dict patch into state.json (grouped snapshot)."""
        try:
            with self._locked():
                cur = _read_json(STATE, {"run_ts": _now(), "topology": {}, "nodes": {}, "agents": {}})
                _deep_merge(cur, patch)
                STATE.write_text(_compact(cur), encoding="utf-8")
        except Exception:
            pass

    # ----- typed helpers (sugar over merge_state/event) -----
    def set_topology(self, parents: Dict[str,str], root: Optional[str]=None):
        topo = {"parents": parents}
        if root is not None:
            topo["root"] = root
        self.merge_state({"topology": topo})

    def upsert_node(self, node_name: Optional[str], patch: Dict[str, Any]):
        n = node_name or self.node
        self.merge_state({"nodes": {n: patch}})

    def set_expected_id(self, node_name: Optional[str], expected_id: int):
        n = node_name or self.node
        self.upsert_node(n, {"expected_id": int(expected_id)})

    def register_services_topics(self, pubs, subs, srvs):
        self.upsert_node(self.node, {
            "services": list(srvs),
            "topics": {"pub": list(pubs), "sub": list(subs)},
        })

    def register_relations(self, parent: str, children: list[str]):
        self.upsert_node(self.node, {"parent": parent, "children": list(children)})

    def snapshot_queues(self, current: Dict[str, Any], history_item: Optional[Dict[str,Any]]=None):
        patch = {"queues": {"current": current}}
        if history_item:
            patch.setdefault("queues", {}).setdefault("history", []).append(history_item)
        # We cannot "append" directly via merge; emulate append by reading/modifying/writing
        try:
            with self._locked():
                cur = _read_json(STATE, {"run_ts": _now(), "topology": {}, "nodes": {}, "agents": {}})
                node = cur.setdefault("nodes", {}).setdefault(self.node, {})
                q = node.setdefault("queues", {})
                q["current"] = current
                if history_item:
                    q.setdefault("history", []).append(history_item)
                STATE.write_text(_compact(cur), encoding="utf-8")
        except Exception:
            pass

    def register_agent(self, agent: str, node: str):
        self.merge_state({"agents": {agent: {"node": node}}})

    def set_agent_last(self, agent: str, **lasts):
        self.merge_state({"agents": {agent: lasts}})
