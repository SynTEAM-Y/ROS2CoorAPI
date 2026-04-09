#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Processor
Strict 3-thread pipeline (REAL T1 collector, T2 classifier, T3 arbiter)
"""

from __future__ import annotations

import json
import time
import threading
import queue
from dataclasses import dataclass
from typing import Any, Deque, Dict, Union, Literal
from collections import deque

from projectn_interfaces.msg import DataMsg, RMsg
from .logging_utils import get_node_logger  # (optional helper; safe if unused)

# Control work shape (opaque to pipeline; TreeNode knows what payload means)
try:
    from .queues import CtrlWork as WorkItem, CtrlKind
except Exception:  # fallback dummy types if import fails
    @dataclass
    class WorkItem:
        kind: Any
        payload: Any
    class CtrlKind:
        GET_MSG_ID = "GET_MSG_ID"
        SUBMIT_DM = "SUBMIT_DM"

# ───────────────────────────────────────────────────────────────────────
# Internal envelope for the unified FIFO
# ───────────────────────────────────────────────────────────────────────
@dataclass
class _InputItem:
    ts: float
    kind: Literal["CTRL", "DM", "R"]
    obj: Union[WorkItem, DataMsg, RMsg]

# ───────────────────────────────────────────────────────────────────
# Compact snapshot helpers (exact format like your example)
# ───────────────────────────────────────────────────────────────────
def _brief_fifo_token(item) -> str:
    """
    item is _InputItem(ts, kind, obj). We print:
      CTRL  → "CTRL"
      DM    → "D"
      R     → "R(id=<id>)"
    """
    try:
        kind = getattr(item, "kind", "?")
        obj  = getattr(item, "obj", item)
        if kind == "CTRL":
            return "CTRL"
        if kind == "DM":
            return "D"
        if kind == "R":
            rid = int(getattr(obj, "msg_id", 0))
            if rid <= 0:
                try:
                    import json as _json
                    d = _json.loads(getattr(obj, "payload_json") or "{}")
                    rid = int(d.get("id", 0))
                except Exception:
                    pass
            return f"R(id={rid if rid>0 else '?'})"
    except Exception:
        pass
    return "?"

def _peek_pq_ids(waiting_pq):
    """Return just the ids from PriorityQueue tuples like (msg_id, seq, DataMsg)."""
    out = []
    try:
        data = list(getattr(waiting_pq, "queue", []))
        for tup in data:
            if isinstance(tup, tuple) and len(tup) >= 1:
                out.append(tup[0])
            else:
                out.append("?")
    except Exception:
        pass
    return out

def _snapshot_node_state(node, stage: str, fifo, waiting_pq, expected_id: int, level="info"):
    """Print: [ A5 ] <stage> | FIFO=[...] | PQ(ids)=[...] | expected=<n>"""
    try:
        log_fn = (getattr(node, "_log_debug", None) if level == "debug" else getattr(node, "_log_info", None))
        if log_fn is None:
            log_fn = (lambda s: node.get_logger().debug(s)) if level == "debug" else (lambda s: node.get_logger().info(s))

        fifo_tokens = []
        for it in list(fifo)[:12]:
            fifo_tokens.append(_brief_fifo_token(it))
        fifo_str = ",".join(fifo_tokens) + (",…" if len(fifo) > 12 else "")

        pq_ids = _peek_pq_ids(waiting_pq)
        pq_str = ",".join(str(i) for i in pq_ids[:18]) + (",…" if len(pq_ids) > 18 else "")

        log_fn(f"[ {node.name} ] {stage} | FIFO=[{fifo_str}] | PQ(ids)=[{pq_str}] | expected={expected_id}")
    except Exception:
        pass

def _rid_from_payload(rm: RMsg) -> int:
    """Best-effort extraction of id from RMsg.payload_json when msg_id is 0."""
    try:
        d = json.loads(getattr(rm, "payload_json") or "{}")
        v = int(d.get("id", 0))
        return v if v > 0 else 0
    except Exception:
        return 0

# ───────────────────────────────────────────────────────────────────────
# Pipeline — 3-thread machine
# ───────────────────────────────────────────────────────────────────────
class Pipeline:
    """
    This class owns the three pipeline threads and queues.
    Lives inside a TreeNode.

    External inputs:
      • ctrl_q: control work from TreeNode services
      • in_q  : incoming DataMsg/RMsg from subscriptions

    Internal queues:
      • _input_q: unified FIFO queue (arrival order, keeps CTRL+R in order)
      • _waiting_pq: priority queue of D, sorted by msg_id

    Threads:
      • T1: merge ctrl_q + in_q → _input_q
      • T2: move DataMsg from _input_q → _waiting_pq
      • T3: deliver D in order, else process FIFO head (CTRL or R)
    """

    # External refs
    node: Any
    ctrl_q: queue.Queue
    in_q: queue.Queue

    # Internal state
    _input_q: Deque[_InputItem]              # unified FIFO (arrival order)
    _waiting_pq: queue.PriorityQueue         # holds (msg_id, seq, DataMsg)
    _pq_seq: int                             # sequence to break ties if same id
    _expected_id: int                        # the next id we want to deliver

    # Synchronization
    _lock: threading.RLock
    _cv: threading.Condition
    _stop: bool

    # Threads
    _t1: threading.Thread
    _t2: threading.Thread
    _t3: threading.Thread

    # Viz/debug
    _last_snapshot: Dict[str, Any]

    def __init__(self, node: Any, ctrl_q: queue.Queue, in_q: queue.Queue):
        self.node = node
        self.ctrl_q = ctrl_q
        self.in_q = in_q
        self._input_q = deque()
        self._waiting_pq = queue.PriorityQueue()
        self._pq_seq = 0
        self._expected_id = 1
        self._lock = threading.RLock()
        self._cv = threading.Condition(self._lock)
        self._stop = False
        self._last_snapshot = {}

        # Start the three pipeline threads
        self._t1 = threading.Thread(target=self._run_t1_collector, name=f"{self.node.name}-T1", daemon=True)
        self._t2 = threading.Thread(target=self._run_t2_classifier, name=f"{self.node.name}-T2", daemon=True)
        self._t3 = threading.Thread(target=self._run_t3_arbiter, name=f"{self.node.name}-T3", daemon=True)

        self._t1.start()
        self._t2.start()
        self._t3.start()

    # ───────────────────────────────────────────────────────────────────
    # Public API
    # ───────────────────────────────────────────────────────────────────
    def enqueue_ctrl(self, work: WorkItem) -> None:
        """Non-blocking: push control work into ctrl_q (later merged by T1)."""
        try:
            self.ctrl_q.put_nowait((_now(), work))
        except queue.Full:
            pass  # drop instead of blocking ROS callbacks

    def enqueue_dm_in(self, msg: Union[DataMsg, RMsg]) -> None:
        """Non-blocking: push incoming D or R into in_q (later merged by T1)."""
        try:
            self.in_q.put_nowait((_now(), msg))
        except queue.Full:
            pass  # drop instead of blocking subscription callbacks

    def set_expected_start(self, start_id: int) -> None:
        """Initialize expected_id (used when parent tells child where to start)."""
        with self._lock:
            if start_id >= 1:
                self._expected_id = int(start_id)

    def snapshot_status(self) -> Dict[str, Any]:
        """Snapshot for visualization/telemetry (input length, waiting length, expected_id)."""
        with self._lock:
            snap = {
                "input_len": len(self._input_q),
                "waiting_len": self._waiting_pq.qsize(),
                "expected_id": self._expected_id,
            }
            self._last_snapshot = snap
            return dict(snap)

    # ───────────────────────────────────────────────────────────────────
    # Thread T1 — Collector
    # ───────────────────────────────────────────────────────────────────
    def _run_t1_collector(self) -> None:
        """
        Continuously pull from ctrl_q and in_q,
        wrap items into _InputItem, and append to _input_q.
        Preserves arrival order by timestamps.
        """
        while not self._stop:
            got_any = False

            # Pull CTRL work
            try:
                ts, work = self.ctrl_q.get(timeout=0.02)
                with self._cv:
                    self._input_q.append(_InputItem(ts=ts, kind="CTRL", obj=work))
                    self._cv.notify_all()
                # INFO: show CTRL entering FIFO
                _snapshot_node_state(self.node, "CTRL in input queue",
                                     self._input_q, self._waiting_pq, self._expected_id)
                got_any = True
            except queue.Empty:
                pass

            # Pull ingress messages (DataMsg or RMsg)
            try:
                ts, msg = self.in_q.get(timeout=0.02)
                kind = "DM" if isinstance(msg, DataMsg) else "R"
                with self._cv:
                    self._input_q.append(_InputItem(ts=ts, kind=kind, obj=msg))
                    self._cv.notify_all()
                # INFO: show D/R entering FIFO
                _stage = "D in input queue" if kind == "DM" else "R in input queue"
                _snapshot_node_state(self.node, _stage,
                                     self._input_q, self._waiting_pq, self._expected_id)
                got_any = True
            except queue.Empty:
                pass

            # If nothing received, backoff briefly
            if not got_any:
                time.sleep(0.005)

    # ───────────────────────────────────────────────────────────────────
    # Thread T2 — Classifier
    # ───────────────────────────────────────────────────────────────────
    def _run_t2_classifier(self) -> None:
        """
        Look at head of _input_q:
        - If DataMsg → remove it and put into _waiting_pq (ordered by id).
        - If CTRL or R → leave in _input_q for T3 to process in FIFO order.
        """
        while not self._stop:
            with self._cv:
                if not self._input_q:
                    self._cv.wait(timeout=0.05)
                    continue
                head = self._input_q[0]

                if head.kind == "DM":
                    # Show that D is at FIFO head and is about to move to PQ
                    _snapshot_node_state(self.node,
                                         "T2 sees D at FIFO head → move to PQ",
                                         self._input_q, self._waiting_pq, self._expected_id)
                    item = self._input_q.popleft()
                else:
                    item = None

            if item is None:
                time.sleep(0.003)
                continue

            dm: DataMsg = item.obj  # should be a DataMsg
            msg_id = int(getattr(dm, "msg_id", 0))
            if msg_id <= 0:
                # Malformed D (missing or invalid id), drop it
                continue

            with self._cv:
                self._waiting_pq.put((msg_id, self._pq_seq, dm))
                self._pq_seq += 1
                self._cv.notify_all()
            _snapshot_node_state(self.node,
                                 f"D(id={msg_id}) → waiting PQ by T2",
                                 self._input_q, self._waiting_pq, self._expected_id)

    # ───────────────────────────────────────────────────────────────────
    # Thread T3 — Arbiter
    # ───────────────────────────────────────────────────────────────────
    def _run_t3_arbiter(self) -> None:
        """
        Strictly enforce ordered delivery:
        1. If head of _waiting_pq has id == expected_id:
            - Deliver it to node._pipeline_handle_data
            - Increment expected_id
        2. Otherwise, process FIFO head if it is CTRL or R.
        """
        while not self._stop:
            did_work = False

            # Step 1: Drain all deliverable D
            while True:
                with self._cv:
                    try:
                        head_tuple = self._waiting_pq.get_nowait()
                    except queue.Empty:
                        head_tuple = None

                if head_tuple is None:
                    break

                head_id, _, head_dm = head_tuple
                if head_id != self._expected_id:
                    # Not the id we expect yet → put it back and stop draining
                    with self._cv:
                        self._waiting_pq.put(head_tuple)
                    break

                dm = head_dm

                # Update dm.path_json with breadcrumbs for viz/debug
                try:
                    try:
                        p = json.loads(dm.path_json) if dm.path_json else {}
                    except Exception:
                        p = {}
                    down = list(p.get("down", []))
                    if not down or down[-1] != self.node.name:
                        down.append(self.node.name)
                    kids = list(self.node.down_pubs.keys()) if hasattr(self.node, "down_pubs") else []
                    p["down"] = down
                    p["to_children"] = kids
                    dm.path_json = json.dumps(p, separators=(",", ":"))
                except Exception:
                    pass

                # Deliver and advance expected_id
                try:
                    # Before delivery (shows PQ still holding head)
                    _snapshot_node_state(
                        self.node,
                        f"T3 ready: D(id={head_id}) == expected",
                        self._input_q, self._waiting_pq, self._expected_id
                    )

                    self.node._pipeline_handle_data(dm)

                    with self._cv:
                        self._expected_id += 1
                        self._cv.notify_all()

                    # After increment (shows PQ/FIFO state post-delivery)
                    _snapshot_node_state(
                        self.node,
                        f"T3 deliver D(id={head_id})",
                        self._input_q, self._waiting_pq, self._expected_id
                    )

                except Exception as e:
                    try:
                        self.node.get_logger().warn(f"{self.node.name}: DATA handler error: {e}")
                    except Exception:
                        pass
                did_work = True

            # Step 2: If no D was delivered, check FIFO head for CTRL or R
            if not did_work:
                with self._cv:
                    if self._input_q and self._input_q[0].kind in ("CTRL", "R"):
                        item = self._input_q.popleft()
                    else:
                        item = None

                if item is not None:
                    try:
                        if item.kind == "CTRL":
                            work: WorkItem = item.obj
                            k = getattr(work, "kind", "?")
                            kname = getattr(k, "name", None) or str(k)
                            _snapshot_node_state(self.node,
                                                 f"T3 process FIFO head (CTRL:{kname})",
                                                 self._input_q, self._waiting_pq, self._expected_id)
                            self.node._pipeline_handle_ctrl(work)
                            _snapshot_node_state(self.node,
                                                 f"T3 done CTRL:{kname}",
                                                 self._input_q, self._waiting_pq, self._expected_id)

                        elif item.kind == "R":
                            rm: RMsg = item.obj
                            rid = int(getattr(rm, "msg_id", 0)) or _rid_from_payload(rm)
                            _snapshot_node_state(self.node,
                                                 f"T3 process FIFO head (R id={rid if rid>0 else '?'})",
                                                 self._input_q, self._waiting_pq, self._expected_id)
                            self.node._pipeline_handle_reply(rm)
                            _snapshot_node_state(self.node,
                                                 f"T3 forwarded R id={rid if rid>0 else '?'}",
                                                 self._input_q, self._waiting_pq, self._expected_id)
                    except Exception as e:
                        try:
                            self.node.get_logger().warn(f"{self.node.name}: handler error: {e}")
                        except Exception:
                            pass
                    did_work = True

            # If nothing to do, sleep shortly to avoid busy loop
            if not did_work:
                time.sleep(0.003)

    # ───────────────────────────────────────────────────────────────────
    # Shutdown
    # ───────────────────────────────────────────────────────────────────
    def stop(self) -> None:
        """Signal all threads to stop."""
        with self._cv:
            self._stop = True
            self._cv.notify_all()

# ───────────────────────────────────────────────────────────────────────
# Helper: current time in monotonic seconds
# ───────────────────────────────────────────────────────────────────────
def _now() -> float:
    return time.monotonic()
