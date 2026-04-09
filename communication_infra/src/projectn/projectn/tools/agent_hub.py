#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Single-Agent Hub (Q / R / D) with grouped seq view + remote control

WHAT'S NEW vs your current hub
──────────────────────────────────────────────────────────────────────────────
1) Supervisor control:
   - Subscribes to /agent_hub/control (RosString JSON).
   - Filters by agent/node, then runs: ping | q | d | send | seq.
   - Long ops run in a thread so ROS callbacks remain responsive.

2) Instant summaries back to supervisor:
   - Publishes to /agent_hub/summary on demand (after commands or ping).
   - Keeps your existing viz heartbeat (/agent_hub/viz/status) untouched.

Nothing else changes: each agent stays in its own terminal and you keep all logs.
"""
from __future__ import annotations

# ───────────────────────────────────────────────────────────────────────
# Stdlib
# ───────────────────────────────────────────────────────────────────────
import argparse
import json
import os
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, List, Optional

# ───────────────────────────────────────────────────────────────────────
# ROS 2
# ───────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

# ProjectN interfaces
from projectn_interfaces.srv import GetMsgId, SubmitDm
from projectn_interfaces.msg import DataMsg, RMsg
from std_msgs.msg import String as RosString

# ───────────────────────────────────────────────────────────────────────
# Emojis / logger helpers (pretty logs; safe fallback if helper missing)
# ───────────────────────────────────────────────────────────────────────
try:
    from ..utils_emojis import elog, TAG  # type: ignore
except Exception:  # safe fallback
    def elog(*args, **kwargs): return " ".join(str(a) for a in args)
    class TAG:
        OK="✅"; WARN="⚠️"
        Q_UP="🟦"; R_IN="🟨"; D_UP="⬆️"; D_IN="📥"
        AG="🤖"

# Aliases so the rest of the file has short names and still works with fallback
T_OK  = getattr(TAG, 'OK',  '✅')
T_WRN = getattr(TAG, 'WARN','⚠️')
E_Q   = getattr(TAG, 'Q_UP','🟦')
E_R   = getattr(TAG, 'R_IN','🟨')
E_DU  = getattr(TAG, 'D_UP','⬆️')
E_DI  = getattr(TAG, 'D_IN','📥')
E_AG  = getattr(TAG, 'AG',  '🤖')

# ───────────────────────────────────────────────────────────────────────
# Small utils
# ───────────────────────────────────────────────────────────────────────
def now() -> float:
    """Wall-clock seconds (used for user-friendly timestamps in logs)."""
    return time.time()

def jparse(s: str) -> dict:
    """Safe JSON parser that always returns a dict (or {})."""
    try:
        v = json.loads(s) if s else {}
        return v if isinstance(v, dict) else {}
    except Exception:
        return {}

def jdump(obj: dict, kind: str = "") -> str:
    """
    Canonical JSON for terminal: keep 'id' before 'dest' for Q/R/D prints.
    This is just for pretty/consistent printing in logs and REPL.
    """
    if not isinstance(obj, dict):
        return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
    order_map = {
        "Q": ["Q", "id", "dest", "route", "msg", "src"],
        "R": ["R", "id", "dest", "route"],
        "D": ["D", "id", "dest", "src", "msg"],
    }
    order = order_map.get(kind, [])
    if order:
        out = {k: obj[k] for k in order if k in obj}
        for k in sorted(obj.keys()):
            if k not in out:
                out[k] = obj[k]
    else:
        out = obj
    return json.dumps(out, ensure_ascii=False, separators=(",", ":"))

def brief_payload(payload_json: str) -> str:
    """Make a short human-readable summary of the D payload for feeds."""
    try:
        v = json.loads(payload_json) if payload_json else {}
        if isinstance(v, dict) and "msg" in v and not isinstance(v["msg"], (dict, list)):
            s = str(v["msg"])
        else:
            s = json.dumps(v, ensure_ascii=False, separators=(",", ":"))
        return (s[:80] + "…") if len(s) > 80 else s
    except Exception:
        s = (payload_json or "")[:80]
        return s + ("…" if len(s) == 80 else "")

# ───────────────────────────────────────────────────────────────────────
# Hub
# ───────────────────────────────────────────────────────────────────────
class SingleAgentHub(Node):
    """
    One hub == one agent attached to one node.

    Guarantees:
      • Q waits for the R id indefinitely (synchronizes via an Event).
      • D is only allowed after a real id was received.
      • 'seq' prints grouped events (Q/R/D sent / D received) with per-agent order.
      • NEW: accepts remote control commands and emits on-demand summaries.
    """

    def __init__(self, args):
        super().__init__('projectn_agent_hub')

        # ── Config from CLI ─────────────────────────────────────────────
        self.agent: str = args.agent.strip()
        self.node_name: str = args.node.strip()
        if not self.agent or not self.node_name:
            raise RuntimeError("--agent and --node are required")

        # ── Files / directories (minimal persistence for later dashboards) ─
        base = os.path.expanduser(os.environ.get("PROJECTN_DIR", "~/ProjectN"))
        self._root_dir = Path(base) / "AgentHub"
        self._root_dir.mkdir(parents=True, exist_ok=True)
        self._seq_file = self._root_dir / f"sequences_{self.agent}.json"
        self._feeds_file = self._root_dir / f"feeds_{self.agent}.json"

        # ── In-memory logs ──────────────────────────────────────────────
        self._timeline: List[dict] = []
        self._logs: Dict[str, List[dict]] = {"Q": [], "R": [], "D_sent": [], "D_recv": []}
        self._feeds: deque = deque(maxlen=200)
        self._seq_counter: int = 0

        # ── Reply synchronization (Q→R) ─────────────────────────────────
        self._rid_event = threading.Event()
        self._last_rid: int = 0
        self._sync_lock = threading.Lock()

        # ── Agent lifecycle pubs + topic subscriptions ──────────────────
        self._attach_pub = self.create_publisher(RosString, f'/{self.node_name}/agents/attach', 10)
        self._detach_pub = self.create_publisher(RosString, f'/{self.node_name}/agents/detach', 10)

        self._agent_inbox_sub = self.create_subscription(
            DataMsg, f'/{self.node_name}/agents/{self.agent}/inbox',
            self._on_agent_inbox_cb, 50
        )
        self._agent_reply_sub = self.create_subscription(
            RMsg, f'/{self.node_name}/agents/{self.agent}/reply',
            self._on_agent_reply_cb, 50
        )

        # ── Service clients (to the node) ───────────────────────────────
        self._get_cli = self.create_client(GetMsgId, f'/{self.node_name}/get_msg_id')
        self._submit_cli = self.create_client(SubmitDm, f'/{self.node_name}/submit_dm')

        # ── Attach heartbeat (startup safety) ───────────────────────────
        self._attach_beats_left = 10
        self._attach_timer = self.create_timer(1.0, self._attach_heartbeat)

        # ── Optional viz heartbeat (for dashboards) ─────────────────────
        self._viz_pub = self.create_publisher(RosString, '/agent_hub/viz/status', 10)
        self._viz_timer = self.create_timer(1.0, self._viz_tick)

        # ── NEW: Remote control + on-demand summary ─────────────────────
        self._ctrl_sub = self.create_subscription(
            RosString, '/agent_hub/control', self._on_control_cmd, 10
        )
        self._summary_pub = self.create_publisher(RosString, '/agent_hub/summary', 10)

        # ── Startup log + REPL thread ───────────────────────────────────
        self.get_logger().info(elog(T_OK, f"{E_AG} {self.agent} @@ {self.node_name} — Single-Agent Hub ready"))
        self._repl_thread = threading.Thread(target=self._run_repl, name="SingleAgentHub-REPL", daemon=True)
        self._repl_thread.start()

    # ───────────────────────────────────────────────────────────────────
    # REPL (interactive shell)
    # ───────────────────────────────────────────────────────────────────
    def _help(self):
        print(f"""
Single-Agent Hub REPL — {self.agent} @@ {self.node_name}
  q                 # send Q and WAIT (no timeout) for R (stores id)
  d <text...>       # send D using the stored id (refuses if none)
  send <text...>    # Q → wait R → D (one shot convenience)
  seq               # grouped view (Q / R / D sent / D rec)
  quit
""".strip())

    def _run_repl(self):
        self._help()
        while rclpy.ok():
            try:
                sys.stdout.write("hub> "); sys.stdout.flush()
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.1); continue
                parts = line.strip().split()
                if not parts:
                    continue
                cmd = parts[0].lower()
                if cmd == "quit":
                    rclpy.shutdown(); break
                elif cmd == "q":
                    self._do_q_wait_r()
                elif cmd == "d" and len(parts) >= 2:
                    text = " ".join(parts[1:])
                    self._do_d_only(text)
                elif cmd == "send" and len(parts) >= 2:
                    text = " ".join(parts[1:])
                    self._do_send(text)
                elif cmd == "seq":
                    self._print_grouped()
                else:
                    self._help()
            except Exception as e:
                print(f"[REPL ERROR] {e}", file=sys.stderr)

    # ───────────────────────────────────────────────────────────────────
    # Attach heartbeat
    # ───────────────────────────────────────────────────────────────────
    def _attach_heartbeat(self):
        try:
            if self._attach_beats_left <= 0:
                self._attach_timer.cancel(); return
            self._attach_beats_left -= 1
            self._attach_pub.publish(RosString(data=self.agent))
        except Exception:
            pass

    # ───────────────────────────────────────────────────────────────────
    # Grouped printer (per-group exact occurrence order)
    # ───────────────────────────────────────────────────────────────────
    def _print_grouped(self):
        def dump(label, items, kind):
            print(label + ":")
            if not items:
                print("  (none)"); return
            items_sorted = sorted(items, key=lambda it: it.get("seq", 0))
            for it in items_sorted:
                ts = time.strftime("%H:%M:%S", time.localtime(it.get("ts", now())))
                print(f"  {ts} {it.get('emoji','')} {jdump(it.get('json'), kind)}")
        dump("Q", self._logs["Q"], "Q")
        dump("R", self._logs["R"], "R")
        dump("D sent", self._logs["D_sent"], "D")
        dump("D rec", self._logs["D_recv"], "D")
        print()

    # ───────────────────────────────────────────────────────────────────
    # Downlink callbacks (messages arriving to the agent)
    # ───────────────────────────────────────────────────────────────────
    def _on_agent_inbox_cb(self, msg: DataMsg):
        try:
            pid = int(getattr(msg, "msg_id", 0))
            payload_json = getattr(msg, "payload_json", "") or ""
            d = jparse(payload_json)
            try:
                self._feeds.append({"id": pid, "brief": brief_payload(payload_json), "ts": now()})
            except Exception:
                pass
            ev = {"type":"D_recv","id":pid,"ts":now(),"seq": self._next_seq(),"emoji":E_DI,"json": d}
            self._timeline.append(ev)
            self._logs["D_recv"].append(ev)
            self.get_logger().info(elog(E_DI, f"{self.agent}@{self.node_name}: D in {jdump(d,'D')}"))
        except Exception:
            pass

    def _on_agent_reply_cb(self, msg: RMsg):
        try:
            rid = int(getattr(msg, "msg_id", 0))
            payload_json = getattr(msg, "payload_json", "") or ""
            r = jparse(payload_json)
            ev = {"type":"R","id":rid,"ts":now(),"seq": self._next_seq(),"emoji":E_R,"json": r}
            self._timeline.append(ev)
            self._logs["R"].append(ev)
            with self._sync_lock:
                self._last_rid = rid
                self._rid_event.set()
            self.get_logger().info(elog(E_R, f"{self.agent}@{self.node_name}: R in {jdump(r,'R')}"))
        except Exception as e:
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: reply cb failed: {e}"))

    # ───────────────────────────────────────────────────────────────────
    # Q / R / D flows (core commands)
    # ───────────────────────────────────────────────────────────────────
    def _do_q_wait_r(self) -> Optional[int]:
        q = {"Q": True, "id": 0, "route": [self.agent], "dest": self.node_name}
        q_ev = {"type":"Q","id":"-","ts":now(),"seq": self._next_seq(),"emoji":E_Q,"json": q}
        self._timeline.append(q_ev)
        self._logs["Q"].append(q_ev)
        self.get_logger().info(elog(E_Q, f"{self.agent}@{self.node_name}: Q out {jdump(q,'Q')}"))

        if not self._get_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: get_msg_id unavailable"))
            return None

        with self._sync_lock:
            self._last_rid = 0
            self._rid_event.clear()

        req = GetMsgId.Request()
        req.agent_name = self.agent
        req.path_json = jdump(q, "Q")
        req.timeout_ms_remaining = 60000
        _ = self._get_cli.call_async(req)

        while rclpy.ok() and not self._rid_event.wait(timeout=0.1):
            pass

        with self._sync_lock:
            rid = int(self._last_rid)
        if rid <= 0:
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: R did not provide a valid id"))
            return None
        return rid

    def _do_d_only(self, text: Any) -> bool:
        with self._sync_lock:
            msg_id = int(self._last_rid)
        if msg_id <= 0:
            print("No R id stored yet. Run 'q' or 'send' first.")
            return False

        dup = {"D": True, "id": msg_id, "src": self.agent, "dest": self.node_name, "msg": text}
        ev_submit = {"type":"D_sent","id":msg_id,"ts":now(),"seq": self._next_seq(),"emoji":E_DU,"json":dup}
        self._timeline.append(ev_submit)
        self._logs["D_sent"].append(ev_submit)
        self.get_logger().info(elog(E_DU, f"{self.agent}@{self.node_name}: D out {jdump(dup,'D')}"))

        if not self._submit_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: submit_dm unavailable"))
            return False

        req = SubmitDm.Request()
        req.msg_id = msg_id
        req.src_agent = self.agent
        req.payload_json = jdump(dup, "D")
        fut = self._submit_cli.call_async(req)
        self._poll(fut, 10.0)
        return True

    def _do_send(self, text: Any) -> bool:
        rid = self._do_q_wait_r()
        if not rid:
            return False
        return self._do_d_only(text)

    # ───────────────────────────────────────────────────────────────────
    # NEW: Supervisor control handler
    # ───────────────────────────────────────────────────────────────────
    def _on_control_cmd(self, msg: RosString):
        """
        Handle supervisor control JSON.
        Schema (all fields optional):
          {
            "cmd": "ping|q|d|send|seq",
            "text": "payload for d/send",
            "agents": ["A1","A2"],      # match list; empty/absent = any
            "nodes":  ["Root","A1"],    # match list; empty/absent = any
            "autoq":  true               # if 'd' and no id yet, do q first
          }
        """
        try:
            cfg = jparse(msg.data)
            agents = set(cfg.get("agents") or [])
            nodes  = set(cfg.get("nodes") or [])
            if agents and self.agent not in agents:
                return
            if nodes and self.node_name not in nodes:
                return

            cmd  = (cfg.get("cmd") or "").lower()
            text = cfg.get("text")

            def do_summary():
                payload = {
                    "agent": self.agent,
                    "node": self.node_name,
                    "last_rid": int(self._last_rid),
                    "seq_counts": {k: len(v) for k, v in self._logs.items()},
                    "ts": now(),
                }
                self._summary_pub.publish(RosString(data=json.dumps(payload, separators=(",", ":"))))

            # Run long ops in a thread to avoid blocking callbacks
            def run():
                if cmd == "ping":
                    do_summary()
                elif cmd == "seq":
                    self._print_grouped()
                    do_summary()
                elif cmd == "q":
                    self._do_q_wait_r()
                    do_summary()
                elif cmd == "d":
                    if int(self._last_rid) <= 0 and cfg.get("autoq"):
                        if not self._do_q_wait_r():
                            do_summary(); return
                    self._do_d_only(text)
                    do_summary()
                elif cmd == "send":
                    self._do_send(text)
                    do_summary()

            threading.Thread(target=run, daemon=True).start()
        except Exception as e:
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: control failed: {e}"))

    # ───────────────────────────────────────────────────────────────────
    # Misc helpers
    # ───────────────────────────────────────────────────────────────────
    def _next_seq(self) -> int:
        self._seq_counter += 1
        return self._seq_counter

    def _poll(self, fut: Future, timeout_s: float) -> bool:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and rclpy.ok():
            if fut.done():
                return True
            time.sleep(0.01)
        return fut.done()

    def _viz_tick(self):
        try:
            payload = {
                "agent": self.agent,
                "node": self.node_name,
                "seq_counts": {k: len(v) for k, v in self._logs.items()},
                "ts": now(),
            }
            self._viz_pub.publish(RosString(data=json.dumps(payload, separators=(",", ":"))))
        except Exception:
            pass

# ───────────────────────────────────────────────────────────────────────
# CLI + main
# ───────────────────────────────────────────────────────────────────────
def parse_args(argv: List[str]):
    p = argparse.ArgumentParser(description="ProjectN Single-Agent Hub (+control)")
    p.add_argument("--agent", required=True, help="agent name to attach")
    p.add_argument("--node", required=True, help="node name to attach the agent to")
    return p.parse_args(argv)

def main(argv: Optional[List[str]] = None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    hub = SingleAgentHub(args)
    exec = MultiThreadedExecutor()
    exec.add_node(hub)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try: exec.shutdown()
        except Exception: pass
        if rclpy.ok():
            try: rclpy.shutdown()
            except Exception: pass

if __name__ == "__main__":
    main()
