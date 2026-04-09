#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Single-Agent Hub (Q / R / D) with grouped seq view

INTRODUCTION 
──────────────────────────────────────────────────────────────────────────────
This program is a *tiny client hub* for exactly ONE agent connected to exactly
ONE ProjectN tree node. You run one process per agent.

What it does:
  • Attaches the agent to a given node (via attach/detach topics).
  • Provides a REPL (interactive prompt) with commands:
      - q           : send a Q (ask for id) and wait indefinitely for the R
      - d <text>    : send a D (data) using the last received id (refuses if none)
      - send <text> : convenience = q → wait R → d
      - seq         : print grouped logs for Q / R / D sent / D received
      - quit        : exit
  • Listens to the agent’s inbox (DataMsg) and reply (RMsg) topics.
  • Logs events with a per-agent sequence counter, so prints keep exact order.
  • Optionally publishes a small viz heartbeat (counts of events).

Important behavior:
  • Q waits for R *with no timeout* (it just blocks until R arrives by topic).
  • D is only allowed after an R id was actually received and stored.
  • This hub does not implement ordering; ordering is enforced by the TreeNode’s
    pipeline. The hub just shows what it sent/received.

Typical usage:
  Open multiple terminals and run this script with different --agent / --node
  to simulate multiple independent agents.
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
from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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
    from ..utils_emojis import elog, TAG
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
        # append any extra fields in sorted order for stability
        for k in sorted(obj.keys()):
            if k not in out:
                out[k] = obj[k]
    else:
        out = obj
    return json.dumps(out, ensure_ascii=False, separators=(",", ":"))

def brief_payload(payload_json: str) -> str:
    """
    Create a short human-readable summary of the D payload for the feed view.
    If payload has a simple 'msg' (string/number), show it; otherwise compact JSON.
    Truncate to ~80 chars with an ellipsis.
    """
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
      • Printing 'seq' groups events (Q/R/D sent/D received) and preserves the
        exact occurrence order within each group using a per-agent sequence counter.
    """

    def __init__(self, args):
        # ROS 2 node name for this hub
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
        self._timeline: List[dict] = []       # full chronological list if needed later
        self._logs: Dict[str, List[dict]] = { # grouped logs for REPL 'seq'
            "Q": [], "R": [], "D_sent": [], "D_recv": []
        }
        self._feeds: deque = deque(maxlen=200)  # recent short messages for quick view

        # Per-agent sequence counter (monotone) to preserve within-group order
        self._seq_counter: int = 0

        # ── Reply synchronization (Q→R) ─────────────────────────────────
        self._rid_event = threading.Event()   # set when an R with id arrives
        self._last_rid: int = 0               # last granted id
        self._sync_lock = threading.Lock()    # protects _last_rid + event

        # ── Agent lifecycle pubs + topic subscriptions ──────────────────
        # Attach/detach notifies the node we exist (the node will create per-agent pubs)
        self._attach_pub = self.create_publisher(RosString, f'/{self.node_name}/agents/attach', 10)
        self._detach_pub = self.create_publisher(RosString, f'/{self.node_name}/agents/detach', 10)

        # Inbox (DataMsg downlink to the agent from its node)
        self._agent_inbox_sub = self.create_subscription(
            DataMsg, f'/{self.node_name}/agents/{self.agent}/inbox',
            self._on_agent_inbox_cb, 50
        )
        # Reply (RMsg routed back down to the agent)
        self._agent_reply_sub = self.create_subscription(
            RMsg, f'/{self.node_name}/agents/{self.agent}/reply',
            self._on_agent_reply_cb, 50
        )

        # ── Service clients (to the node) ───────────────────────────────
        self._get_cli = self.create_client(GetMsgId, f'/{self.node_name}/get_msg_id')
        self._submit_cli = self.create_client(SubmitDm, f'/{self.node_name}/submit_dm')

        # ── Attach heartbeat (startup safety) ───────────────────────────
        # Publish our agent name a few times so the node reliably sees us
        self._attach_beats_left = 10
        self._attach_timer = self.create_timer(1.0, self._attach_heartbeat)

        # ── Optional viz heartbeat (for dashboards) ─────────────────────
        self._viz_pub = self.create_publisher(RosString, '/agent_hub/viz/status', 10)
        self._viz_timer = self.create_timer(1.0, self._viz_tick)

        # ── Startup log + REPL thread ───────────────────────────────────
        self.get_logger().info(elog(T_OK, f"{E_AG} {self.agent} @@ {self.node_name} — Single-Agent Hub ready"))
        self._repl_thread = threading.Thread(target=self._run_repl, name="SingleAgentHub-REPL", daemon=True)
        self._repl_thread.start()

    # ───────────────────────────────────────────────────────────────────
    # REPL (interactive shell)
    # ───────────────────────────────────────────────────────────────────
    def _help(self):
        """Print available commands and brief descriptions."""
        print(f"""
Single-Agent Hub REPL — {self.agent} @@ {self.node_name}
  q                 # send Q and WAIT (no timeout) for R (stores id)
  d <text...>       # send D using the stored id (refuses if none)
  send <text...>    # Q → wait R → D (one shot convenience)
  seq               # grouped view (Q / R / D sent / D rec)
  quit
""".strip())

    def _run_repl(self):
        """
        Interactive loop reading commands from stdin.
        Uses small sleeps so Ctrl+C (KeyboardInterrupt) and ROS shutdown work smoothly.
        """
        self._help()
        while rclpy.ok():
            try:
                sys.stdout.write("hub> ")
                sys.stdout.flush()
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.1); continue
                parts = line.strip().split()
                if not parts:
                    continue

                cmd = parts[0].lower()
                if cmd == "quit":
                    rclpy.shutdown()
                    break

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
        """
        On startup, publish our agent name a few times so the node picks us up
        and creates the per-agent topics. Then stop the timer.
        """
        try:
            if self._attach_beats_left <= 0:
                self._attach_timer.cancel()
                return
            self._attach_beats_left -= 1
            self._attach_pub.publish(RosString(data=self.agent))
        except Exception:
            pass

    # ───────────────────────────────────────────────────────────────────
    # Grouped printer (per-group exact occurrence order)
    # ───────────────────────────────────────────────────────────────────
    def _print_grouped(self):
        """Pretty print logs grouped by kind, ordered by the per-agent seq field."""
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
        """
        Called when a DataMsg arrives to the agent’s inbox.
        We log it, append a short “feed” item, and print a pretty line.
        """
        try:
            pid = int(getattr(msg, "msg_id", 0))
            payload_json = getattr(msg, "payload_json", "") or ""
            d = jparse(payload_json)

            # Feed item (short summary for UI)
            try:
                self._feeds.append({"id": pid, "brief": brief_payload(payload_json), "ts": now()})
            except Exception:
                pass

            ev = {"type":"D_recv","id":pid,"ts":now(),"seq": self._next_seq(),"emoji":E_DI,"json": d}
            self._timeline.append(ev)
            self._logs["D_recv"].append(ev)
            self.get_logger().info(elog(E_DI, f"{self.agent}@{self.node_name}: D in {jdump(d,'D')}"))
        except Exception:
            # Swallow to avoid crashing the subscription thread on malformed messages
            pass

    def _on_agent_reply_cb(self, msg: RMsg):
        """
        Called when an RMsg (reply/id grant) arrives to the agent.
        We store the id and signal the waiting 'q' command by setting an Event.
        """
        try:
            rid = int(getattr(msg, "msg_id", 0))
            payload_json = getattr(msg, "payload_json", "") or ""
            r = jparse(payload_json)

            ev = {"type":"R","id":rid,"ts":now(),"seq": self._next_seq(),"emoji":E_R,"json": r}
            self._timeline.append(ev)
            self._logs["R"].append(ev)

            # Synchronize: record id and wake up q-waiter
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
        """
        Send Q and wait indefinitely for R; return the granted id or None.

        Implementation notes:
          • We send the service call but we do NOT depend on its return payload
            for the id. The actual R comes via the reply topic; we block on an
            Event that the reply callback sets.
        """
        # Compose Q with a simple route: [agent] → dest = node_name
        q = {"Q": True, "id": 0, "route": [self.agent], "dest": self.node_name}
        q_ev = {"type":"Q","id":"-","ts":now(),"seq": self._next_seq(),"emoji":E_Q,"json": q}
        self._timeline.append(q_ev)
        self._logs["Q"].append(q_ev)
        self.get_logger().info(elog(E_Q, f"{self.agent}@{self.node_name}: Q out {jdump(q,'Q')}"))

        # Ensure service exists
        if not self._get_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: get_msg_id unavailable"))
            return None

        # Reset id + event before calling
        with self._sync_lock:
            self._last_rid = 0
            self._rid_event.clear()

        # Fire the service (R will be routed back via topic)
        req = GetMsgId.Request()
        req.agent_name = self.agent
        req.path_json = jdump(q, "Q")
        req.timeout_ms_remaining = 60000  # carried upstream; we ignore here
        fut = self._get_cli.call_async(req)

        # Wait forever for the R event; wake periodically to let ROS spin/shutdown
        while rclpy.ok() and not self._rid_event.wait(timeout=0.1):
            pass

        # Retrieve granted id
        with self._sync_lock:
            rid = int(self._last_rid)

        if rid <= 0:
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: R did not provide a valid id"))
            return None

        return rid

    def _do_d_only(self, text: Any) -> bool:
        """
        Send a D using the last received R id.
        If no id is stored, refuse and instruct the user to run 'q' or 'send'.
        """
        # Snapshot id under lock
        with self._sync_lock:
            msg_id = int(self._last_rid)

        if msg_id <= 0:
            print("No R id stored yet. Run 'q' or 'send' first.")
            return False

        # Compose D
        dup = {"D": True, "id": msg_id, "src": self.agent, "dest": self.node_name, "msg": text}
        ev_submit = {"type":"D_sent","id":msg_id,"ts":now(),"seq": self._next_seq(),"emoji":E_DU,"json":dup}
        self._timeline.append(ev_submit)
        self._logs["D_sent"].append(ev_submit)
        self.get_logger().info(elog(E_DU, f"{self.agent}@{self.node_name}: D out {jdump(dup,'D')}"))

        # Ensure submit_dm is reachable
        if not self._submit_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(elog(T_WRN, f"{self.agent}@{self.node_name}: submit_dm unavailable"))
            return False

        # Fire and (lightly) wait for transport ACK (not end-to-end)
        req = SubmitDm.Request()
        req.msg_id = msg_id
        req.src_agent = self.agent
        req.payload_json = jdump(dup, "D")
        fut = self._submit_cli.call_async(req)
        self._poll(fut, 10.0)
        return True

    def _do_send(self, text: Any) -> bool:
        """Convenience: Q → wait R → D. Returns True on success."""
        rid = self._do_q_wait_r()
        if not rid:
            return False
        return self._do_d_only(text)

    # ───────────────────────────────────────────────────────────────────
    # Misc helpers
    # ───────────────────────────────────────────────────────────────────
    def _next_seq(self) -> int:
        """Monotone sequence counter for within-group order."""
        self._seq_counter += 1
        return self._seq_counter

    def _poll(self, fut: Future, timeout_s: float) -> bool:
        """Tiny future poller to avoid blocking forever on service ACKs."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end and rclpy.ok():
            if fut.done():
                return True
            time.sleep(0.01)
        return fut.done()

    def _viz_tick(self):
        """Optional heartbeat for dashboards (counts per group)."""
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
    """Parse --agent and --node from the command line."""
    p = argparse.ArgumentParser(description="ProjectN Single-Agent Hub")
    p.add_argument("--agent", required=True, help="agent name to attach")
    p.add_argument("--node", required=True, help="node name to attach the agent to")
    return p.parse_args(argv)

def main(argv: Optional[List[str]] = None):
    """Standard ROS 2 entrypoint: start the hub and spin the executor."""
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
        # Clean shutdown of executor and ROS
        try:
            exec.shutdown()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()
