#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Agent Hub (plain strings + ack, strict W-queue; multi-robot per agent)
• Fully scoped robot I/O (per-robot topics)
• Immediate numeric attach menu on startup (runs before REPL)
• Minimal numbered REPL with live W-queue delay control
"""

from __future__ import annotations
import argparse, json, random, re, threading, time, queue, sys
from typing import Any, Dict, Optional, Set, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String as RosString
from projectn_interfaces.srv import GetMsgId, SubmitDm
from projectn_interfaces.msg import RMsg, DataMsg


# ── helpers ─────────────────────────────────────────────────────────────
def jparse(s: str) -> dict:
    try:
        v = json.loads(s) if s else {}
        return v if isinstance(v, dict) else {}
    except Exception:
        return {}

def jdump(obj: dict, kind: str = "") -> str:
    if not isinstance(obj, dict):
        return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
    order_map = {
        "Q": ["Q", "id", "dest", "route", "msg", "src"],
        "R": ["R", "id", "dest", "route"],
        "D": ["D", "id", "dest", "src", "msg"],
    }
    order = order_map.get(kind, [])
    out = {k: obj[k] for k in order if k in obj} if order else dict(obj)
    if order:
        for k in sorted(obj.keys()):
            if k not in out:
                out[k] = obj[k]
    return json.dumps(out, ensure_ascii=False, separators=(",", ":"))


# ── hub ─────────────────────────────────────────────────────────────────
class AgentHubRobot(Node):
    _VIZ_RE = re.compile(r"^/([^/]+)/viz/status$")  # /<node>/viz/status

    def __init__(self, agent: str, heartbeat_hz: float = 1.0, attach_timeout_s: float = 10.0, d_delay_s: float = 30.0):
        super().__init__("projectn_agent_hub_robot")

        # config
        self.agent: str = agent.strip()
        if not self.agent:
            raise RuntimeError("--agent is required")
        self._hb_period = max(0.2, 1.0 / heartbeat_hz)
        self._attach_timeout_s = max(1.0, attach_timeout_s)
        self._d_delay_s = max(0.0, d_delay_s)  # delay after each D (UP/DOWN)

        # discovery/attach
        self._attached = False
        self._node_name: Optional[str] = None
        self._known_nodes: Set[str] = set()
        self._node_subs: Dict[str, Any] = {}
        self._scan_lock = threading.Lock()

        # MULTI-ROBOT: registry and per-robot input subscriptions
        self._robots: Set[str] = set()
        self._robots_lock = threading.Lock()
        self._sub_in_by_robot: Dict[str, Any] = {}

        # reply coordination (R)
        self._r_event = threading.Event()
        self._rid_fifo: "queue.Queue[int]" = queue.Queue()

        # strict ordering (SEEDed)
        self.nid_seeded = False
        self.nid = 1  # set by SEED

        # queues
        # I items: ("UP", text, robot)
        self.I: "queue.Queue[Tuple[str, str, str]]" = queue.Queue()
        # W items: (did, {"origin":"UP"/"DOWN", "d":payload, "text":..., "robot":...})
        self.W: "queue.PriorityQueue[Tuple[int, Dict[str, Any]]]" = queue.PriorityQueue()

        # robot-bridge pubs/subs
        self._pub_agents = self.create_publisher(RosString, "/robot_bridge/agents", 10)
        self._sub_attach = self.create_subscription(RosString, "/robot_bridge/attach", self._on_robot_attach_cmd, 50)
        self._sub_detach = self.create_subscription(RosString, "/robot_bridge/detach", self._on_robot_detach_cmd, 50)
        # per-robot /robot_bridge/in/<agent>/<robot> subscriptions are created on attach.

        # publisher caches
        self._pub_out_by_robot: Dict[str, Any] = {}
        self._pub_ack_by_robot: Dict[str, Any] = {}
        self._pub_attach_by_robot: Dict[str, Any] = {}
        self._pub_detach_by_robot: Dict[str, Any] = {}

        # tree-node handles
        self._sub_reply: Optional[Any] = None   # RMsg (and SEED)
        self._sub_inbox: Optional[Any] = None   # DataMsg DOWN
        self._cli_get: Optional[Any] = None     # GetMsgId
        self._cli_submit: Optional[Any] = None  # SubmitDm

        # timers
        self._hb_timer = self.create_timer(self._hb_period, self._heartbeat)
        self._scan_timer = self.create_timer(1.0, self._scan_topics_for_nodes)

        # workers
        threading.Thread(target=self._worker_I, daemon=True).start()
        threading.Thread(target=self._worker_W, daemon=True).start()

        # run startup interaction (attach menu) FIRST, then start the REPL (avoid stdin contention)
        threading.Thread(target=self._startup_then_repl, daemon=True).start()

        self.get_logger().info(f"🤖 AgentHubRobot for agent '{self.agent}' ready (multi-robot enabled).")

    # ── heartbeat / discovery ───────────────────────────────────────────
    def _heartbeat(self):
        try:
            with self._robots_lock:
                robots = sorted(self._robots)
            payload = {
                "agent": self.agent,
                "status": "attached" if self._attached else "idle",
                "node": self._node_name or "",
                "nid": self.nid,
                "seeded": self.nid_seeded,
                "robots": robots,
                "ts": time.time(),
            }
            self._pub_agents.publish(RosString(data=json.dumps(payload, separators=(",", ":"))))
        except Exception:
            pass

    def _scan_topics_for_nodes(self):
        try:
            names_and_types = dict(self.get_topic_names_and_types())
        except Exception:
            return
        for topic, types in names_and_types.items():
            if "std_msgs/msg/String" not in types:
                continue
            m = self._VIZ_RE.match(topic)
            if not m:
                continue
            name = m.group(1)
            if not name:
                continue
            with self._scan_lock:
                if name in self._node_subs:
                    continue
                self._node_subs[name] = self.create_subscription(
                    RosString, topic, lambda _msg, nn=name: self._on_viz(nn), 10
                )

    def _on_viz(self, node_name: str):
        with self._scan_lock:
            self._known_nodes.add(node_name)

    # ── scoped confirm helpers ──────────────────────────────────────────
    def _publish_attached(self, robot: str, ok: bool, reason: str = "OK"):
        topic = f"/robot_bridge/attached/{self.agent}/{robot}"
        pub = self._pub_attach_by_robot.get(robot)
        if pub is None:
            pub = self.create_publisher(RosString, topic, 10)
            self._pub_attach_by_robot[robot] = pub
        payload = {"agent": self.agent, "ok": bool(ok), "reason": reason}
        pub.publish(RosString(data=json.dumps(payload, separators=(",", ":"))))

    def _publish_detached(self, robot: str, ok: bool, reason: str = "BYE"):
        topic = f"/robot_bridge/detached/{self.agent}/{robot}"
        pub = self._pub_detach_by_robot.get(robot)
        if pub is None:
            pub = self.create_publisher(RosString, topic, 10)
            self._pub_detach_by_robot[robot] = pub
        payload = {"agent": self.agent, "ok": bool(ok), "reason": reason}
        pub.publish(RosString(data=json.dumps(payload, separators=(",", ":"))))

    def _publish_out(self, robot: str, text: str):
        topic = f"/robot_bridge/out/{self.agent}/{robot}"
        pub = self._pub_out_by_robot.get(robot)
        if pub is None:
            pub = self.create_publisher(RosString, topic, 10)
            self._pub_out_by_robot[robot] = pub
        pub.publish(RosString(data=text))

    def _publish_ack(self, robot: str, text: str):
        topic = f"/robot_bridge/ack/{self.agent}/{robot}"
        pub = self._pub_ack_by_robot.get(robot)
        if pub is None:
            pub = self.create_publisher(RosString, topic, 10)
            self._pub_ack_by_robot[robot] = pub
        pub.publish(RosString(data=f"ack: {text}"))

    # ── robot attach/detach (control) ───────────────────────────────────
    def _on_robot_attach_cmd(self, msg: RosString):
        cfg = jparse(msg.data)
        agent = (cfg.get("agent") or "").strip()
        if agent != self.agent:
            return
        robot = (cfg.get("robot") or "").strip() or "Robot"

        # add robot to registry, create per-robot input subscription
        with self._robots_lock:
            first_time = robot not in self._robots
            self._robots.add(robot)
        if first_time:
            topic_in = f"/robot_bridge/in/{self.agent}/{robot}"
            self._sub_in_by_robot[robot] = self.create_subscription(
                RosString, topic_in, lambda m, r=robot: self._on_robot_text(r, m), 50
            )
            self.get_logger().info(f"[HUB] Registered robot '{robot}' on {topic_in}")

        self.get_logger().info(f"[HUB] Robot '{robot}' wishes to use agent '{self.agent}'.")
        self._publish_attached(robot, True, "OK")

    def _on_robot_detach_cmd(self, msg: RosString):
        cfg = jparse(msg.data)
        agent = (cfg.get("agent") or "").strip()
        robot = (cfg.get("robot") or "").strip() or "Robot"
        if agent != self.agent:
            return
        self.get_logger().info(f"[HUB] Robot '{robot}' detached from '{self.agent}'.")
        self._publish_detached(robot, True, "BYE")
        with self._robots_lock:
            if robot in self._robots:
                self._robots.remove(robot)
        sub = self._sub_in_by_robot.pop(robot, None)
        if sub:
            try: self.destroy_subscription(sub)
            except Exception: pass

    # ── attach/detach to tree-node ──────────────────────────────────────
    def _attach_to_node(self, node_name: str):
        try:
            self._cli_get = self.create_client(GetMsgId, f"/{node_name}/get_msg_id")
            self._cli_submit = self.create_client(SubmitDm, f"/{node_name}/submit_dm")

            if self._sub_reply:
                try: self.destroy_subscription(self._sub_reply)
                except Exception: pass
                self._sub_reply = None
            if self._sub_inbox:
                try: self.destroy_subscription(self._sub_inbox)
                except Exception: pass
                self._sub_inbox = None

            self._sub_reply = self.create_subscription(
                RMsg, f"/{node_name}/agents/{self.agent}/reply", self._on_agent_reply_cb, 50
            )
            self._sub_inbox = self.create_subscription(
                DataMsg, f"/{node_name}/agents/{self.agent}/inbox", self._on_agent_inbox_cb, 50
            )

            attach_pub = self.create_publisher(RosString, f"/{node_name}/agents/attach", 10)
            attach_pub.publish(RosString(data=self.agent))

            t0 = time.time()
            while time.time() - t0 < self._attach_timeout_s and rclpy.ok():
                if self._cli_get.wait_for_service(timeout_sec=0.2) and self._cli_submit.wait_for_service(timeout_sec=0.2):
                    break

            if not (self._cli_get and self._cli_get.service_is_ready() and self._cli_submit and self._cli_submit.service_is_ready()):
                self.get_logger().warn(f"[HUB] Services unavailable for '{node_name}'.")
                return

            self._node_name = node_name
            self._attached = True
            self.nid_seeded = False
            self.nid = 1
            self.get_logger().info(f"[HUB] ✅ Agent '{self.agent}' attached to '{node_name}'. Waiting for SEED...")
        except Exception as e:
            self.get_logger().warn(f"[HUB] Attach error: {e}")

    def _detach_from_tree(self):
        """Detach this agent from the currently attached tree node (keeps robots)."""
        try:
            if self._sub_reply:
                try: self.destroy_subscription(self._sub_reply)
                except Exception: pass
                self._sub_reply = None
            if self._sub_inbox:
                try: self.destroy_subscription(self._sub_inbox)
                except Exception: pass
                self._sub_inbox = None
            self._cli_get = None
            self._cli_submit = None
            self._attached = False
            self._node_name = None
            self.nid_seeded = False
            self.nid = 1
            self.get_logger().info("[HUB] Detached from tree node.")
        except Exception as e:
            self.get_logger().warn(f"[HUB] Detach error: {e}")

    # ── parent callbacks (R/SEED and DOWN) ──────────────────────────────
    def _on_agent_reply_cb(self, msg: RMsg):
        payload = jparse(getattr(msg, "payload_json", "") or "{}")
        # SEED
        if payload.get("SEED") is True:
            expected = int(payload.get("expected_id", 0))
            if expected > 0 and not self.nid_seeded:
                self.nid = expected
                self.nid_seeded = True
                self.get_logger().info(f"[SEED] expected_id={expected} -> nid={self.nid}")
            return

        rid = int(payload.get("id", 0) or getattr(msg, "msg_id", 0))
        if rid > 0:
            self._rid_fifo.put(rid)
            self._r_event.set()
            self.get_logger().info(f"[R] id={rid}")

    def _on_agent_inbox_cb(self, msg: DataMsg):
        try:
            payload = jparse(getattr(msg, "payload_json", "") or "{}")
            did = int(payload.get("id", 0))
            if did <= 0:
                return
            self.W.put((did, {"origin": "DOWN", "d": payload}))
            self.get_logger().info(f"[Inbox DOWN] id={did} enqueued (nid={self.nid})")
        except Exception as e:
            self.get_logger().warn(f"[Inbox DOWN] error: {e}")

    # ── robot → hub (plain string, per-robot topic) ─────────────────────
    def _on_robot_text(self, robot: str, msg: RosString):
        if not self._attached or not self._node_name:
            self.get_logger().warn("Robot string received but agent not attached to a node.")
            return
        text = (msg.data or "").strip()
        if not text:
            return
        self.get_logger().info(f"[Robot->Hub] enqueue text from {robot}: {text}")
        self.I.put(("UP", text, robot))

    # ── worker I (Q→R→enqueue UP into W) ────────────────────────────────
    def _worker_I(self):
        while rclpy.ok():
            try:
                kind, text, robot = self.I.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                if kind == "UP":
                    node = self._node_name
                    self._r_event.clear()

                    # Q
                    q = {"Q": True, "id": 0, "route": [self.agent], "dest": node}
                    self.get_logger().info(f"[Q] {jdump(q,'Q')}")
                    req = GetMsgId.Request()
                    req.agent_name = self.agent
                    req.path_json = jdump(q, "Q")
                    req.timeout_ms_remaining = 60000
                    self._cli_get.call_async(req)

                    # wait R
                    if not self._r_event.wait(timeout=10.0):
                        self.get_logger().warn("[R] timeout waiting for id")
                        self.I.task_done(); continue

                    rid = None
                    while not self._rid_fifo.empty():
                        rid = self._rid_fifo.get()
                    if not rid:
                        self.get_logger().warn("[R] missing id after wake")
                        self.I.task_done(); continue

                    # enqueue D(UP) into W (keep sender robot)
                    dup = {"D": True, "id": int(rid), "src": self.agent, "dest": node, "msg": text}
                    self.W.put((int(rid), {"origin": "UP", "d": dup, "text": text, "robot": robot}))
                    self.get_logger().info(f"[ENQ UP] id={rid} from {robot} (nid={self.nid}) → W")

            except Exception as e:
                self.get_logger().warn(f"[I] error: {e}")
            finally:
                self.I.task_done()

    # ── worker W (strict ordered processing + D-delay) ──────────────────
    def _worker_W(self):
        while rclpy.ok():
            try:
                did, item = self.W.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                if not self.nid_seeded:
                    self.W.put((did, item))
                    time.sleep(0.02)
                    continue

                if did != self.nid:
                    self.W.put((did, item))
                    time.sleep(0.02)
                    continue

                origin = item.get("origin")
                dmsg = item.get("d") or {}

                if origin == "UP":
                    # 1) submit upstream
                    dm = SubmitDm.Request()
                    dm.msg_id = did
                    dm.src_agent = self.agent
                    dm.payload_json = jdump(dmsg, "D")
                    self._cli_submit.call_async(dm)
                    self.get_logger().info(f"[PROC UP] submit id={did}")

                    # 2) ACK only to the sender robot
                    robot = item.get("robot", "")
                    if robot:
                        self._publish_ack(robot, item.get("text", ""))

                    # 3) advance nid
                    self.nid += 1

                    # 4) delay
                    delay = float(self._d_delay_s)
                    if delay > 0:
                        self.get_logger().info(f"[W] {delay:.3f}s delay before next D message...")
                        time.sleep(delay)

                elif origin == "DOWN":
                    down_text = str(dmsg.get("msg", "")).strip()
                    with self._robots_lock:
                        targets = list(self._robots)
                    if not targets:
                        self.get_logger().warn(f"[PROC DOWN] id={did} has no target robots; delivered to none.")
                    for r in targets:
                        self._publish_out(r, down_text)
                    self.get_logger().info(f"[PROC DOWN] id={did} delivered to robots={targets if targets else '[]'}")

                    self.nid += 1

                    delay = float(self._d_delay_s)
                    if delay > 0:
                        self.get_logger().info(f"[W] {delay:.3f}s delay before next D message...")
                        time.sleep(delay)

                else:
                    self.get_logger().warn(f"[W] unknown origin for id={did}; dropping")

            except Exception as e:
                self.get_logger().warn(f"[W] error: {e}")
            finally:
                self.W.task_done()

    # ── startup attach (numeric) then REPL ──────────────────────────────
    def _startup_then_repl(self):
        try:
            # small pause so discovery can begin
            time.sleep(0.6)
            self._numeric_attach_menu()
        finally:
            # after attach menu returns, start the minimal REPL
            self._repl_minimal()

    def _numeric_attach_menu(self):
        """
        Numeric, self-describing attach menu. Runs once at startup.
        Avoids using letter shortcuts to keep UX consistent with your request.
        """
        while rclpy.ok():
            with self._scan_lock:
                nodes = sorted(self._known_nodes)

            print("\n[HUB] Attach this agent to a TreeNode:")
            if nodes:
                # list each visible node with numbers 1..N
                for i, n in enumerate(nodes, 1):
                    print(f"  {i}) Attach to '{n}' now")
                base = len(nodes)
                print(f"  {base+1}) Refresh list (rescan for nodes)")
                print(f"  {base+2}) Attach by name (type exact node name)")
                print(f"  {base+3}) Random choice from visible list")
                print(f"  {base+4}) Skip for now (you can attach later via REPL option 3)")
                sel = input(f"Select [1..{base+4}]: ").strip()
                try:
                    k = int(sel)
                except Exception:
                    print("Please enter a number."); continue

                if 1 <= k <= base:
                    self._attach_to_node(nodes[k-1])
                    return
                elif k == base+1:
                    continue  # refresh
                elif k == base+2:
                    name = input("Enter exact node name: ").strip()
                    if name:
                        self._attach_to_node(name)
                        return
                elif k == base+3:
                    pick = random.choice(nodes)
                    print(f"[choose] Random → {pick}")
                    self._attach_to_node(pick)
                    return
                elif k == base+4:
                    print("[skip] You can attach later from the REPL (option 3).")
                    return
                else:
                    print("Out of range.")
            else:
                print("  (No nodes visible yet)")
                print("  1) Refresh list")
                print("  2) Attach by name (type exact node name)")
                print("  3) Skip for now (attach later via REPL option 3)")
                sel = input("Select [1..3]: ").strip()
                try:
                    k = int(sel)
                except Exception:
                    print("Please enter a number."); continue
                if k == 1:
                    continue
                elif k == 2:
                    name = input("Enter exact node name: ").strip()
                    if name:
                        self._attach_to_node(name)
                        return
                elif k == 3:
                    print("[skip] You can attach later from the REPL (option 3).")
                    return
                else:
                    print("Out of range.")

    # ── minimal numbered REPL ───────────────────────────────────────────
    def _repl_minimal(self):
        time.sleep(0.2)
        while rclpy.ok():
            try:
                print("\n[AGENT REPL]")
                print("  1) Detach robot")
                print("  2) Detach from tree")
                print("  3) Attach to tree")
                print("  4) List queues")
                print("  5) List attached robots")
                print("  6) Show delay")
                print("  7) Set delay")
                print("  8) Quit")
                choice = input("> ").strip()

                if choice == "1":
                    self._repl_detach_robot()
                elif choice == "2":
                    self._detach_from_tree()
                elif choice == "3":
                    self._numeric_attach_menu()
                elif choice == "4":
                    self._print_queue_snapshots()
                elif choice == "5":
                    with self._robots_lock:
                        robots = sorted(self._robots)
                    print(f"[robots] {robots if robots else '[]'}")
                elif choice == "6":
                    print(f"[delay] {self._d_delay_s:.3f} s")
                elif choice == "7":
                    val = input("New delay seconds (float >= 0): ").strip()
                    try:
                        v = float(val)
                        if v < 0:
                            raise ValueError
                        self._d_delay_s = v
                        print(f"[delay] set to {self._d_delay_s:.3f} s (applies to next D)")
                    except Exception:
                        print("Invalid number.")
                elif choice == "8":
                    print("[REPL] closing (node keeps running).")
                    return
                else:
                    print("Choose 1..8")
            except EOFError:
                return
            except Exception:
                time.sleep(0.1)

    def _repl_detach_robot(self):
        with self._robots_lock:
            robots = sorted(self._robots)
        if not robots:
            print("[robots] none attached.")
            return
        print("Select robot to detach:")
        for i, r in enumerate(robots, 1):
            print(f"  {i}) {r}")
        sel = input("> ").strip()
        try:
            idx = int(sel)
            if not (1 <= idx <= len(robots)):
                raise ValueError
            rname = robots[idx - 1]
        except Exception:
            print("Invalid selection.")
            return

        self._publish_detached(rname, True, "BYE")
        with self._robots_lock:
            if rname in self._robots:
                self._robots.remove(rname)
        sub = self._sub_in_by_robot.pop(rname, None)
        if sub:
            try: self.destroy_subscription(sub)
            except Exception: pass
        print(f"[robots] '{rname}' detached.")

    def _print_queue_snapshots(self):
        # snapshot I
        try:
            iq: List[Tuple[str, str, str]] = []
            with self.I.mutex:
                iq = list(self.I.queue)
        except Exception:
            iq = []

        # snapshot W
        try:
            with self.W.mutex:
                wq = list(self.W.queue)  # [(did, item)]
            wq_sorted = sorted(wq, key=lambda x: x[0])
        except Exception:
            wq_sorted = []

        with self._robots_lock:
            robots = sorted(self._robots)

        print("=== Queue Snapshot ===")
        print(f"I (len={len(iq)})")
        if not iq:
            print("  (empty)")
        else:
            for idx, (kind, text, robot) in enumerate(iq, 1):
                if kind == "UP":
                    print(f"  {idx}. UP from {robot}: '{text}'")
                else:
                    print(f"  {idx}. {kind}")

        print(f"W (len={len(wq_sorted)}) nid={self.nid} seeded={self.nid_seeded} robots={robots}")
        if not wq_sorted:
            print("  (empty)")
        else:
            for did, item in wq_sorted:
                origin = item.get("origin")
                if origin == "UP":
                    print(f"  did={did}  UP   from {item.get('robot','?')} text='{item.get('text','')}'")
                elif origin == "DOWN":
                    d = item.get("d", {})
                    print(f"  did={did}  DOWN text='{str(d.get('msg','')).strip()}'")
                else:
                    print(f"  did={did}  ?")


# ── CLI / main ──────────────────────────────────────────────────────────
def parse_args():
    ap = argparse.ArgumentParser(description="ProjectN Agent Hub (plain strings + ack, strict W-queue, scoped multi-robot I/O)")
    ap.add_argument("--agent", required=True)
    ap.add_argument("--heartbeat-hz", type=float, default=1.0)
    ap.add_argument("--attach-timeout-s", type=float, default=10.0)
    ap.add_argument("--d-delay-s", type=float, default=30.0, help="Seconds to wait after processing each D message (UP/DOWN). Use 0 for no delay.")
    return ap.parse_args()

def main(argv=None):
    args = parse_args()
    rclpy.init()
    hub = AgentHubRobot(
        agent=args.agent,
        heartbeat_hz=args.heartbeat_hz,
        attach_timeout_s=args.attach_timeout_s,
        d_delay_s=args.d_delay_s,
    )
    exe = MultiThreadedExecutor()
    exe.add_node(hub)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            exe.shutdown()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()
