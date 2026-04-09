#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Agent Supervisor (minimal, with D-only fanout)
- Lists running agent hubs (Agent @@ Node)
- Triggers Q from one/many/all agents
- Triggers D-only to one/many/all agents (TEXT required; hubs must already have an id)
- Triggers Q→R→D (QRE) to one/many/all agents (TEXT required)
"""

import sys, json, time, threading
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String as RosString

def jparse(s: str) -> dict:
    try:
        v = json.loads(s) if s else {}
        return v if isinstance(v, dict) else {}
    except Exception:
        return {}

def now() -> float:
    return time.time()

HELP_TEXT = """
Supervisor REPL (minimal)
────────────────────────────────────────────────────────────────
list                        # show how many agents + their names and nodes

q <AGENT...>                # send Q from the listed agent(s)
qall                        # send Q from ALL agents

d <TEXT> -- <AGENT...>      # send D(TEXT) to listed agents (requires stored id)
dall <TEXT>                 # send D(TEXT) to ALL agents  (requires stored id)

qre <TEXT> -- <AGENT...>    # Q→R→D(TEXT) for listed agents
qreall <TEXT>               # Q→R→D(TEXT) for ALL agents

help                        # show this help
quit                        # exit

Examples
  list
  q Agent1
  q Agent1 Agent3
  qall
  d hello -- Agent2 Agent4
  dall broadcast to everyone
  qre ping payload -- Agent2
  qreall system-wide check
"""

class AgentSupervisor(Node):
    def __init__(self):
        super().__init__('projectn_agent_supervisor_min')
        # agent -> info dict from heartbeats
        self._agents: Dict[str, dict] = {}

        # pubs/subs
        self._ctrl_pub = self.create_publisher(RosString, '/agent_hub/control', 10)
        self._viz_sub  = self.create_subscription(
            RosString, '/agent_hub/viz/status', self._on_viz, 50
        )

        # housekeeping
        self._lock = threading.Lock()
        self._gc_timer = self.create_timer(5.0, self._gc)

        # REPL
        threading.Thread(target=self._repl, daemon=True).start()
        self.get_logger().info("🧭 Minimal Agent Supervisor ready. Type 'help'.")

    # ── subscriptions ──────────────────────────────────────────────────
    def _on_viz(self, msg: RosString):
        d = jparse(msg.data)
        a = d.get("agent"); n = d.get("node")
        if not a or not n: return
        with self._lock:
            self._agents[a] = {"agent": a, "node": n, "ts": float(d.get("ts", now()))}

    def _gc(self):
        cutoff = now() - 15.0
        with self._lock:
            stale = [a for a,info in self._agents.items() if info.get("ts",0) < cutoff]
            for a in stale: del self._agents[a]

    # ── helpers ────────────────────────────────────────────────────────
    def _emit(self, payload: dict):
        self._ctrl_pub.publish(RosString(data=json.dumps(payload, separators=(",",":"))))

    def _known_agents(self) -> List[str]:
        with self._lock:
            return sorted(self._agents.keys())

    def _print_list(self):
        names = self._known_agents()
        print(f"Agents running: {len(names)}")
        if not names:
            print("  (none)"); return
        for a in names:
            node = self._agents[a]["node"]
            print(f"  - {a} @@ {node}")

    def _split_text_and_targets(self, args: List[str]):
        """
        Split user input like:
          ['hello', 'world', '--', 'Agent1', 'Agent2']
        -> text='hello world', targets=['Agent1','Agent2']
        If no '--' present: text=' '.join(args), targets=[]
        """
        if '--' in args:
            idx = args.index('--')
            text = ' '.join(args[:idx]).strip()
            targets = args[idx+1:]
        else:
            text = ' '.join(args).strip()
            targets = []
        return text, targets

    # ── REPL ───────────────────────────────────────────────────────────
    def _help(self):
        print(HELP_TEXT.strip())

    def _repl(self):
        self._help()
        while rclpy.ok():
            try:
                sys.stdout.write("super> "); sys.stdout.flush()
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.1); continue
                parts = line.strip().split()
                if not parts: continue

                cmd = parts[0].lower()
                args = parts[1:]

                if cmd == "quit":
                    rclpy.shutdown(); break

                elif cmd == "help":
                    self._help(); continue

                elif cmd == "list":
                    self._print_list(); continue

                elif cmd == "q":
                    if not args:
                        print("usage: q <AGENT...>"); continue
                    known = set(self._known_agents())
                    targets = [a for a in args if a in known]
                    if not targets:
                        print("No matching agents are currently online."); continue
                    self._emit({"cmd":"q", "agents": targets})
                    print(f"Sent Q to: {', '.join(targets)}")

                elif cmd == "qall":
                    if not self._known_agents():
                        print("No agents online."); continue
                    self._emit({"cmd":"q"})  # no filters => all hubs act
                    print("Sent Q to ALL agents.")

                elif cmd == "d":
                    if not args:
                        print("usage: d <TEXT> -- <AGENT...>"); continue
                    text, targets = self._split_text_and_targets(args)
                    if not text:
                        print("D requires TEXT. usage: d <TEXT> -- <AGENT...>"); continue
                    if not targets:
                        print("Specify agents after '--'. usage: d <TEXT> -- <AGENT...>"); continue
                    known = set(self._known_agents())
                    targets = [a for a in targets if a in known]
                    if not targets:
                        print("No matching agents are currently online."); continue
                    # D-only: hubs will refuse if no stored id
                    self._emit({"cmd":"d", "text": text, "agents": targets})
                    print(f"Sent D('{text}') to: {', '.join(targets)}")

                elif cmd == "dall":
                    if not args:
                        print("usage: dall <TEXT>"); continue
                    text = ' '.join(args).strip()
                    if not text:
                        print("D requires TEXT. usage: dall <TEXT>"); continue
                    if not self._known_agents():
                        print("No agents online."); continue
                    self._emit({"cmd":"d", "text": text})  # no filters => all hubs act
                    print(f"Sent D('{text}') to ALL agents.")

                elif cmd == "qre":
                    if not args:
                        print("usage: qre <TEXT> -- <AGENT...>"); continue
                    text, targets = self._split_text_and_targets(args)
                    if not text:
                        print("QRE requires TEXT. usage: qre <TEXT> -- <AGENT...>"); continue
                    if not targets:
                        print("Specify agents after '--'. usage: qre <TEXT> -- <AGENT...>"); continue
                    known = set(self._known_agents())
                    targets = [a for a in targets if a in known]
                    if not targets:
                        print("No matching agents are currently online."); continue
                    self._emit({"cmd":"send", "text": text, "agents": targets})
                    print(f"Sent QRE('{text}') to: {', '.join(targets)}")

                elif cmd == "qreall":
                    if not args:
                        print("usage: qreall <TEXT>"); continue
                    text = ' '.join(args).strip()
                    if not text:
                        print("QRE requires TEXT. usage: qreall <TEXT>"); continue
                    if not self._known_agents():
                        print("No agents online."); continue
                    self._emit({"cmd":"send", "text": text})  # no filters => all hubs act
                    print(f"Sent QRE('{text}') to ALL agents.")

                else:
                    self._help()

            except Exception as e:
                print(f"[REPL ERROR] {e}", file=sys.stderr)

def main():
    rclpy.init()
    sup = AgentSupervisor()
    exec = MultiThreadedExecutor()
    exec.add_node(sup)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try: exec.shutdown()
        except: pass
        if rclpy.ok():
            try: rclpy.shutdown()
            except: pass

if __name__ == "__main__":
    main()
