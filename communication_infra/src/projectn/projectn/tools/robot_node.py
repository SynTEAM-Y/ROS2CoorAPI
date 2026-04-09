#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Robot Node 
+ Auto-capture COORDS from DOWN text like: "Rescue [x, y]?"
+ Explicit IDLE block (no sendable): use `>idle` (or `>__idle__`) with only "commands to execute:"
  - Runs once on scenario load.
  - Runs again automatically after any winning program finishes.

Behavior summary:
- On scenario load → run IDLE block steps (now: only once attached; see code).
- On trigger DOWN:
    • robot sends its '!' (pre-ack protocol unchanged).
    • if lose → stay in idle (do nothing).
    • if win  → run that program's steps, then run IDLE steps again (return to idle).

Notes:
- IDLE block does NOT contain a send-text line ending with '!'. It is never sent or triggered.
"""

from __future__ import annotations
import argparse, os, sys, time, threading, subprocess, shutil, re, signal
from pathlib import Path
from typing import Optional, Dict, List, Tuple
from collections import deque

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String as RosString

try:
    import yaml  # for parsing the inline 'commands:' block
except Exception:
    yaml = None


# ---------- helpers ----------
def _compose_shell(cmd: str) -> str:
    setup = os.environ.get("ROS_SETUP", "").strip()
    if setup:
        return f'source "{setup}" >/dev/null 2>&1; {cmd}'
    return cmd

def exec_shell_any(cmd: str, timeout: Optional[int] = None) -> Tuple[int, str, str]:
    if timeout is None:
        try:
            timeout = int(os.environ.get("EXEC_TIMEOUT", "60"))
        except ValueError:
            timeout = 60
    try:
        wrapped = _compose_shell(cmd)
        proc = subprocess.run(
            wrapped,
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            text=True,
            timeout=timeout,
            env=os.environ,
        )
        return proc.returncode, proc.stdout or "", proc.stderr or ""
    except subprocess.TimeoutExpired as e:
        return 124, e.stdout or "", (e.stderr or "") + "\n[TIMEOUT]"
    except Exception as e:
        return 127, "", f"[exec-error] {e}"

def _norm_base(token: str) -> str:
    """
    Normalize trigger names:

    - Strip ONE trailing '?' or '!' (Rescue? / Rescue! -> Rescue)
    - Keep ONLY the first word (before any space), so that:
        "Rescue [1.0, 2.0]!" -> "Rescue"
        "Rescue [1.0, 2.0]"  -> "Rescue"
        "Rescue?"            -> "Rescue"
        "Rescue"             -> "Rescue"
    """
    if not token:
        return ""
    s = token.strip()
    # Strip one trailing ? or !
    if s.endswith("?") or s.endswith("!"):
        s = s[:-1].strip()
    # Only first word
    return s.split()[0]


# coordinates regex — matches [x, y] with optional decimals / signs
_COORD_RE = re.compile(r"\[\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\]")


# ---------- scenario parsing ----------
class Program:
    __slots__ = ("recv_name_raw", "recv_base", "send_text", "win_cmds", "rival_recv_raw", "rival_base")
    def __init__(self, recv_name_raw: str, send_text: str, win_cmds: List[str], rival_recv_raw: str):
        self.recv_name_raw = recv_name_raw              # e.g., "Rescue?"
        self.recv_base = _norm_base(recv_name_raw)      # e.g., "Rescue"
        self.send_text = send_text                      # e.g., "propose!"
        self.win_cmds = list(win_cmds)                  # e.g., ["patrol_stop", "set_coords", ...]
        self.rival_recv_raw = rival_recv_raw            # e.g., "propose?"
        self.rival_base = _norm_base(rival_recv_raw)    # e.g., "propose"

class Scenario:
    def __init__(self, commands_map: Dict[str, str]):
        self.by_recv_base: Dict[str, Program] = {}   # key by recv_base
        self.sendables: List[str] = []               # unique send_text (with '!')
        self.commands_map: Dict[str, str] = commands_map or {}
        self.programs_in_order: List[Program] = []   # preserve order
        self.idle_cmds: List[str] = []               # explicit idle steps (no sendable)

    @staticmethod
    def _parse_front_commands(lines: List[str]) -> Tuple[Dict[str, str], int]:
        if yaml is None:
            # skip preface if PyYAML missing
            idx = 0
            while idx < len(lines) and not lines[idx].lstrip().startswith(">"):
                idx += 1
            return {}, idx

        pre = []
        idx = 0
        while idx < len(lines) and not lines[idx].lstrip().startswith(">"):
            pre.append(lines[idx]); idx += 1

        text = "\n".join(pre).strip()
        if not text:
            return {}, idx

        try:
            data = yaml.safe_load(text)
        except Exception:
            return {}, idx

        if not isinstance(data, dict) or "commands" not in data:
            return {}, idx

        cmds = data["commands"]
        mapping: Dict[str, str] = {}
        if isinstance(cmds, dict):
            for k, v in cmds.items():
                k2 = (k or "").strip(); v2 = (v or "").strip()
                if k2 and v2: mapping[k2] = v2
        elif isinstance(cmds, list):
            for item in cmds:
                if isinstance(item, dict) and "name" in item and "shell" in item:
                    name = str(item["name"]).strip()
                    shell = str(item["shell"]).strip()
                    if name and shell: mapping[name] = shell
        return mapping, idx

    @staticmethod
    def load_from_file(path: str) -> "Scenario":
        lines = Path(path).read_text(encoding="utf-8").splitlines()

        # commands preface
        commands_map, start_idx = Scenario._parse_front_commands(lines)
        sc = Scenario(commands_map)

        # scan blocks
        i = start_idx
        while i < len(lines):
            line = lines[i].rstrip(); i += 1
            if not line.lstrip().startswith(">"):
                continue

            token = line.lstrip()[1:].strip()
            if not token:
                continue

            base_token = _norm_base(token).lower()

            # ---------- explicit idle block WITHOUT sendable ----------
            if base_token in ("idle", "__idle__"):
                idle_steps: List[str] = []
                state = "scan"
                while i < len(lines):
                    l = lines[i].rstrip()
                    if l.lstrip().startswith(">"):
                        break
                    i += 1
                    s = l.strip()
                    if not s:
                        continue
                    if state == "scan":
                        if s.lower().startswith("commands to execute"):
                            # collect until '+' or next block
                            while i < len(lines):
                                l2 = lines[i].rstrip()
                                if l2.lstrip().startswith(">") or l2.strip().startswith("+"):
                                    break
                                i += 1
                                cs = l2.strip()
                                if cs:
                                    idle_steps.append(cs)
                            # skip optional '+' and one possible rival line (ignored for idle)
                            if i < len(lines) and lines[i].strip().startswith("+"):
                                i += 1
                                if i < len(lines) and not lines[i].lstrip().startswith(">"):
                                    i += 1
                            break
                        else:
                            continue
                sc.idle_cmds = idle_steps
                continue
            # ----------------------------------------------------------

            # ---------- normal program (requires a sendable '!') ----------
            recv_raw = token
            send_text = ""
            win_cmds: List[str] = []
            rival_recv_raw = ""

            state = "need_send"
            while i < len(lines):
                l = lines[i].rstrip()
                if l.lstrip().startswith(">"):
                    break
                i += 1
                s = l.strip()
                if not s:
                    continue

                if state == "need_send":
                    if s.endswith("!"):
                        send_text = s
                        state = "maybe_cmds"
                    else:
                        continue
                    continue

                if state == "maybe_cmds":
                    if s.lower().startswith("commands to execute"):
                        while i < len(lines):
                            l2 = lines[i].rstrip()
                            if l2.lstrip().startswith(">") or l2.strip().startswith("+"):
                                break
                            i += 1
                            cs = l2.strip()
                            if cs:
                                win_cmds.append(cs)
                        if i < len(lines) and lines[i].strip().startswith("+"):
                            i += 1
                            state = "after_plus"
                        continue
                    elif s.startswith("+"):
                        state = "after_plus"
                        continue
                    else:
                        continue

                if state == "after_plus":
                    rival_recv_raw = s
                    break

            send_text = send_text.strip()
            if not send_text.endswith("!"):
                continue  # malformed; skip

            prog = Program(recv_raw, send_text, win_cmds, rival_recv_raw)
            sc.by_recv_base[prog.recv_base] = prog
            if send_text not in sc.sendables:
                sc.sendables.append(send_text)
            sc.programs_in_order.append(prog)

        return sc

    def idle_fallback(self) -> List[str]:
        """If no explicit >idle, fallback to first program's steps (if any)."""
        if self.idle_cmds:
            return list(self.idle_cmds)
        if self.programs_in_order:
            return list(self.programs_in_order[0].win_cmds)
        return []


# ---------- Robot Node ----------
class RobotNode(Node):
    def __init__(self, robot: str):
        super().__init__("projectn_robot_node")
        self.robot = robot
        self.agent: Optional[str] = None

        # pubs/subs set on attach
        self.pub_in = None
        self.sub_out = None
        self.sub_ack = None
        self.sub_attached = None
        self.sub_detached = None

        # control
        self.pub_attach = self.create_publisher(RosString, "/robot_bridge/attach", 10)
        self.pub_detach = self.create_publisher(RosString, "/robot_bridge/detach", 10)

        # history
        self.sent_seq: List[str] = []
        self.rec_seq: List[str] = []

        # pending + Robot Fifo
        self._pending_ack = deque()
        self._pending_lock = threading.Lock()
        self._robot_fifo = deque()  # holds 'skip' tokens

        # scenario
        self._scenario_path: Optional[str] = None
        self._scenario: Optional[Scenario] = None
        self._send_to_program: Dict[str, Program] = {}
        self._idle_cmds: List[str] = []  # explicit idle steps (or fallback)
        self._idle_armed: bool = False   # run idle after attach confirm if True

        # sync
        self._print_lock = threading.Lock()
        self._attach_ok_event = threading.Event()

    # ----- scenario API -----
    def scenario_load(self, path: str):
        p = str(Path(path).expanduser())
        sc = Scenario.load_from_file(p)
        self._scenario_path = p
        self._scenario = sc
        self._send_to_program.clear()
        for prog in sc.by_recv_base.values():
            self._send_to_program[prog.send_text] = prog

        # choose idle: explicit >idle if present, otherwise fallback to first program steps
        self._idle_cmds = sc.idle_fallback()

        print(f"[scenario] loaded: {p}  programs={len(sc.by_recv_base)}  sendables={len(sc.sendables)}  commands={len(sc.commands_map)}")
        if self._idle_cmds:
            print(f"[idle] steps: {len(self._idle_cmds)}")
            # Only run idle immediately if we are already attached & confirmed.
            if self.agent and self._attach_ok_event.is_set():
                self._idle_armed = False
                self._enter_idle()
            else:
                self._idle_armed = True
                print("[idle] armed (will run after attach confirm)")
        else:
            print("[idle] no idle steps found")
            self._idle_armed = False

    def scenario_reload(self):
        if not self._scenario_path:
            print("[scenario] no file loaded"); return
        self.scenario_load(self._scenario_path)

    def scenario_show(self):
        if not self._scenario:
            print("[scenario] not loaded"); return
        print("=== Scenario Programs (in order) ===")
        for i, prog in enumerate(self._scenario.programs_in_order, 1):
            base = prog.recv_base
            print(f"{i}. Trigger: {prog.recv_name_raw}  (base='{base}')  send: {prog.send_text}  rival: {prog.rival_recv_raw}")
            if prog.win_cmds:
                print("   commands to execute:")
                for j, c in enumerate(prog.win_cmds, 1):
                    print(f"     {j}. {c}")
        print("\n=== Embedded Commands Map ===")
        if not self._scenario.commands_map:
            print("  (none)")
        else:
            for i, (k, v) in enumerate(self._scenario.commands_map.items(), 1):
                print(f"  {i}. {k} → {v}")
        print("\n=== Idle Steps ===")
        if not self._idle_cmds:
            print("  (none)")
        else:
            for i, c in enumerate(self._idle_cmds, 1):
                print(f"  {i}. {c}")

    # ----- helper: (re)enter idle -----
    def _enter_idle(self):
        """Run idle steps now."""
        if not self._idle_cmds:
            return
        print("[idle] entering...")
        for step in self._idle_cmds:
            self._execute_one(step)

    # ----- attach/detach -----
    def _cleanup_agent_bindings(self):
        for sub in (self.sub_out, self.sub_ack, self.sub_attached, self.sub_detached):
            if sub:
                try: self.destroy_subscription(sub)
                except Exception: pass
        self.sub_out = self.sub_ack = self.sub_attached = self.sub_detached = None
        if self.pub_in:
            try: self.destroy_publisher(self.pub_in)
            except Exception: pass
        self.pub_in = None
        self._attach_ok_event.clear()
        # clear env hints
        os.environ.pop("AGENT", None)
        os.environ.pop("ROBOT", None)

    def attach(self, agent: str):
        agent = (agent or "").strip()
        if not agent:
            print("usage: attach <AGENT>"); return
        if self.agent == agent and self.pub_in:
            print(f"[attach] already attached to {agent}"); return

        self._cleanup_agent_bindings()
        self.agent = agent

        # export into environment so scenario shell commands can use $AGENT / $ROBOT
        os.environ["AGENT"] = agent
        os.environ["ROBOT"] = self.robot

        self.pub_in = self.create_publisher(RosString, f"/robot_bridge/in/{agent}/{self.robot}", 10)
        self.sub_out = self.create_subscription(RosString, f"/robot_bridge/out/{agent}/{self.robot}", self._on_down_string, 50)
        self.sub_ack = self.create_subscription(RosString, f"/robot_bridge/ack/{agent}/{self.robot}", self._on_ack_string, 50)
        self.sub_attached = self.create_subscription(RosString, f"/robot_bridge/attached/{agent}/{self.robot}", self._on_scoped_attached, 10)
        self.sub_detached = self.create_subscription(RosString, f"/robot_bridge/detached/{agent}/{self.robot}", self._on_scoped_detached, 10)

        payload = f'{{"agent":"{agent}","robot":"{self.robot}"}}'
        self.pub_attach.publish(RosString(data=payload))
        print(f"[attach] requested -> agent={agent}")

    def detach(self):
        if not self.agent:
            print("Not attached."); return
        with self._pending_lock:
            self._pending_ack.clear()
        self._robot_fifo.clear()
        payload = f'{{"agent":"{self.agent}","robot":"{self.robot}"}}'
        self.pub_detach.publish(RosString(data=payload))
        print(f"[detach] requested -> agent={self.agent}")
        # env will be cleared in _cleanup_agent_bindings once detach confirm arrives

    def _on_scoped_attached(self, msg: RosString):
        s = (msg.data or "").strip()
        if s:
            self.rec_seq.append(s)
            print(f"[hub-confirm] {s}")
            self._attach_ok_event.set()
        # If idle was armed (scenario loaded before attach), run it now.
        if self._idle_cmds and self._idle_armed:
            self._idle_armed = False
            self._enter_idle()

    def _on_scoped_detached(self, msg: RosString):
        s = (msg.data or "").strip()
        if s:
            self.rec_seq.append(s)
            print(f"[hub-detach] {s}")
        self._cleanup_agent_bindings()
        self.agent = None

    # ----- send -----
    def send_text(self, text: str):
        if not self.agent or not self.pub_in:
            print("[ROBOT] Attach first: attach <AGENT>"); return
        s = (text or "").strip()
        if not s:
            return
        self.pub_in.publish(RosString(data=s))
        self.sent_seq.append(s)
        with self._pending_lock:
            self._pending_ack.append(s)
        print(f"[send] {s}")

    # ----- DOWN handler -----
    def _on_down_string(self, ros_msg: RosString):
        text = (ros_msg.data or "").strip()
        if not text:
            return
        self.rec_seq.append(text)
        print(f"[down] {text}")

        # global skip is dropped
        if text == "skip":
            print("[ROBOT] received 'skip' -> dropped")
            return

        # auto-capture coords like "[x, y]" into ENV COORDS for later steps
        m = _COORD_RE.search(text)
        if m:
            x, y = m.group(1), m.group(2)
            os.environ["COORDS"] = f"[{x}, {y}]"
            print(f"[coords] cached COORDS={os.environ['COORDS']}")

        if not self._scenario:
            return

        incoming_base = _norm_base(text)

        # Rival detection first: if this DOWN is the rival marker and our own send is pending -> enqueue 'skip'
        for prog in self._scenario.by_recv_base.values():
            if incoming_base == prog.rival_base and prog.rival_base:
                with self._pending_lock:
                    pending = list(self._pending_ack)
                if prog.send_text in pending:
                    self._robot_fifo.append("skip")
                    print(f"[FIFO] rival '{prog.rival_recv_raw}' while '{prog.send_text}' pending -> Robot Fifo += 'skip'")

        # Trigger: match by base (Summon/Summon?/Summon!)
        prog = self._scenario.by_recv_base.get(incoming_base)
        if prog:
            # Send its UP (e.g., propose!)
            self.send_text(prog.send_text)
            # Remember mapping for execution on pre-ack
            self._send_to_program[prog.send_text] = prog
            return

    # ----- PRE-ACK handler -----
    def _on_ack_string(self, ros_msg: RosString):
        raw = (ros_msg.data or "").strip()
        if not raw:
            return
        self.rec_seq.append(raw)

        # We only care about pre-ack
        if not raw.lower().startswith("ack:ready:"):
            return

        # Minimal operator log (hide ids/text)
        print("[agent ack] your message is ready")

        # Parse to get id + original to decide
        try:
            _, rest = raw.split("ack:ready:", 1)
            sid, original = rest.split(":", 1)
            did = int(sid)
            original = original.strip()
        except Exception:
            return

        # Decision via Robot Fifo
        decision = "send"
        if self._robot_fifo:
            try: self._robot_fifo.popleft()
            except Exception: pass
            decision = "skip"

        if self.pub_in:
            self.pub_in.publish(RosString(data=f"__dec__:{decision}:{did}"))
            print(f"[robot ack] {decision}")

        # Remove original from pending (if present)
        removed = False
        with self._pending_lock:
            try:
                for _ in range(len(self._pending_ack)):
                    s = self._pending_ack[0].strip()
                    if s == original:
                        self._pending_ack.popleft()
                        removed = True
                        break
                    else:
                        self._pending_ack.rotate(-1)
            except Exception:
                pass

        if decision == "skip":
            # Lost → remain in idle (no action)
            return

        if not removed:
            # Nothing to execute
            return

        # Win path → execute program's commands, then return to idle
        prog = self._send_to_program.get(original)

        if prog and prog.win_cmds:
            for step in prog.win_cmds:
                self._execute_one(step)
        else:
            # Fallback: if no defined steps, try to run the original literally
            self._execute_one(original)

        # Return to idle (run idle steps again)
        self._enter_idle()

    # ----- execution -----
    def _resolve_command(self, spec: str) -> str:
        spec = (spec or "").strip()
        base = f"/{self.robot}/cmd_vel"

        # raw shell form
        if spec.startswith("sh:"):
            return spec[3:].strip()

        # lookup in embedded commands map
        if self._scenario and spec in self._scenario.commands_map:
            template = self._scenario.commands_map[spec]
            try:
                return template.format(robot=self.robot, base=base)
            except KeyError as e:
                raise RuntimeError(f"missing placeholder {e} in '{spec}'")

        # else, run literally
        return spec

    def _execute_one(self, spec: str):
        # --- 1) High-level pseudo-commands (no shell) -----------------------
        spec = (spec or "").strip()

        # send_up_file:/path/to/file  → read file & send via RobotNode.send_text()
        if spec.startswith("send_up_file:"):
            path = spec[len("send_up_file:"):].strip()
            if not path:
                with self._print_lock:
                    print("[exec] send_up_file: missing path")
                return
            try:
                txt = Path(path).read_text(encoding="utf-8").strip()
            except Exception as e:
                with self._print_lock:
                    print(f"[exec] send_up_file: failed to read '{path}': {e}")
                return
            if not txt:
                with self._print_lock:
                    print(f"[exec] send_up_file: '{path}' is empty, nothing to send")
                return
            # Use normal send logic so it goes through /robot_bridge/in/<agent>/<robot>,
            # shows up in list sent, and participates in the ack pipeline.
            self.send_text(txt)
            return

        # --- 2) Normal path: resolve via commands map and run as shell -------
        try:
            cmd = self._resolve_command(spec)
        except Exception as e:
            with self._print_lock:
                print(f"[exec-map] error: {e}")
            return

        with self._print_lock:
            print(f"[exec] {spec} → {cmd}")
        rc, out, err = exec_shell_any(cmd)
        with self._print_lock:
            if out.strip():
                print(out.rstrip())
            if err.strip():
                for line in err.rstrip().splitlines():
                    print(f"[stderr] {line}")
            print(f"[exit] {rc}")



    # ----- REPL -----
    def _help(self):
        print("""
Robot REPL
  attach <AGENT>           attach to an agent
  detach                   request detach from current agent
  send                     show numbered sendables from loaded scenario; choose to send one
  scenario load <PATH>     load a scenario file (contains BOTH programs + commands + optional >idle)
  scenario reload          reload last scenario file
  scenario show            print parsed programs, embedded commands map, and idle steps
  list sent                list sent strings (order)
  list rec                 list received lines (down + pre-ack + confirms)
  setenv KEY=VALUE         set env for future execs (e.g., ROS_SETUP=/opt/ros/humble/setup.bash)
  timeout <seconds>        change EXEC_TIMEOUT at runtime
  help
  quit
""".strip())

    def _maybe_reprint(self, last_line: str):
        if last_line.strip() == "":
            print()
            self._help()

    def run_repl(self):
        time.sleep(0.2)
        self._help()
        while rclpy.ok():
            try:
                sys.stdout.write("robot> "); sys.stdout.flush()
                line = sys.stdin.readline()
                if line is None:
                    time.sleep(0.1); continue
                parts = line.strip().split()
                if not parts:
                    self._maybe_reprint(line); continue

                cmd, *args = parts
                if cmd == "quit":
                    self._graceful_shutdown(); break

                elif cmd == "help":
                    self._help()
                    self._maybe_reprint("")
                    continue

                elif cmd == "attach":
                    if not args: print("usage: attach <AGENT>")
                    else: self.attach(args[0])
                    continue

                elif cmd == "detach":
                    self.detach(); continue

                elif cmd == "send":
                    if not self._scenario:
                        print("[send] scenario file is not loaded yet"); continue
                    if not self._scenario.sendables:
                        print("[send] no sendables found in scenario"); continue
                    print("Choose what to send:")
                    for i, s in enumerate(self._scenario.sendables, 1):
                        print(f"  {i}) {s}")
                    sel = input("> ").strip()
                    try:
                        k = int(sel)
                        if not (1 <= k <= len(self._scenario.sendables)):
                            raise ValueError
                        send_text = self._scenario.sendables[k-1]
                        self.send_text(send_text)
                        if send_text not in self._send_to_program:
                            for p in self._scenario.by_recv_base.values():
                                if p.send_text == send_text:
                                    self._send_to_program[send_text] = p
                                    break
                    except Exception:
                        print("Invalid selection.")
                    continue

                elif cmd == "scenario":
                    if not args:
                        print("usage: scenario [load <PATH>|reload|show]"); continue
                    sub = args[0].lower()
                    if sub == "load":
                        if len(args) < 2:
                            print("usage: scenario load <PATH>")
                        else:
                            path = " ".join(args[1:])
                            try:
                                self.scenario_load(path)
                            except Exception as e:
                                print(f"[scenario] load error: {e}")
                    elif sub == "reload":
                        self.scenario_reload()
                    elif sub == "show":
                        self.scenario_show()
                    else:
                        print("usage: scenario [load <PATH>|reload|show]")
                    continue

                elif cmd == "list":
                    if not args:
                        print("usage: list [sent|rec]"); continue
                    sub = args[0].lower()
                    if sub == "sent":
                        print("--- sent ---")
                        for i, s in enumerate(self.sent_seq, 1):
                            print(f"  {i}. {s}")
                    elif sub == "rec":
                        print("--- received ---")
                        for i, s in enumerate(self.rec_seq, 1):
                            print(f"  {i}. {s}")
                    else:
                        print("usage: list [sent|rec]")
                    continue

                elif cmd == "setenv":
                    if not args or "=" not in args[0]:
                        print('usage: setenv KEY=VALUE'); continue
                    k, v = args[0].split("=", 1)
                    os.environ[k] = v
                    print(f"[env] {k}={v}")
                    continue

                elif cmd == "timeout":
                    if not args:
                        print(f"[timeout] current EXEC_TIMEOUT={os.environ.get('EXEC_TIMEOUT','60')}"); continue
                    try:
                        sec = float(args[0]);  assert sec > 0
                        os.environ["EXEC_TIMEOUT"] = str(int(sec))
                        print(f"[timeout] EXEC_TIMEOUT set to {int(sec)}")
                    except Exception:
                        print("usage: timeout <seconds>, seconds > 0")
                    continue

                else:
                    self._help()
                    self._maybe_reprint("")
                    continue

            except Exception as e:
                print(f"[REPL error] {e}")
                time.sleep(0.1)

    # ----- startup / shutdown -----
    def startup_interactive(self):
        print(f"🤖 RobotNode '{self.robot}' ready.")
        yn = input("Do you want to create an agent for this robot? [Y/n]: ").strip().lower()
        if yn in ("", "y", "yes"):
            default_agent = f"{self.robot}_agent"
            name = input(f"Enter agent name (default = {default_agent}): ").strip()
            agent_name = name if name else default_agent

            # Try to spawn in a new terminal (optional)
            term = None
            for name, argv in [
                ("gnome-terminal", ["gnome-terminal", "--", "bash", "-lc"]),
                ("x-terminal-emulator", ["x-terminal-emulator", "-e", "bash", "-lc"]),
                ("konsole", ["konsole", "-e", "bash", "-lc"]),
                ("xfce4-terminal", ["xfce4-terminal", "--command", "bash -lc"]),
                ("tilix", ["tilix", "-e", "bash", "-lc"]),
                ("kitty", ["kitty", "bash", "-lc"]),
                ("alacritty", ["alacritty", "-e", "bash", "-lc"]),
                ("lxterminal", ["lxterminal", "-e", "bash", "-lc"]),
                ("xterm", ["xterm", "-e", "bash", "-lc"]),
            ]:
                if shutil.which(name):
                    term = argv; break
            if term:
                launch = _compose_shell(f'ros2 run projectn agent_hub_robot --agent "{agent_name}"; exec bash')
                try:
                    subprocess.Popen(term + [launch], env=os.environ)
                    print(f"[spawn] agent '{agent_name}' launched in a new terminal.")
                except Exception as e:
                    print(f"[spawn] failed to spawn terminal: {e}")
            else:
                print("[spawn] no terminal emulator found. Start agent manually if needed.")

            self.attach(agent_name)
            t0 = time.time()
            payload = f'{{"agent":"{agent_name}","robot":"{self.robot}"}}'
            while not self._attach_ok_event.is_set() and (time.time() - t0 < 20.0):
                if self.pub_attach:
                    self.pub_attach.publish(RosString(data=payload))
                time.sleep(0.5)

            if self._attach_ok_event.is_set():
                print(f"[attach] confirmed with agent '{agent_name}'.")
            else:
                print("[attach] no confirmation yet; topics bound, continue.")
        else:
            print("Skipping agent creation. Use 'attach <AGENT>' later.")

    def _graceful_shutdown(self):
        try:
            if self.agent:
                self.detach()
        finally:
            try: rclpy.shutdown()
            except Exception: pass


def main():
    ap = argparse.ArgumentParser(description="ProjectN Robot Node (punctuation-agnostic + coords-capture + explicit idle)")
    ap.add_argument("--robot", required=True, help="Robot name")
    ap.add_argument("--scenario", help="Path to scenario file (contains BOTH programs + commands + optional >idle)")
    args = ap.parse_args()

    rclpy.init()
    node = RobotNode(args.robot)

    # NOTE: scenario is loaded BEFORE interactive attach; idle is armed and will run after attach confirm
    if args.scenario:
        try:
            node.scenario_load(args.scenario)
        except Exception as e:
            print(f"[scenario] load error: {e}")

    node.startup_interactive()

    exe = MultiThreadedExecutor()
    exe.add_node(node)
    repl_thread = threading.Thread(target=node.run_repl, daemon=True)
    repl_thread.start()

    try:
        exe.spin()
    except KeyboardInterrupt:
        node._graceful_shutdown()
    finally:
        try: exe.shutdown()
        except Exception: pass
        if rclpy.ok():
            try: rclpy.shutdown()
            except Exception: pass


if __name__ == "__main__":
    main()
