#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Robot Node
(YAML-optional commands map; fully scoped I/O; ack-gated exec; 'start' random/fixed)
+ Agent auto-spawn (named) in a new terminal and auto-attach with retries.
+ Properties (getters/setters) and overload-style type hints (no behavior change).

Launch examples:
  ros2 run projectn robot_node --robot R1 --commands ~/.projectn/robot_commands.yaml
  START_COMMANDS_FILE=~/.projectn/robot_commands.yaml ros2 run projectn robot_node --robot R1
"""

from __future__ import annotations
import argparse
import os
import subprocess
import sys
import threading
import time
import secrets
import random
import shutil
from collections import deque
from pathlib import Path
from typing import Optional, Sequence, overload, Dict, List

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String as RosString

try:
    import yaml  # PyYAML
except Exception:
    yaml = None


# ---------------- Utilities ----------------
def _norm(s: str) -> str:
    return (s or "").strip().lower()


def _compose_shell(cmd: str) -> str:
    """If ROS_SETUP is set, source it before the command so 'ros2 ...' works."""
    setup = os.environ.get("ROS_SETUP", "").strip()
    if setup:
        return f'source "{setup}" >/dev/null 2>&1; {cmd}'
    return cmd


def exec_shell_any(cmd: str, timeout: Optional[int] = None) -> tuple[int, str, str]:
    """Execute arbitrary shell command string exactly as given."""
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


def _which_terminal() -> Optional[list]:
    """
    Return a terminal launch command (argv list) that supports: <term> -- bash -lc "<cmd>"
    Preference order is tuned for common Linux desktops.
    """
    candidates = [
        ("gnome-terminal", ["gnome-terminal", "--", "bash", "-lc"]),
        ("x-terminal-emulator", ["x-terminal-emulator", "-e", "bash", "-lc"]),
        ("konsole", ["konsole", "-e", "bash", "-lc"]),
        ("xfce4-terminal", ["xfce4-terminal", "--command", "bash -lc"]),
        ("tilix", ["tilix", "-e", "bash", "-lc"]),
        ("kitty", ["kitty", "bash", "-lc"]),
        ("alacritty", ["alacritty", "-e", "bash", "-lc"]),
        ("lxterminal", ["lxterminal", "-e", "bash", "-lc"]),
        ("xterm", ["xterm", "-e", "bash", "-lc"]),
        ("urxvt", ["urxvt", "-e", "bash", "-lc"]),
    ]
    for name, argv in candidates:
        if shutil.which(name):
            return argv
    return None


def _spawn_agent_terminal(agent: str) -> bool:
    """
    Spawn the Agent Hub in a new terminal, inheriting current env.
    Keeps the terminal open after the program exits (via 'exec bash').
    """
    term = _which_terminal()
    if not term:
        print("[spawn] no terminal emulator found (gnome-terminal/xterm/etc). Start agent manually.")
        return False

    launch = _compose_shell(f'ros2 run projectn agent_hub_robot --agent "{agent}"; exec bash')
    try:
        if term[0] in ("xfce4-terminal",):
            argv = term + [launch]
        else:
            argv = term + [launch]
        subprocess.Popen(argv, env=os.environ)
        print(f"[spawn] agent '{agent}' launched in a new terminal.")
        return True
    except Exception as e:
        print(f"[spawn] failed to spawn terminal: {e}")
        return False


# ---------------- Node ----------------
class RobotNode(Node):
    def __init__(self, robot: str, commands_file: Optional[str]):
        super().__init__("projectn_robot_node")
        self.robot = robot
        self.agent: Optional[str] = None

        # data pubs/subs (bound after attach)
        self.pub_in = None
        self.sub_out = None
        self.sub_ack = None

        # scoped confirms (bound after attach)
        self.sub_attached = None
        self.sub_detached = None

        # control pubs
        self.pub_attach = self.create_publisher(RosString, "/robot_bridge/attach", 10)
        self.pub_detach = self.create_publisher(RosString, "/robot_bridge/detach", 10)

        # histories
        self.sent_seq: list[str] = []   # list of sent strings (UP)
        self.rec_seq: list[str] = []    # list of received lines (DOWN + ACK + confirms)

        # ack tracking
        self._pending_ack = deque()     # strings awaiting ack (FIFO)
        self._pending_lock = threading.Lock()

        # execution lock to avoid interleaved prints
        self._print_lock = threading.Lock()

        # randomness (for 'start' random mode)
        self._rng = random.Random(secrets.randbits(64))

        # YAML commands map (optional but preferred)
        env_path = os.environ.get("START_COMMANDS_FILE", "").strip() or None
        self._commands_file = commands_file or env_path
        self._cmd_map: Dict[str, str] = {}
        self._builtins: List[str] = []

        self._try_load_yaml_commands(initial=True)

        # 'start' behavior: random | fixed
        mode_env = (os.environ.get("START_MODE", "").strip().lower() or "random")
        self._start_mode = "fixed" if mode_env == "fixed" else "random"
        self._start_fixed_cmd = os.environ.get("START_FIXED_CMD", "").strip()

        # attach confirm event (used by auto-attach retry loop)
        self._attach_ok_event = threading.Event()

    # ---------- Properties (getters / setters) ----------
    @property
    def start_mode(self) -> str:
        """'random' or 'fixed'."""
        return self._start_mode

    @start_mode.setter
    def start_mode(self, mode: str) -> None:
        m = (mode or "").strip().lower()
        if m not in ("random", "fixed"):
            raise ValueError("start_mode must be 'random' or 'fixed'")
        self._start_mode = m
        print(f"[onstart] mode set to {m}")

    @property
    def start_fixed_name(self) -> str:
        """The command NAME to send when start_mode == 'fixed'."""
        return self._start_fixed_cmd

    @start_fixed_name.setter
    def start_fixed_name(self, name: str) -> None:
        n = (name or "").strip()
        if not n:
            raise ValueError("start_fixed_name cannot be empty")
        self._start_fixed_cmd = n
        print(f"[onstart] fixed name set to '{n}'")

    @property
    def commands_file(self) -> Optional[str]:
        """Path to YAML commands file (setting it does NOT auto-load)."""
        return self._commands_file

    @commands_file.setter
    def commands_file(self, path: str) -> None:
        p = str(Path(path).expanduser())
        self._commands_file = p
        print(f"[commands] file path set -> {p} (use 'commands load' or node.load_commands() to load)")

    @property
    def commands_map(self) -> Dict[str, str]:
        """Read-only snapshot of name→shell mapping."""
        return dict(self._cmd_map)

    @property
    def command_names(self) -> List[str]:
        """List of loaded command names (used for random mode)."""
        return list(self._builtins)

    # ---------- Overloaded helpers ----------
    @overload
    def send_text(self, text: str) -> None: ...
    @overload
    def send_text(self, text: Sequence[str]) -> None: ...

    def send_text(self, text):  # runtime impl, preserves existing behavior
        if not self.agent or not self.pub_in:
            print("Attach first: attach <AGENT>")
            return

        if isinstance(text, (list, tuple)):
            for item in text:
                s = (str(item) or "").strip()
                if not s:  # skip empties
                    continue
                self.pub_in.publish(RosString(data=s))
                self.sent_seq.append(s)
                with self._pending_lock:
                    self._pending_ack.append(s)
                print(f"[send] {s}")
            return

        s = (str(text) or "").strip()
        if not s:
            return
        self.pub_in.publish(RosString(data=s))
        self.sent_seq.append(s)
        with self._pending_lock:
            self._pending_ack.append(s)
        print(f"[send] {s}")

    @overload
    def load_commands(self) -> None: ...
    @overload
    def load_commands(self, path: str) -> None: ...

    def load_commands(self, path: Optional[str] = None) -> None:
        """load_commands() -> reload last; load_commands(path) -> set path then load."""
        if path is None:
            try:
                self.load_commands_map_from_yaml(None)
            except Exception as e:
                print(f"[commands] reload error: {e}")
                print("[note] Staying in UNMAPPED mode; you can still run unmapped text as raw shells.")
        else:
            self.commands_file = path
            try:
                self.load_commands_map_from_yaml(self._commands_file)
            except Exception as e:
                print(f"[commands] load error: {e}")
                print("[note] Staying in UNMAPPED mode; you can still run unmapped text as raw shells.")

    # ---------- YAML commands map ----------
    def _try_load_yaml_commands(self, initial: bool = False):
        """Attempt to load YAML. On failure, stay in unmapped mode and inform the user."""
        if yaml is None:
            print("[warn] PyYAML not available (pip install pyyaml). Running in UNMAPPED mode.")
            print("[note] You can still execute unmapped text as raw shell commands.")
            if initial and self._commands_file:
                print(f"[hint] To load later: commands load {self._commands_file}")
            return

        path = self._commands_file
        if not path:
            if initial:
                print("[warn] No YAML path provided. Running in UNMAPPED mode.")
                print("[note] You can still execute unmapped text as raw shell commands.")
                print("[hint] Provide one via --commands PATH or START_COMMANDS_FILE, then use 'commands load <PATH>'.")
            return

        try:
            self.load_commands_map_from_yaml(path)
        except Exception as e:
            print(f"[warn] Failed to load YAML commands from '{path}': {e}")
            print("[note] Continuing in UNMAPPED mode — you can still execute unmapped text as raw shell commands.")
            print("[hint] Fix the YAML file and run: commands reload")

    def _parse_yaml_commands(self, path: str) -> Dict[str, str]:
        p = Path(path).expanduser()
        if not p.exists() or not p.is_file():
            raise FileNotFoundError(f"commands YAML not found: {p}")
        raw = p.read_text(encoding="utf-8")
        data = yaml.safe_load(raw)

        mapping: Dict[str, str] = {}

        def merge_map(obj):
            for k, v in obj.items():
                name = (k or "").strip()
                shell = (v or "").strip()
                if not name or not shell:
                    raise ValueError(f"invalid mapping entry: {k!r} -> {v!r}")
                mapping[name] = shell

        if isinstance(data, dict):
            if "commands" in data:
                cmds = data["commands"]
                if isinstance(cmds, dict):
                    merge_map(cmds)
                elif isinstance(cmds, list):
                    for idx, item in enumerate(cmds, 1):
                        if not isinstance(item, dict) or "name" not in item or "shell" not in item:
                            raise ValueError(f"commands[{idx}] must have 'name' and 'shell'")
                        name = str(item["name"]).strip()
                        shell = str(item["shell"]).strip()
                        if not name or not shell:
                            raise ValueError(f"commands[{idx}] has empty name/shell")
                        mapping[name] = shell
                else:
                    raise ValueError("'commands' must be a mapping or a list")
            else:
                merge_map(data)
        elif isinstance(data, list):
            for idx, item in enumerate(data, 1):
                if not isinstance(item, dict) or "name" not in item or "shell" not in item:
                    raise ValueError(f"[{idx}] must be an object with 'name' and 'shell'")
                name = str(item["name"]).strip()
                shell = str(item["shell"]).strip()
                if not name or not shell:
                    raise ValueError(f"[{idx}] empty name/shell")
                mapping[name] = shell
        else:
            raise ValueError("YAML must be a mapping, or a list, or a mapping with 'commands'")

        if not mapping:
            raise ValueError("no commands loaded from YAML")
        return mapping

    def load_commands_map_from_yaml(self, path: str | None) -> None:
        if path is not None:
            self._commands_file = str(Path(path).expanduser())
        if not getattr(self, "_commands_file", None):
            raise ValueError("no commands file path set")
        self._cmd_map = self._parse_yaml_commands(self._commands_file)
        self._builtins = list(self._cmd_map.keys())
        print(f"[commands] loaded {len(self._builtins)} YAML commands from {self._commands_file}")

    # ---------- Attach / Detach ----------
    def _cleanup_agent_bindings(self):
        for sub in (self.sub_out, self.sub_ack, self.sub_attached, self.sub_detached):
            if sub:
                try:
                    self.destroy_subscription(sub)
                except Exception:
                    pass
        self.sub_out = self.sub_ack = self.sub_attached = self.sub_detached = None
        if self.pub_in:
            try:
                self.destroy_publisher(self.pub_in)
            except Exception:
                pass
        self.pub_in = None
        self._attach_ok_event.clear()

    def attach(self, agent: str):
        agent = agent.strip()
        if not agent:
            print("usage: attach <AGENT>")
            return
        if self.agent == agent and self.pub_in:
            print(f"[attach] already attached to {agent}")
            return

        self._cleanup_agent_bindings()
        self.agent = agent

        # data channels (SCOPED)
        self.pub_in = self.create_publisher(RosString, f"/robot_bridge/in/{agent}/{self.robot}", 10)

        self.sub_out = self.create_subscription(
            RosString, f"/robot_bridge/out/{agent}/{self.robot}", self._on_down_string, 50
        )
        self.sub_ack = self.create_subscription(
            RosString, f"/robot_bridge/ack/{agent}/{self.robot}", self._on_ack_string, 50
        )

        # scoped confirms
        self.sub_attached = self.create_subscription(
            RosString, f"/robot_bridge/attached/{agent}/{self.robot}", self._on_scoped_attached, 10
        )
        self.sub_detached = self.create_subscription(
            RosString, f"/robot_bridge/detached/{agent}/{self.robot}", self._on_scoped_detached, 10
        )

        # announce intent to hub (JSON control; data remains plain strings)
        payload = f'{{"agent":"{agent}","robot":"{self.robot}"}}'
        self.pub_attach.publish(RosString(data=payload))
        print(f"[attach] requested -> agent={agent}")

    def detach(self):
        if not self.agent:
            print("Not attached.")
            return
        with self._pending_lock:
            self._pending_ack.clear()
        payload = f'{{"agent":"{self.agent}","robot":"{self.robot}"}}'
        self.pub_detach.publish(RosString(data=payload))
        print(f"[detach] requested -> agent={self.agent}")

    # ---------- Scoped confirm handlers ----------
    def _on_scoped_attached(self, msg: RosString):
        text = (msg.data or "").strip()
        if text:
            self.rec_seq.append(text)
            print(f"[hub-confirm] {text}")
            self._attach_ok_event.set()

    def _on_scoped_detached(self, msg: RosString):
        text = (msg.data or "").strip()
        if text:
            self.rec_seq.append(text)
            print(f"[hub-detach] {text}")
        self._cleanup_agent_bindings()
        self.agent = None

    # ---------- Receive: DOWN ----------
    def _on_down_string(self, ros_msg: RosString):
        text = (ros_msg.data or "").strip()
        if not text:
            return
        self.rec_seq.append(text)
        print(f"[down] {text}")

        if _norm(text) == "start":
            if self._start_mode == "fixed":
                choice = self._start_fixed_cmd.strip()
                if not choice:
                    print("[auto] start → fixed command empty; ignoring")
                    return
                print(f"[auto] start (fixed) → send '{choice}'")
                self.send_text(choice)
                return
            else:
                if not self._builtins:
                    print("[auto] start → no YAML commands loaded; random mode is idle.")
                    print("[hint] Use 'commands load <PATH>' or switch to 'onstart fixed <NAME>'")
                    return
                choice = self._rng.choice(self._builtins)
                print(f"[auto] start (random) → send '{choice}'")
                self.send_text(choice)
                return

        threading.Thread(target=self._execute, args=(text,), daemon=True).start()

    # ---------- Receive: ACK ----------
    def _on_ack_string(self, ros_msg: RosString):
        raw = (ros_msg.data or "").strip()
        if not raw:
            return
        self.rec_seq.append(raw)
        print(f"[ack] {raw}")

        low = raw.lower()
        if not low.startswith("ack:"):
            return
        original = raw[4:].strip()
        if not original:
            return

        found = False
        with self._pending_lock:
            try:
                for _ in range(len(self._pending_ack)):
                    s = self._pending_ack[0].strip()
                    if s == original:
                        self._pending_ack.popleft()
                        found = True
                        break
                    else:
                        self._pending_ack.rotate(-1)
            except Exception:
                pass

        if found:
            threading.Thread(target=self._execute, args=(original,), daemon=True).start()
        else:
            print("[ack] ignored (not pending here)")

    # ---------- Execute (resolve via YAML map or run raw) ----------
    def _execute(self, text: str):
        """
        Resolve 'text' via YAML map (name → shell template) if present.
        Placeholders:
          {robot} → robot name        e.g., R1
          {base}  → /{robot}/cmd_vel e.g., /R1/cmd_vel
        If 'text' not in the map, run it as a raw shell command.
        """
        name = (text or "").strip()
        base = f"/{self.robot}/cmd_vel"

        if name in self._cmd_map:
            template = self._cmd_map[name]
            try:
                cmd_to_run = template.format(robot=self.robot, base=base)
            except KeyError as e:
                with self._print_lock:
                    print(f"[exec-map] formatting error for '{name}': missing placeholder {e}")
                return
            with self._print_lock:
                print(f"[exec-map] {name} → {cmd_to_run}")
        else:
            cmd_to_run = name
            with self._print_lock:
                print(f"[exec-any] {cmd_to_run}")

        rc, out, err = exec_shell_any(cmd_to_run)

        with self._print_lock:
            if out.strip():
                print(out.rstrip())
            if err.strip():
                for line in err.rstrip().splitlines():
                    print(f"[stderr] {line}")
            print(f"[exit] {rc}")

    # ---------- REPL ----------
    def _help(self):
        print("""
Robot REPL
  attach <AGENT>          attach to an agent
  send <TEXT...>          send a plain string to the agent (UP)
  list sent               list sent strings (order)
  list rec                list received lines (down + ack + confirms)

  commands show           show YAML file path and commands (name → shell)
  commands load <PATH>    load YAML commands from file
  commands reload         reload the last loaded YAML commands file

  onstart random          set 'start' behavior to random choice from YAML names
  onstart fixed <NAME>    set 'start' behavior to always send <NAME>
  onstart show            show current 'start' mode and fixed name (if any)

  setenv KEY=VALUE        set env for future execs (e.g., ROS_SETUP=/opt/ros/humble/setup.bash)
  showenv                 print key env vars (ROS_SETUP, ROS_DOMAIN_ID, PATH, RMW_IMPL)
  detach                  request detach from current agent
  help                    show this help
  quit                    exit
""".strip())

    def run_repl(self):
        time.sleep(0.2)
        self._help()
        while rclpy.ok():
            try:
                sys.stdout.write("robot> "); sys.stdout.flush()
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.1); continue
                parts = line.strip().split()
                if not parts: continue

                cmd, *args = parts
                if cmd == "quit":
                    self._graceful_shutdown()
                    break
                elif cmd == "help":
                    self._help()
                elif cmd == "attach":
                    if not args:
                        print("usage: attach <AGENT>"); continue
                    self.attach(args[0])
                elif cmd == "detach":
                    self.detach()
                elif cmd == "send":
                    if not args:
                        print("usage: send <TEXT...>"); continue
                    text = " ".join(args)
                    self.send_text(text)
                elif cmd == "list":
                    if not args:
                        print("usage: list [sent|rec]"); continue
                    which = args[0].lower()
                    if which == "sent":
                        print("--- sent ---")
                        for i, s in enumerate(self.sent_seq, 1):
                            print(f"  {i}. {s}")
                    elif which == "rec":
                        print("--- received ---")
                        for i, s in enumerate(self.rec_seq, 1):
                            print(f"  {i}. {s}")
                    else:
                        print("usage: list [sent|rec]")
                elif cmd == "setenv":
                    if not args or "=" not in args[0]:
                        print('usage: setenv KEY=VALUE   (example: setenv ROS_SETUP=/opt/ros/humble/setup.bash)')
                        continue
                    k, v = args[0].split("=", 1)
                    os.environ[k] = v
                    print(f"[env] {k}={v}")
                elif cmd == "showenv":
                    print(f"ROS_SETUP={os.environ.get('ROS_SETUP','')}")
                    print(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID','')}")
                    print(f"RMW_IMPL={os.environ.get('RMW_IMPLEMENTATION','')}")
                    print(f"PATH={os.environ.get('PATH','')}")
                elif cmd == "commands":
                    if not args:
                        print("usage: commands [show|load <PATH>|reload]")
                        continue
                    sub = args[0].lower()
                    if sub == "show":
                        print(f"[commands] file={getattr(self, '_commands_file', '') or '(none)'}")
                        if not self._cmd_map:
                            print("(no commands loaded) — running in UNMAPPED mode.")
                            print("You can still execute unmapped text as raw shells.")
                        else:
                            print("--- commands (name → shell) ---")
                            for i, k in enumerate(self._builtins, 1):
                                print(f"  {i}. {k}  →  {self._cmd_map[k]}")
                    elif sub == "load":
                        if len(args) < 2:
                            print("usage: commands load <PATH>")
                            continue
                        path = " ".join(args[1:]).strip()
                        try:
                            self.load_commands(path)
                        except Exception as e:
                            print(f"[commands] load error: {e}")
                            print("[note] Staying in UNMAPPED mode; you can still run unmapped text as raw shells.")
                    elif sub == "reload":
                        try:
                            self.load_commands()
                        except Exception as e:
                            print(f"[commands] reload error: {e}")
                            print("[note] Staying in UNMAPPED mode; you can still run unmapped text as raw shells.")
                    else:
                        print("usage: commands [show|load <PATH>|reload]")
                elif cmd == "onstart":
                    if not args:
                        print("usage: onstart [show|random|fixed <NAME...>]")
                        continue
                    sub = args[0].lower()
                    if sub == "show":
                        mode = self._start_mode
                        print(f"[onstart] mode={mode}" +
                              (f" fixed_name='{self._start_fixed_cmd}'" if mode == "fixed" else ""))
                    elif sub == "random":
                        self.start_mode = "random"
                        if not self._builtins:
                            print("[note] No YAML names loaded; random 'start' will do nothing until you load a file.")
                    elif sub == "fixed":
                        if len(args) < 2:
                            print("usage: onstart fixed <NAME...>")
                            continue
                        name = " ".join(args[1:]).strip()
                        try:
                            self.start_fixed_name = name
                        except ValueError as e:
                            print(f"[onstart] {e}")
                            continue
                        self.start_mode = "fixed"
                    else:
                        print("usage: onstart [show|random|fixed <NAME...>]")
                else:
                    self._help()
            except Exception as e:
                print(f"[REPL error] {e}")
                time.sleep(0.1)

    # ---------- Startup interactive flow (Agent spawn + auto-attach) ----------
    def startup_interactive(self):
        print(f"🤖 RobotNode '{self.robot}' ready.")
        yn = input("Do you want to create an agent for this robot? [Y/n]: ").strip().lower()
        if yn in ("", "y", "yes"):
            default_agent = f"{self.robot}_agent"
            name = input(f"Enter agent name (default = {default_agent}): ").strip()
            agent_name = name if name else default_agent

            _spawn_agent_terminal(agent_name)

            self.attach(agent_name)
            t_start = time.time()
            total_timeout = 20.0
            resend_every = 0.5
            payload = f'{{"agent":"{agent_name}","robot":"{self.robot}"}}'
            while not self._attach_ok_event.is_set() and (time.time() - t_start < total_timeout):
                self.pub_attach.publish(RosString(data=payload))
                time.sleep(resend_every)

            if self._attach_ok_event.is_set():
                print(f"[attach] confirmed with agent '{agent_name}'.")
            else:
                print("[attach] no confirmation yet (agent might still be starting). You can continue; topics are bound.")
        else:
            print("Skipping agent creation. Use 'attach <AGENT>' later to bind to an existing agent.")

    # ---------- Shutdown ----------
    def _graceful_shutdown(self):
        try:
            if self.agent:
                self.detach()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass


def main():
    ap = argparse.ArgumentParser(description="ProjectN Robot Node (YAML-optional; scoped I/O; ack-gated exec; start switch; agent auto-spawn)")
    ap.add_argument("--robot", required=True, help="Robot name")
    ap.add_argument("--commands", help="Path to YAML commands file (overrides START_COMMANDS_FILE)")
    args = ap.parse_args()

    rclpy.init()
    node = RobotNode(args.robot, commands_file=args.commands)
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
