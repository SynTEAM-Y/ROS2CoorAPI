#!/usr/bin/env python3

# ProjectN • Tree Launcher (interactive)
# Adds:
#   • --node-log-level (default DEBUG) for your nodes
#   • --infra-log-level (default WARN) for infra loggers (rcl, rclpy, rmw_*)

import argparse, json, os, random, time, subprocess, signal, sys, pathlib

# ───────────────────────────────────────────────────────────────────
# Config
# ───────────────────────────────────────────────────────────────────
ROSTER_DIR = os.path.expanduser("~/.projectn")
TREE_FILE  = os.path.join(ROSTER_DIR, "tree.json")

DEFAULT_PKG = os.environ.get("PROJECTN_PKG", "projectn")
DEFAULT_EXE = os.environ.get("PROJECTN_NODE_EXE", "server")
DEFAULT_LOG_LEVEL = os.environ.get("PROJECTN_NODE_LOG_LEVEL", "DEBUG")
DEFAULT_INFRA_LEVEL = os.environ.get("PROJECTN_INFRA_LOG_LEVEL", "WARN")
SERVICE_WAIT_SEC = float(os.environ.get("PROJECTN_SVC_WAIT", "8.0"))

# ───────────────────────────────────────────────────────────────────
# ROS probe (tiny client just to wait for services)
# ───────────────────────────────────────────────────────────────────
ROS_READY = True
try:
    import rclpy
    from rclpy.node import Node
    from projectn_interfaces.srv import GetMsgId, SubmitDm
except Exception as e:
    ROS_READY = False
    _ROS_IMPORT_ERR = repr(e)

class _Probe(Node):
    def __init__(self):
        super().__init__("projectn_launch_random_probe")
        self._getid = {}
        self._submit = {}
    def _cli_getid(self, node):
        c = self._getid.get(node)
        if c is None:
            c = self.create_client(GetMsgId, f"/{node}/get_msg_id")
            self._getid[node] = c
        return c
    def _cli_submit(self, node):
        c = self._submit.get(node)
        if c is None:
            c = self.create_client(SubmitDm, f"/{node}/submit_dm")
            self._submit[node] = c
        return c
    def wait_both(self, node: str, timeout_sec: float) -> bool:
        t0 = time.time()
        getid = self._cli_getid(node)
        submit = self._cli_submit(node)
        while time.time() - t0 < timeout_sec and rclpy.ok():
            g = getid.wait_for_service(timeout_sec=0.25)
            s = submit.wait_for_service(timeout_sec=0.25)
            if g and s:
                return True
        return False

# ───────────────────────────────────────────────────────────────────
# Tree build helpers
# ───────────────────────────────────────────────────────────────────
def build_random_tree_simple(root_name: str, total_nodes: int, seed: int) -> dict:
    rng = random.Random(seed)
    names = [root_name]
    idx = 1
    while len(names) < total_nodes:
        names.append(f"A{idx}"); idx += 1
    parents = {root_name: ""}
    for n in names[1:]:
        parent = rng.choice(list(parents.keys()))
        parents[n] = parent
    return parents

def build_random_tree_complex(root_name: str, seed: int,
                              min_nodes: int = 12,
                              max_nodes: int = 40,
                              max_depth: int = 6,
                              max_children_per_node: int = 5) -> dict:
    rng = random.Random(seed)
    total = rng.randint(min_nodes, max_nodes)
    names = [root_name]
    i = 1
    while len(names) < total:
        names.append(f"A{i}"); i += 1
    parents = {root_name: ""}
    frontier = [(root_name, 1)]
    remaining = set(names[1:])
    while frontier and remaining:
        node, depth = frontier.pop(0)
        if depth > max_depth:
            attachable = [n for n in parents.keys()]
            while remaining:
                child = remaining.pop()
                parents[child] = rng.choice(attachable)
            break
        capacity = rng.randint(0, max_children_per_node)
        take = min(capacity, len(remaining))
        if take == 0:
            continue
        batch = []
        for _ in range(take):
            child = remaining.pop()
            batch.append(child)
            parents[child] = node
        for c in batch:
            frontier.append((c, depth + 1))
    while remaining:
        child = remaining.pop()
        parents[child] = rng.choice(list(parents.keys()))
    return parents

def build_custom_tree_interactive() -> dict:
    print("\n[custom] Build your tree interactively.")
    while True:
        root = input("Root node name [default: Root]: ").strip() or "Root"
        if root: break
    parents = {root: ""}
    print("Add children. Leave node name empty to finish.")
    while True:
        name = input(" New node name (blank to finish): ").strip()
        if name == "": break
        if name in parents:
            print("  ! Name already exists; choose another."); continue
        if name.upper() == "R":
            print("  ! 'R' reserved; choose another."); continue
        parent = input(f"  Parent for {name} (existing node, default: {root}): ").strip() or root
        if parent not in parents:
            print("  ! Parent not found; add the parent first or choose an existing parent."); continue
        parents[name] = parent
    print("\n[custom] Summary:"); print_tree(parents)
    confirm = input("Launch this tree? [y/N]: ").strip().lower()
    if confirm != "y":
        print("[custom] Aborted by user."); sys.exit(0)
    return parents

def print_tree(parents: dict):
    roots = [n for n, p in parents.items() if p == ""]
    print(" Nodes (node -> parent):")
    for n in sorted(parents.keys()):
        print(f"  {n} -> {parents[n] if parents[n] else '(root)'}")
    print(f" Total: {len(parents)}; Root(s): {', '.join(roots)}")

# ───────────────────────────────────────────────────────────────────
# File I/O
# ───────────────────────────────────────────────────────────────────
def ensure_roster_dir():
    pathlib.Path(ROSTER_DIR).mkdir(parents=True, exist_ok=True)

def write_tree(parents: dict):
    ensure_roster_dir()
    with open(TREE_FILE, "w") as f:
        json.dump(parents, f, indent=2)
    print(f"[launcher] wrote tree to {TREE_FILE}")

# ───────────────────────────────────────────────────────────────────
# Process spawn / wait
# ───────────────────────────────────────────────────────────────────
def spawn_node(pkg: str, exe: str, name: str, parent: str, node_level: str, infra_level: str):
    """
    Start one ProjectN node via `ros2 run` with:
      - per-node log level (your app)
      - per-infra logger level (quiet rcl/rclpy/rmw)
    """
    args = [
        "ros2", "run", pkg, exe,
        "--ros-args",
        "-p", f"name:={name}",
        "-p", f"is_root:={'true' if not parent else 'false'}",
    ]
    if parent:
        args += ["-p", f"parent:={parent}"]

    # Your node's default log level
    args += ["--log-level", node_level]

    # Silence infra noise (still overridable via --infra-log-level)
    args += ["--log-level", f"rcl:={infra_level}"]
    args += ["--log-level", f"rclpy:={infra_level}"]
    args += ["--log-level", f"rmw_fastrtps_cpp:={infra_level}"]
    args += ["--log-level", f"rmw_cyclonedds_cpp:={infra_level}"]

    return subprocess.Popen(args)

def launch_processes(parents: dict, pkg: str, exe: str,
                     wait_sec: float, stagger_ms: int,
                     node_level: str, infra_level: str):
    rclpy.init()
    probe = _Probe()
    procs = {}
    try:
        # Roots first
        for n, p in parents.items():
            if p == "":
                print(f"[launcher] spawn {n} (root)")
                procs[n] = spawn_node(pkg, exe, n, "", node_level, infra_level)
                print(f"[launcher] {n} pid={procs[n].pid}")
                if not probe.wait_both(n, timeout_sec=wait_sec):
                    rc = procs[n].poll()
                    extra = f" exited rc={rc}" if rc is not None else ""
                    print(f"[ERR] {n} services not ready{extra}", file=sys.stderr)
                    sys.exit(1)
                print(f"[OK] {n} services ready")
                time.sleep(stagger_ms/1000.0)

        # Then children
        for n, p in parents.items():
            if p != "":
                print(f"[launcher] spawn {n} (parent={p})")
                procs[n] = spawn_node(pkg, exe, n, p, node_level, infra_level)
                print(f"[launcher] {n} pid={procs[n].pid}")
                if not probe.wait_both(n, timeout_sec=wait_sec):
                    rc = procs[n].poll()
                    extra = f" exited rc={rc}" if rc is not None else ""
                    print(f"[ERR] {n} services not ready{extra}", file=sys.stderr)
                    sys.exit(1)
                print(f"[OK] {n} services ready")
                time.sleep(stagger_ms/1000.0)

        print("\n[launcher] running. Ctrl+C to stop all.")
        while True:
            for name, p in list(procs.items()):
                if p.poll() is not None:
                    print(f"[launcher] {name} exited (rc={p.returncode})")
                    procs.pop(name, None)
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n[launcher] stopping...")
    finally:
        for name, p in list(procs.items()):
            try: p.send_signal(signal.SIGINT)
            except Exception: pass
        time.sleep(0.5)
        for name, p in list(procs.items()):
            try: p.terminate()
            except Exception: pass
        if rclpy.ok():
            rclpy.shutdown()

# ───────────────────────────────────────────────────────────────────
# Main
# ───────────────────────────────────────────────────────────────────
def main():
    if not ROS_READY:
        print(f"[FATAL] ROS2 imports failed: {_ROS_IMPORT_ERR}", file=sys.stderr)
        sys.exit(2)

    ap = argparse.ArgumentParser(description="ProjectN Tree Launcher (interactive)")
    ap.add_argument("--pkg", default=DEFAULT_PKG, help="ROS 2 package (ros2 run <pkg> <exe>)")
    ap.add_argument("--exe", default=DEFAULT_EXE, help="ROS 2 executable in the package")
    ap.add_argument("--wait", type=float, default=SERVICE_WAIT_SEC, help="seconds to wait per node for services")
    ap.add_argument("--stagger-ms", type=int, default=400, help="delay between spawns")
    ap.add_argument("--seed", type=int, default=42, help="seed for random generators")
    ap.add_argument("--mode", type=int, choices=[1,2,3], help="1=simple, 2=complex, 3=custom")
    ap.add_argument("--root", default="Root", help="root name (used in simple/complex if provided)")
    ap.add_argument("--nodes", type=int, help="for mode=1, total nodes incl. root (default 6)")
    # New flags
    ap.add_argument("--node-log-level", default=DEFAULT_LOG_LEVEL,
                    help="ROS log level for spawned nodes (DEBUG/INFO/WARN/ERROR)")
    ap.add_argument("--infra-log-level", default=DEFAULT_INFRA_LEVEL,
                    help="Infra logger level for rcl/rclpy/rmw_* (DEBUG/INFO/WARN/ERROR)")

    args = ap.parse_args()

    # Choose mode (menu if not provided)
    mode = args.mode
    if mode is None:
        print("\nChoose a launch mode:")
        print("  1) Simple random tree (6 nodes including root)")
        print("  2) Very complicated random tree (deep & wide)")
        print("  3) Build custom tree")
        while True:
            pick = input("Select [1/2/3]: ").strip()
            if pick in ("1","2","3"):
                mode = int(pick); break
            print("Please type 1, 2, or 3.")

    # Build parents map
    if mode == 1:
        total = args.nodes if args.nodes and args.nodes >= 2 else 6
        parents = build_random_tree_simple(args.root, total_nodes=total, seed=args.seed)
        print("\n[simple] Generated tree:"); print_tree(parents)
    elif mode == 2:
        print("\n[complex] Defaults: min_nodes=12, max_nodes=40, max_depth=6, max_children_per_node=5")
        tweak = input("Change defaults? [y/N]: ").strip().lower()
        if tweak == "y":
            def ask_int(prompt, default):
                s = input(f"{prompt} [{default}]: ").strip()
                return int(s) if s.isdigit() else default
            min_nodes  = ask_int("  min_nodes", 12)
            max_nodes  = ask_int("  max_nodes", 40)
            max_depth  = ask_int("  max_depth", 6)
            max_fanout = ask_int("  max_children_per_node", 5)
        else:
            min_nodes, max_nodes, max_depth, max_fanout = 12, 40, 6, 5
        parents = build_random_tree_complex(args.root, args.seed, min_nodes, max_nodes, max_depth, max_fanout)
        print("\n[complex] Generated tree:"); print_tree(parents)
    else:
        parents = build_custom_tree_interactive()

    # Persist and launch
    write_tree(parents)
    launch_processes(parents,
                     pkg=args.pkg, exe=args.exe,
                     wait_sec=args.wait, stagger_ms=args.stagger_ms,
                     node_level=args.node_log_level, infra_level=args.infra_log_level)

if __name__ == "__main__":
    main()
