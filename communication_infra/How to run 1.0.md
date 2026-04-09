# How to Run communication infra (Robot + Agent Hub + Tree)

This guide shows **every practical way** to launch and test your current setup with the new files you have:

* `robot_node.py` (auto-spawn optional, YAML command map, ack-gated exec)
* `agent_hub_robot.py` (numeric attach menu, strict W-queue, runtime delay control)
* Your existing **Tree** (the message-tree that seeds `nid` and routes D messages)

---

## 0) Prerequisites & Shell Setup

1. **Source your ROS 2** in every terminal:

   ```bash
   source /opt/ros/humble/setup.bash
   # or jazzy/etc
   ```
2. **Recommended env vars** (optional):

   ```bash
   export ROS_DOMAIN_ID=77               # pick any, but use the SAME for Tree/Agent/Robot
   # export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # only if you intentionally change DDS vendor
   ```
3. **Optional exec helpers** for the Robot’s shell commands:

   ```bash
   export ROS_SETUP=/opt/ros/humble/setup.bash  # robot will source this before running 'ros2 ...'
   export EXEC_TIMEOUT=60                       # seconds for each executed shell
   ```
4. **(If you use a YAML commands map)** make sure PyYAML is installed where you run the robot:

   ```bash
   pip install pyyaml
   ```

---

## 1) Launch the Tree (always first)

Run your tree launcher as you normally do, e.g.:

```bash
ros2 run projectn projectn-launch-random-tree -- --root Root --nodes 6
```

The Tree publishes discovery (`/<node>/viz/status`), provides `get_msg_id` and `submit_dm`, and **sends SEED**.

---

## 2) Start the Agent Hub Robot (two ways)

### A) Interactively (recommended during development)

Just run it:

```bash
ros2 run projectn agent_hub_robot --agent A1
```

What happens:

* It shows a **numbered attach menu** *first* (no REPL yet).
* Pick a TreeNode by number, type a name, refresh, or skip.
* After that, the **minimal REPL** appears:

```
[AGENT REPL]
  1) Detach robot
  2) Detach from tree
  3) Attach to tree
  4) List queues
  5) List attached robots
  6) Show delay
  7) Set delay
  8) Quit
```

#### Agent flags you can pass:

* `--agent <NAME>` – required.
* `--d-delay-s <FLOAT>` – **delay after each D** (UP or DOWN). Default `30`. Set `0` for no delay.
* `--heartbeat-hz <FLOAT>` – status heartbeat rate (default `1.0`).
* `--attach-timeout-s <FLOAT>` – wait time for services on attach (default `10`).

You can **change the delay at runtime** (REPL option **7**). It applies to the **next** D processed.

### B) Spawned automatically by the Robot (hands-free)

If you let the Robot auto-spawn an agent (see §3-B), the Agent comes up in **another terminal** with the same env and immediately shows the **numeric attach menu**. Pick your TreeNode there and you’re set.

---

## 3) Start the Robot (all run modes)

### A) Robot only (no agent spawn)

```bash
ros2 run projectn robot_node --robot R1
```

On startup the robot asks:

```
Do you want to create an agent for this robot? [Y/n]:
```

Type **n** (or press Enter on **Y** to skip if you prefer).
Then **manually** attach later:

```
robot> attach A1
```

### B) Robot with **auto-spawn agent** (interactive naming)

```bash
ros2 run projectn robot_node --robot R1
```

Answer the prompts:

```
Do you want to create an agent for this robot? [Y/n]: y
Enter agent name (default = R1_agent): A1
```

* A **new terminal** opens and runs:

  ```
  ros2 run projectn agent_hub_robot --agent A1
  ```
* Robot **auto-attaches** to `A1` and retries until it receives an attach confirm.
* In the Agent terminal, choose the TreeNode from the **numbered menu**.

> Tip: If the new terminal **doesn’t open**, install a terminal emulator (e.g., `sudo apt install gnome-terminal`) or start the Agent manually in another terminal.

### C) Robot with a YAML commands map

The robot **prefers** a YAML file that maps human-friendly names to shell commands.

#### Pass YAML via CLI:

```bash
ros2 run projectn robot_node --robot R1 --commands ~/.projectn/robot_commands.yaml
```

#### Or via environment:

```bash
export START_COMMANDS_FILE=~/.projectn/robot_commands.yaml
ros2 run projectn robot_node --robot R1
```

#### YAML formats supported

**Mapping (top-level):**

```yaml
forward:  timeout 5 ros2 topic pub -r 20 {base} geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
turn left: timeout 5 ros2 topic pub -r 20 {base} geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.2}}"
echo hello: echo "hello from {robot}"
```

**Object list under `commands`:**

```yaml
commands:
  - name: forward
    shell: timeout 5 ros2 topic pub -r 20 {base} geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
  - name: turn left
    shell: timeout 5 ros2 topic pub -r 20 {base} geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.2}}"
  - name: echo hello
    shell: echo "hello from {robot}"
```

**Placeholders** expanded at runtime:

* `{robot}` → e.g., `R1`
* `{base}`  → `/{robot}/cmd_vel` → e.g., `/R1/cmd_vel`

**At runtime in the robot REPL:**

```
robot> commands show
robot> commands load /path/to/file.yaml
robot> commands reload
```

If YAML **is missing or broken**, the robot goes into **UNMAPPED mode**:

* You can still `send` any text; it executes as a raw shell command.
* `onstart random` will print a hint (no YAML names to choose from).

---

## 4) Robot REPL — All Commands

```
attach <AGENT>          # attach to agent (scoped topics created)
detach                  # request detach from current agent
send <TEXT...>          # send a plain string UP (executes only after ACK)
list sent               # list sent strings
list rec                # list all received lines (down + ack + confirms)

commands show           # show YAML file & mappings
commands load <PATH>    # load YAML commands from file
commands reload         # reload the last loaded YAML file

onstart random          # DOWN 'start' → send a random YAML name UP (no-op if no YAML)
onstart fixed <NAME>    # DOWN 'start' → always send <NAME> UP
onstart show            # show current start-mode and fixed name

setenv KEY=VALUE        # set env for future execs (e.g., ROS_SETUP=/opt/ros/humble/setup.bash)
showenv                 # print key env vars used during execution
help                    # show help
quit                    # exit robot
```

**ACK-gated execution:**

* `send <X>` enqueues `<X>` pending.
* Only when the robot receives `ack: X` (from its Agent on the scoped `/ack/agent/robot` topic) does it **execute** X (YAML-mapped or raw).

**DOWN ‘start’ behavior:**

* If the robot receives `start` from its Agent:

  * `onstart random` → picks a random **YAML name** and `send`s it **UP**.
  * `onstart fixed NAME` → always `send` **NAME** UP.
  * In **UNMAPPED** mode, `random` prints a hint (no names to choose).

---

## 5) Agent REPL — All Commands (numbered)

After the startup attach menu finishes, you get:

```
[AGENT REPL]
  1) Detach robot           # choose a robot to detach (scoped confirm is published)
  2) Detach from tree       # drop tree bindings; nid reset; keeps robot registry
  3) Attach to tree         # open the numeric attach menu again
  4) List queues            # snapshot of I and W queues (with nid status)
  5) List attached robots   # print current robot list
  6) Show delay             # print current D-delay (seconds)
  7) Set delay              # update D-delay (float >= 0), applies to the next D
  8) Quit                   # close REPL (node keeps running)
```

**Strict order:**

* The agent waits for **SEED** (`expected_id`) from the Tree, sets `nid`, then processes **W** in **ascending id**.
* For **DOWN** fan-out to multiple robots, `nid` increments **once** per D.

**D-delay:**

* After every processed D (UP or DOWN), the agent sleeps for `d-delay-s` seconds.
* Change it live with option **7**.

---

## 6) Typical Workflows

### A) Fast local test (one machine, single robot)

1. **Tree**:

   ```bash
   ros2 run projectn projectn-launch-random-tree -- --root Root --nodes 6
   ```
2. **Robot**:

   ```bash
   ros2 run projectn robot_node --robot R1 --commands ~/.projectn/robot_commands.yaml
   ```

   * Answer **Y** to spawn agent, name it **A1**.
3. **Agent terminal**:

   * Pick a node from the numeric menu.
   * Optionally set delay (7 → `0` for fastest tests).
4. **Robot terminal**:

   * `send forward`
   * Watch for `ack: forward`, then see it execute.

### B) Multiple robots to the same agent

* Start Agent once:

  ```bash
  ros2 run projectn agent_hub_robot --agent A1 --d-delay-s 0
  ```
* Start robot R1 and **skip** auto-spawn:

  ```bash
  ros2 run projectn robot_node --robot R1
  robot> attach A1
  ```
* Start robot R2 and attach to A1 as well:

  ```bash
  ros2 run projectn robot_node --robot R2
  robot> attach A1
  ```
* DOWN from Tree is **fanned out** to all attached robots (`/out/A1/<robot>`).
  UP from each robot is acked **only** back to the sender.

### C) Two machines (same LAN)

* Ensure both machines share the **same ROS_DOMAIN_ID** and compatible DDS (default is fine).
* If using Fast DDS Router/bridges, start them first.
* Run Tree on machine A; Agent + Robot on machine B (or spread them out).
* Attach normally; numeric menus work the same.

---

## 7) YAML Management (from terminal vs code)

### Set the YAML file from the **terminal**

* **CLI flag** when launching the robot:

  ```bash
  ros2 run projectn robot_node --robot R1 --commands ~/mycmds.yaml
  ```
* **Environment variable** (no flag):

  ```bash
  export START_COMMANDS_FILE=~/mycmds.yaml
  ros2 run projectn robot_node --robot R1
  ```
* **Robot REPL** after launch:

  ```
  robot> commands load ~/mycmds.yaml
  robot> commands show
  robot> commands reload
  ```

### “Edit its cd from the code” (interpreting your request)

If you mean **change the robot’s working directory for command execution**:

* Either **cd** inside the YAML shell:

  ```yaml
  echo log:
    shell: 'cd ~/logs && echo "hello from {robot}" >> run.log'
  ```
* Or wrap the shell with a prefix (global or per-command), e.g. in YAML:

  ```yaml
  forward: 'cd ~/ws && timeout 5 ros2 topic pub -r 20 {base} geometry_msgs/Twist "{linear: {x:0.1}, angular:{z:0.0}}"'
  ```
* If you want a **global base directory**, add an env var and use it:

  ```bash
  export BASE_DIR=~/ws
  ```

  Then in YAML:

  ```yaml
  forward: 'cd "$BASE_DIR" && timeout 5 ros2 topic pub -r 20 {base} geometry_msgs/Twist "{linear: {x:0.1}, angular:{z:0.0}}""
  ```

> Note: The robot **does not** implicitly `cd` for you; it runs each command as given. Use `cd ... &&` in your YAML/commands.

---

## 8) Advanced: “start” Triggers

If your Tree sends the string `start` **DOWN**, the robot reacts according to **onstart**:

* `robot> onstart random`
  Robot chooses a **random YAML name** and `send`s it **UP** (requires YAML loaded).
* `robot> onstart fixed forward`
  Robot always `send`s `forward` **UP**.

**Important:** “start” only **sends UP**. Execution of that command still happens **after the matching ACK**.

---

## 9) Troubleshooting

* **Agent terminal doesn’t open when auto-spawning**

  * Install a terminal emulator: `sudo apt install gnome-terminal` (or `xterm`, `xfce4-terminal`, etc.).
  * As a fallback, start the agent manually in another terminal:

    ```bash
    ros2 run projectn agent_hub_robot --agent A1
    ```

    Then in the robot: `attach A1`.

* **No SEED → W queue doesn’t progress**

  * The Tree must send SEED (`expected_id`). Make sure the agent attached to a valid TreeNode.
  * Re-attach from the agent REPL (**3**) if the services were not ready.

* **No ACK → robot never executes the command**

  * The agent must process UP D messages in order and publish `ack: <text>` to the **sender’s** scoped ACK topic.
  * Check Agent **W queue** (REPL **4**) and **D-delay** (REPL **6/7**). Reduce delay to `0` when testing.

* **YAML not loaded / random ‘start’ is idle**

  * `robot> commands load /path/to/cmds.yaml`
  * `robot> commands show`

* **Different machines can’t discover each other**

  * Verify `ROS_DOMAIN_ID` is the **same** everywhere.
  * Stay on the default DDS unless you know you changed it; if you changed `RMW_IMPLEMENTATION`, set it everywhere.

---

## 10) Quick Reference (copy-paste)

**Tree**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
ros2 run projectn projectn-launch-random-tree -- --root Root --nodes 6
```

**Agent (manual)**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
ros2 run projectn agent_hub_robot --agent A1 --d-delay-s 0
# pick TreeNode from numeric menu
```

**Robot (auto-spawn Agent A1 and use YAML)**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
export START_COMMANDS_FILE=~/.projectn/robot_commands.yaml
export ROS_SETUP=/opt/ros/humble/setup.bash
ros2 run projectn robot_node --robot R1
# answer Y, name agent A1
```

**Robot (no spawn; attach later)**

```bash
ros2 run projectn robot_node --robot R2
robot> attach A1
```

---

