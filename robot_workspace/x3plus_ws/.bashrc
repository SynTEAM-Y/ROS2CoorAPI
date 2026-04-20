# =========================
# ~/.bashrc  (Jetson + ROS 2 Humble + X3 Plus)
# =========================

# Exit early if this is not an interactive shell (Keeps non-interactive scripts fast)
[[ $- != *i* ]] && return

# ----- History & basics -----
export HISTSIZE=20000                    # Keeps more commands in memory
export HISTFILESIZE=40000                # Keeps more commands on disk
export HISTCONTROL=ignoredups:erasedups  # Skips duplicates in history
export HISTTIMEFORMAT="%F %T "           # Shows timestamps in `history`
export LANG=C.UTF-8                      # Sets sane locale
export LC_ALL=C.UTF-8
export PATH="$HOME/bin:$PATH"            # Adds ~/bin to PATH
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"  # Simplify ROS 2 log output format
export RCUTILS_COLORIZED_OUTPUT=1

# ----- Bash completion -----
# Enables tab completion if available (Handy for ros2, colcon, git, etc.)
if [ -f /etc/bash_completion ]; then
  . /etc/bash_completion
fi

# =========================
# Colored prompt (Ubuntu style)
# =========================
force_color_prompt=yes
if [ -n "$force_color_prompt" ]; then
  if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
    color_prompt=yes
  else
    color_prompt=
  fi
fi
if [ "$color_prompt" = yes ]; then
  PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
  PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# Sets terminal title to "user@host: cwd" for xterm/rxvt
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)  ;;
esac

# =========================
# Color for ls/grep + handy aliases
# =========================
if [ -x /usr/bin/dircolors ]; then
  test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
  alias ls='ls --color=auto'
  alias grep='grep --color=auto'
  alias fgrep='fgrep --color=auto'
  alias egrep='egrep --color=auto'
fi

# Loads extra aliases from ~/.bash_aliases if present
if [ -f ~/.bash_aliases ]; then
  . ~/.bash_aliases
fi

# =========================
# ROS 2 base (Humble)
# =========================
# Sources the base ROS 2 distro so `ros2` and message types are available
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# =========================
# Robot environment (Edit these for your setup)
# =========================
export ROS_DOMAIN_ID=42       # Isolates DDS network; change if you have conflicts
export ROS_LOCALHOST_ONLY=0   # Allows network discovery
export ROBOT=                 # Update to the correct name of the robot. ROBOT=PREFIX+ROBOT_ID for example ROBOT=robot123
export ROBOT_TYPE=X3plus
export RPLIDAR_TYPE=a1
export CAMERA_TYPE=astraplus
export REPO_PATH="$HOME/ROS2CoorAPI/robot_workspace" # Location of the git repository. 

# =========================
# Workspace helpers (Manual, explicit overlays)
# =========================
# Resets ROS overlay variables when switching projects
reset_ros_env() {
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
  unset LD_LIBRARY_PATH PYTHONPATH
}

# Safely sources a workspace overlay if it's built
_use_ws() {
  local ws="$1"
  if [ -f "$ws/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "$ws/install/setup.bash"
  else
    echo "[warn] Workspace not built: $ws"
    return 1
  fi
}

# Declares paths to your workspaces (Adjust if your paths differ)
# export YDLIDAR_WS="$REPO_PATH/ydlidar_ws"
export X3PLUS_WS="$REPO_PATH/x3plus_ws"

# Helper functions to source overlays on demand
# use_ydlidar() { _use_ws "$YDLIDAR_WS"; }
# use_x3plus()  { _use_ws "$X3PLUS_WS"; }

# Convenience function: sources base + ydlidar + x3plus (In correct order)
use_robot() {
  _use_ws "$X3PLUS_WS";
  # [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
  # [ -f "$X3PLUS_WS/install/setup.bash" ] && source "$X3PLUS_WS/install/setup.bash"
  # [ -f "$YDLIDAR_WS/install/setup.bash" ] && source "$YDLIDAR_WS/install/setup.bash"
}

# Alternate variant that fully resets env first (Keep commented unless you need it)
# use_robot() {
#   reset_ros_env
#   [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
#   use_ydlidar && use_x3plus
# }

# Saves a Nav2 map to ~/maps (Creates directory if missing)
save_map() {
  mkdir -p "$HOME/maps"

  # Use first argument as map name, or default to timestamp
  local name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
  local out="$HOME/maps/$name"

  # Verify that a map is being published
  if ! ros2 topic echo /map --once >/dev/null 2>&1; then
    echo "No /map topic detected — SLAM may still be initializing."
    return 1
  fi

  echo "Saving map to: $out.{pgm,yaml}"

  ros2 run nav2_map_server map_saver_cli \
    -f "$out" \
    --ros-args -p save_map_timeout:=10.0

  echo "Map saved: $out.pgm and $out.yaml"
}


# LiDAR launcher (Loads overlays then starts TG30 bringup; forwards any extra args)
launch_lidar() {
  use_robot
  ros2 launch x3plus_lidar_bringup bringup_tg30.launch.py "$@"
}

# Camera launcher (Loads overlays then starts Astra Plus bringup, forwards any extra args)
launch_astra_camera() {
  use_robot
  ros2 launch x3plus_vision_bringup astra_plus.launch.py ns:=orbbec rgb_device:=/dev/video2 "$@"
}

# Camera launcher (Loads overlays then starts the front camera bringup, forwards any extra args)
launch_front_camera() {
  use_robot
  ros2 launch x3plus_vision_bringup arm_cam.launch.py "$@"
}

# Displays the battery percentage on the RGB strip at the back of the robot
display_battery() {
  use_robot
  ros2 run x3plus_examples display_battery
}

# Clean build, clears env, log, build and install directories, and builds everything from start. 
clean_build() {
  SKIP_PC=0
  CLEAN_ARGS=()

  # Parse args and remove flag in one loop
  for arg in "$@"; do
    if [ "$arg" = "--pc" ]; then
      SKIP_PC=1
    else
      CLEAN_ARGS+=("$arg")
    fi
  done

  # Safety checks
  if [ "$EUID" -eq 0 ]; then
    echo "Refusing to run as root."
    return 1
  fi
  if [ "$PWD" = "/" ] || [ "$PWD" = "$HOME" ]; then
    echo "Unsafe directory ($PWD). Please change to your colcon workspace root."
    return 1
  fi
  if [ ! -d "./src" ]; then
    echo "This directory does not appear to be a colcon workspace."
    return 1
  fi

  # Remove build artifacts
  for d in build install log; do
    if [ -e "$d" ]; then
      echo "Removing: $d"
      rm -rf -- "$d"
    fi
  done

  # --- Rebuild ---
  echo "Starting colcon build..."

  if [ "$SKIP_PC" -eq 1 ]; then
    echo "Skipping robot-based packages..."
    if ! colcon build \
      --symlink-install \
      --packages-skip orbbec_camera astra_camera ydlidar_ros2_driver x3plus_lidar_bringup x3plus_mapping_bringup x3plus_vision_bringup astra_camera_msgs orbbec_camera_msgs \
      --continue-on-error \
      "${CLEAN_ARGS[@]}"; then
      echo "Build failed."
      return 0
    fi
  else
    if ! colcon build --symlink-install --packages-select ydlidar_sdk ydlidar_ros2_driver \
      --cmake-args -Wno-dev -DCMAKE_C_FLAGS=-w -DCMAKE_CXX_FLAGS=-w \
      --continue-on-error; then
      echo "Build failed."
      return 0
    fi

    if ! colcon build --symlink-install \
      --packages-skip ydlidar_sdk ydlidar_ros2_driver \
      --continue-on-error \
      "${CLEAN_ARGS[@]}"; then
      echo "Build failed."
      return 0
    fi
  fi

  if [ -f install/setup.bash ]; then
    . install/setup.bash
    echo "Rebuild complete and workspace sourced: $PWD"
  else
    echo "install/setup.bash not found."
  fi
}


# Kill all running ROS 2 nodes safely
ros2_restart() {
  FORCE_MODE=0

  # Check for the --force flag
  if [ "$1" == "--force" ]; then
    FORCE_MODE=1
    echo "Force mode enabled: stubborn ROS 2 processes will be terminated with SIGKILL."
  fi

  echo "Stopping all ROS 2 nodes..."

  # Try to find any running ROS 2 processes
  ROS_PIDS=$(ps -ef | grep -E 'ros2|robot_state_publisher|rviz2|rqt' | grep -v grep | awk '{print $2}')

  if [ -z "$ROS_PIDS" ]; then
    echo "No active ROS 2 nodes found."
  else
    echo "Killing ROS 2 processes..."
    for PID in $ROS_PIDS; do
      echo "Stopping PID $PID"
      kill "$PID" 2>/dev/null
    done
    sleep 1

    # If --force flag used, perform hard kill
    if [ $FORCE_MODE -eq 1 ]; then
      echo "Force killing any remaining ROS 2 processes..."
      pkill -9 -f ros2 2>/dev/null
    fi
  fi

  echo "Resetting ROS 2 daemon..."
  ros2 daemon stop >/dev/null 2>&1
  ros2 daemon start >/dev/null 2>&1

  echo "ROS 2 environment cleaned up safely."

  # Re-source environment
  source /opt/ros/humble/setup.bash
  use_robot
}



# Displays an instruction for all of the commands within this file
bashrc_help() {
echo -e "\033[31m--------------------------------------------------------\033[0m
\033[33mShell commands defined in the .bashrc file:\033[0m

\033[32m- ros_ws_info\033[0m             Displays information about the ROS workspace.
\033[32m- use_robot\033[0m               Sources the X3Plus workspace and driver overlays.
\033[32m- launch_lidar\033[0m            Starts the LiDAR bringup.
\033[32m- launch_astra_camera\033[0m     Starts the Astra Plus depth camera bringup.
\033[32m- launch_front_camera\033[0m     Starts the front camera bringup.
\033[32m- save_map [name]\033[0m         Saves a Nav2 map to ~/maps. If no name is provided,
                            saves under the current date and time.
\033[32m- display_battery\033[0m         Displays the battery percentage on the RGB strip.
\033[32m- clean_build [--pc]\033[0m      Performs a clean build by clearing log, build, and
                            install directories, then rebuilding. Use --pc to skip
                            robot related camera and lidar packages.
\033[32m- ros2_restart [--force]\033[0m  Safely stops all running ROS 2 nodes and restarts
                            the ROS 2 daemon. Use '--force' for a hard kill.

\033[31m--------------------------------------------------------\033[0m"


}

# Quality-of-life aliases
alias use_ydlidar='use_ydlidar'
alias use_x3plus='use_x3plus'
alias use_robot='use_robot'
alias ll='ls -alF'
alias lsa='ls -a'
alias l='ls -CF'

# Desktop notification helper (Run '...; alert' to get a toast when it finishes)
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1 | sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# =========================
# Optional: Yahboom workspace (Disabled by default)
# =========================
# YAHBOOM_WS="$HOME/yahboomcar_ros2_ws/yahboomcar_ws"
# if [ -f "$YAHBOOM_WS/install/setup.bash" ]; then
#   source "$YAHBOOM_WS/install/setup.bash"
# fi

# =========================
# Quick info helper (Prints overlay status and paths)
# =========================
ros_ws_info() {
  local reset='\033[0m'
  local green='\033[32m'
  local cyan='\033[36m'
  local yellow='\033[33m'
  local dim='\033[2m'

  printf "${dim}--------------------------------------------------------${reset}\n"
  printf "Robot Name    : %b%s%b\n" "$green" "${ROBOT:-<not set>}" "$reset"
  printf "ROS 2 Distro  : %b%s%b\n" "$green" "${ROS_DISTRO:-<not set>}" "$reset"
  printf "Domain ID     : %b%s%b\n" "$green" "${ROS_DOMAIN_ID:-<not set>}" "$reset"
  printf "YDLIDAR ws    : %b%s%b\n" "$cyan" "${YDLIDAR_WS:-<not set>}" "$reset"
  printf "X3PLUS ws     : %b%s%b\n" "$cyan" "${X3PLUS_WS:-<not set>}" "$reset"

  printf "\n%bAMENT_PREFIX_PATH%b\n" "$yellow" "$reset"
  IFS=':' read -r -a paths <<< "${AMENT_PREFIX_PATH:-}"
  for p in "${paths[@]}"; do
    if [ -n "$p" ]; then
      printf "  %b%s%b\n" "$dim" "$p" "$reset"
    fi
  done

  printf "${dim}--------------------------------------------------------${reset}\n"
}

# ===== Custom login banner (Clears screen and shows IP + env) =====
: "${SHOW_BANNER:=1}"   # Set SHOW_BANNER=0 to disable temporarily

show_banner() {
  # Return early if not interactive or disabled
  [[ $- != *i* ]] && return
  [[ "$SHOW_BANNER" -eq 0 ]] && return
  [ -t 1 ] || return

  # Local colors
  local reset='\033[0m'
  local red='\033[31m'
  local yellow='\033[33m'
  local green='\033[32m'
  local dim='\033[2m'

  printf '\033[2J\033[H'  # Clear screen

  # Terminal width (fallback 80)
  if command -v tput >/dev/null 2>&1; then
    cols=$(tput cols 2>/dev/null || echo 80)
  else
    cols=$(stty size 2>/dev/null | awk '{print $2}')
    cols=${cols:-80}
  fi

  # Centered title
  title="System Information"
  title_len=${#title}
  pad=$(( (cols - title_len - 2) / 2 ))
  (( pad < 0 )) && pad=0
  printf "${red}%*s[%s]${reset}\n" "$pad" '' "$title"

  # Get IPs
  read -r -a ips <<<"$(hostname -I 2>/dev/null)"
  ip1="${ips[0]}"
  ip2="${ips[1]}"

  # Dot line helper
  print_dotline() { printf "%*s\n" "$cols" "" | tr ' ' '.'; }

  # Print IPs
  printf "${yellow}"
  [ -n "$ip1" ] && printf "IP_Address_1: %s\n" "$ip1"
  [ -n "$ip2" ] && printf "IP_Address_2: %s\n" "$ip2"
  printf "${reset}"
  print_dotline

  # Robot info
  printf "Robot Name   : ${green}%s${reset}\n" "${ROBOT:-<not set>}"
  printf "ROS_DOMAIN_ID: ${green}%s${reset}\n" "${ROS_DOMAIN_ID:-<not set>}"
  printf "my_robot_type: ${green}%s${reset} | my_lidar: ${green}%s${reset} | my_camera: ${green}%s${reset}\n" \
         "${ROBOT_TYPE:-<not set>}" \
         "${RPLIDAR_TYPE:-<not set>}" \
         "${CAMERA_TYPE:-<not set>}"

  printf "\nRun ${red}bashrc_help${reset} for a list of available commands.\n"
  print_dotline
}

if [[ -z "$MY_BASHRC_INIT_DONE" ]]; then
  export MY_BASHRC_INIT_DONE=1

  use_robot
  display_battery
fi

# ===== End custom banner =====
  show_banner