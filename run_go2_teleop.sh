#!/usr/bin/env bash
#
# run_go2_teleop.sh — bring up the Unitree Go2 sim, keyboard teleop, and the
# go2_eskf error-state EKF, all torn down together by a single Ctrl-C in this
# launching shell.
#
#   Window: Go2 Sim   — Gazebo + CHAMP stack (log view)
#   Window: ESKF      — error-state EKF (log view; shows leg-odom/GPS arrival)
#   Window: Grnd Truth / Plot  — only with --plot
#   Window: TELEOP    — LIVE keyboard driver (give it focus: i/j/k/l/, to drive)
#
# One window PER component (GNOME Terminal 3.52 can't reliably put multiple tabs
# in one window from the CLI — that bug is why only one tab appeared before). The
# sim/ESKF/etc. run as background children of THIS script (so it owns their process
# groups) and their output is shown via `tail`; teleop runs live in its own window.
# Press Ctrl-C in THIS shell to stop everything. (Want a true single window with
# tabs/panes? Install tmux and ask — a tmux layout can replace the windows.)
#
# Usage:
#   ./run_go2_teleop.sh                    # LIGHT default: RViz OFF, GPS on, NVIDIA
#   ./run_go2_teleop.sh --rviz             # add the (heavy) RViz window
#   ./run_go2_teleop.sh --plot             # + ground-truth bridge + live XY/error plot
#   ./run_go2_teleop.sh --lite             # lowest load (no RViz, slower plot)
#   ./run_go2_teleop.sh --software-render   # CPU (llvmpipe) rendering fallback
#   ./run_go2_teleop.sh --obstacles        # bring the boxes/cylinders back
#
# World: defaults to the obstacle-free go2_eskf/worlds/flat.sdf (same GPS datum,
# physics, lighting and ground plane as the vendored default.sdf, minus the five
# obstacle models — box1 sat at (5,0), right on the 10 m square path). Nothing was
# deleted: --obstacles selects the vendored default.sdf instead.
#
# Load: RViz is OFF by default (biggest easy saving on the RTX 3050). The plot is
# throttled and, like the ground-truth bridge, niced + started only after the sim
# is up, so nothing piles onto Gazebo's boot. The remaining GPU cost is Gazebo's
# camera/LiDAR sensor rendering — trimming that needs a (vendored) model edit; ask.
#
# Notes:
#   * Shut down with Ctrl-C HERE (in the launching shell), not in the tabs.
#   * The sim's OFFSCREEN sensor renderer is forced onto the NVIDIA GPU; without
#     that, Gazebo segfaults on this Optimus laptop. If it still crashes, re-run
#     with --software-render.
#   * The ESKF waits for /odom/raw (leg odom) and /gps/fix before starting, so it
#     never dead-reckons before its corrections exist. Watch the ESKF tab for the
#     "First /odom/raw ... received" and "First /gps/fix ... received" lines — if
#     either is missing, that input isn't reaching the filter (the usual cause of
#     a runaway position estimate).

set -uo pipefail
# Job control ON so every background component lands in its OWN process group
# (PGID == its PID). That lets cleanup() signal a whole ros2-launch subtree with
# one `kill -- -PGID`, cleanly tearing down gz, bridges, controllers, etc.
set -m

# --- config ---------------------------------------------------------------
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
RVIZ="false"                                 # RViz is heavy on the RTX 3050 — opt in
PLOT="false"
SQUARE="false"                               # --square: autonomous drift test, no teleop
OBSTACLES="false"                            # --obstacles: use the vendored world WITH boxes/cylinders
RENDER="nvidia"                              # nvidia | software
PLOT_INTERVAL="0.1"                          # plot redraw period [s] (10 Hz)
PLOT_VIEW="10"                               # plot XY half-width [m] (13 for --square)

for arg in "$@"; do
  case "$arg" in
    --rviz)             RVIZ="true" ;;
    --no-rviz)          RVIZ="false" ;;   # default; kept for compatibility
    --plot)             PLOT="true" ;;
    --square)           SQUARE="true"; PLOT="true"; PLOT_VIEW="13" ;;  # autonomous square drift test
    --obstacles)        OBSTACLES="true" ;;   # bring the boxes/cylinders back
    --software-render)  RENDER="software" ;;
    --light|--lite)     RVIZ="false"; PLOT_INTERVAL="0.2" ;;  # lowest load
    -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done

# `nice`/`ionice` prefix for non-realtime helpers (plot, bridges) so the sim and
# controllers win CPU during the busy boot — that's what was starving the
# controller_manager and causing "Failed to acquire lock" timeouts.
LOW_PRIO="nice -n 15 ionice -c3"
command -v ionice >/dev/null 2>&1 || LOW_PRIO="nice -n 15"

# --- sanity checks --------------------------------------------------------
[[ -f "$WS/install/setup.bash" ]] || {
  echo "ERROR: workspace not built. Run 'colcon build' in $WS first." >&2; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || {
  echo "ERROR: gnome-terminal not found." >&2; exit 1; }

# --- world selection --------------------------------------------------------
# Default is the FLAT world (first-party, go2_eskf/worlds/flat.sdf): same GPS datum,
# physics, lighting and ground plane as the vendored default.sdf, minus the five
# obstacle models. box1 sits at (5,0), right on the 10 m square path. The obstacles
# aren't deleted — they're still in the vendored world; --obstacles selects it.
FLAT_WORLD="$WS/install/share/go2_eskf/worlds/flat.sdf"
OBSTACLE_WORLD="$WS/install/share/unitree_go2_description/worlds/default.sdf"
if [[ "$OBSTACLES" == "true" ]]; then
  WORLD="$OBSTACLE_WORLD";  WORLD_DESC="default.sdf (obstacles ON)"
else
  WORLD="$FLAT_WORLD";      WORLD_DESC="flat.sdf (obstacles OFF)"
fi
[[ -f "$WORLD" ]] || {
  echo "ERROR: world not found: $WORLD" >&2
  echo "       Build it first: colcon build --packages-select go2_eskf --merge-install --symlink-install" >&2
  exit 1; }

# --- GPU / offscreen-rendering env for Gazebo -----------------------------
# Gazebo's sensor-rendering thread (Ogre2) makes an OFFSCREEN GL context separate
# from the GUI. On an NVIDIA Optimus laptop it wrongly picks the Mesa/DRI2 path
# for the NVIDIA card ("failed to create dri2 screen") and segfaults, killing the
# gz server (and controller_manager, /odom/raw, GPS...). Force NVIDIA's own EGL/GLX;
# --software-render falls back to CPU rasterisation (llvmpipe).
NV_EGL_ICD="/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
if [[ "$RENDER" == "software" ]]; then
  RENDER_ENV="export LIBGL_ALWAYS_SOFTWARE=1; export __GLX_VENDOR_LIBRARY_NAME=mesa;"
else
  RENDER_ENV="export __NV_PRIME_RENDER_OFFLOAD=1; \
              export __GLX_VENDOR_LIBRARY_NAME=nvidia; \
              export __VK_LAYER_NV_optimus=NVIDIA_only;"
  [[ -f "$NV_EGL_ICD" ]] && \
    RENDER_ENV="$RENDER_ENV export __EGL_VENDOR_LIBRARY_FILENAMES='$NV_EGL_ICD';"
fi

SOURCE_ENV="source '$ROS_SETUP' && source '$WS/install/setup.bash'"

# --- readiness guard -------------------------------------------------------
# The sim is only USABLE once ros2_control has ACTIVATED the leg controller.
#
# This used to wait for /odom/raw, which is useless: CHAMP's state_estimation_node
# starts publishing /odom/raw (all zeros) ~2 s after launch, long before
# controller_manager exists. So the guard fired instantly and the ESKF, ground-truth
# bridge, plot and square test all piled onto Gazebo's boot — starving the
# controller_manager that lives INSIDE the gz process. The spawners then failed with
# "Failed to acquire lock in 20 seconds" / "waiting for service
# /controller_manager/list_controllers", no controller held the legs, and the Go2
# collapsed onto its belly (base z 0.375 -> 0.057) and never walked. The estimate
# looked "frozen at the origin" — but the robot really was frozen.
#
# joint_group_effort_controller reporting `active` is the honest signal.
WAIT_READY="echo 'Waiting for ros2_control to activate the leg controller (~30 s)...'; \
  until timeout 5 ros2 control list_controllers 2>/dev/null \
        | grep -q 'joint_group_effort_controller.*active'; do sleep 2; done; \
  echo 'Leg controller ACTIVE — the Go2 is standing.'"
LOGDIR="$(mktemp -d /tmp/go2_teleop.XXXXXX)"
declare -a PIDS=()   # process-group leaders of everything we start in the bg

# Start a command in the background (its own process group, via `set -m`),
# logging to $LOGDIR/<name>.log. Sets REPLY to the PID (== PGID) and records it.
start_bg() {  # $1 = name, $2 = command
  bash -c "$SOURCE_ENV; $2" >"$LOGDIR/$1.log" 2>&1 &
  REPLY=$!
  PIDS+=("$REPLY")
}

# --- teardown -------------------------------------------------------------
cleanup() {
  trap '' INT TERM EXIT   # no re-entry
  set +e
  echo; echo "Shutting down the Go2 stack..."
  # SIGINT each process group first so ros2 launch stops its nodes cleanly.
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done
  pkill -INT -f teleop_twist_keyboard 2>/dev/null   # lives in the terminal server
  sleep 3
  # Hard-kill any stragglers; this also ends the `tail` tabs (they follow --pid),
  # which closes the window.
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
  pkill -KILL -f teleop_twist_keyboard 2>/dev/null
  rm -rf "$LOGDIR"
  echo "Done."
  exit 0
}
trap cleanup INT TERM

echo "Workspace : $WS"
echo "RViz      : $RVIZ    Plot: $PLOT (${PLOT_INTERVAL}s)    Rendering: $RENDER"
echo "World     : $WORLD_DESC"
echo "Logs      : $LOGDIR"
echo

# --- 1. Gazebo + CHAMP sim ------------------------------------------------
# Sim + controllers get FULL CPU priority; everything else is staggered behind
# it and niced, so they don't pile onto Gazebo's boot (the load spike you saw).
start_bg sim \
  "$RENDER_ENV ros2 launch unitree_go2_sim unitree_go2_launch.py use_sim_time:=true rviz:=$RVIZ world:='$WORLD'"
SIM_PID=$REPLY

# --- 2. Ground-truth bridge (optional) — wait for the sim clock first ------
if [[ "$PLOT" == "true" ]]; then
  start_bg ground_truth \
    "$WAIT_READY; \
     exec $LOW_PRIO ros2 launch go2_eskf ground_truth.launch.py use_sim_time:=true"
  GT_PID=$REPLY
fi

# --- 3. Error-state EKF (waits for its inputs before starting) ------------
start_bg eskf \
  "$WAIT_READY; \
   echo 'Starting ESKF (GPS OFF — the sim navsat is broken:'; \
   echo '  ~0.5 deg / ~55 km position noise, which would wreck the estimate).'; \
   ros2 launch go2_eskf eskf.launch.py use_sim_time:=true use_gps:=false"
ESKF_PID=$REPLY

# --- 4. Live trajectory plot (optional) — start only once the ESKF publishes,
#        niced and throttled so it never competes with the sim. --------------
if [[ "$PLOT" == "true" ]]; then
  start_bg plot \
    "until timeout 5 ros2 topic echo /eskf/odom --once >/dev/null 2>&1; do :; done; \
     exec $LOW_PRIO ros2 run go2_eskf plot_trajectory.py --ros-args \
       -p use_sim_time:=true -- --interval $PLOT_INTERVAL --view $PLOT_VIEW"
  PLOT_PID=$REPLY
fi

# --- 5. Autonomous square drift test (optional, replaces teleop) -----------
# Waits for the ESKF to publish AND a short gait warm-up, then drives a 10 m
# square closed-loop on ground truth. All drift you see in the plot/summary is
# estimator error, not driving error.
if [[ "$SQUARE" == "true" ]]; then
  start_bg square \
    "until timeout 5 ros2 topic echo /eskf/odom --once >/dev/null 2>&1; do :; done; \
     echo 'ESKF is up — 10 s gait warm-up before driving the square...'; sleep 10; \
     ros2 run go2_eskf square_test.py --ros-args -p use_sim_time:=true"
  SQUARE_PID=$REPLY
fi

# --- Open a viewer window per component -----------------------------------
# NOTE: GNOME Terminal 3.52 cannot reliably open multiple tabs in ONE window from
# a single command line (a known CLI regression — it silently collapses to one
# tab, which is why teleop never appeared before). So each component gets its own
# window instead. `tail --pid=X` exits when node X dies, so cleanup's kills also
# close these viewer windows. Teleop runs LIVE in its own window (needs a TTY).
show_log() {  # $1 = window title, $2 = log name, $3 = pid to follow
  gnome-terminal --title "$1" -- \
    bash -c "trap '' INT; echo '── $1 (log) ──'; tail --pid=$3 -n +1 -f '$LOGDIR/$2.log'" \
    2>/dev/null &
}

show_log "Go2 Sim" sim "$SIM_PID"
[[ "$PLOT" == "true" ]] && show_log "Go2 Ground Truth" ground_truth "$GT_PID"
show_log "Go2 ESKF (watch for leg-odom/GPS arrival)" eskf "$ESKF_PID"
[[ "$PLOT" == "true" ]] && show_log "Go2 Plot" plot "$PLOT_PID"

if [[ "$SQUARE" == "true" ]]; then
  # Autonomous square drives /cmd_vel — no teleop (they would fight).
  show_log "SQUARE DRIFT TEST (watch corners + summary)" square "$SQUARE_PID"
else
  # Teleop: live, interactive, its own window.
  gnome-terminal --title "TELEOP ← DRIVE HERE" -- \
    bash -c "$SOURCE_ENV; echo 'DRIVE THE ROBOT HERE — keys: i/j/k/l and , (comma). Keep this window focused.'; echo; \
             ros2 run teleop_twist_keyboard teleop_twist_keyboard" 2>/dev/null &
fi

echo "======================================================================"
if [[ "$SQUARE" == "true" ]]; then
  echo " Go2 stack launched — AUTONOMOUS 10 m SQUARE drift test (no teleop)."
  echo "   Watch the SQUARE window for per-corner errors and the final summary."
else
  echo " Go2 stack launched (one window per component + a TELEOP window)."
  echo "   Drive from the TELEOP window (give it focus): i / j / k / l / , "
fi
echo
echo " >>> Press Ctrl-C IN THIS SHELL to shut EVERYTHING down. <<<"
echo "======================================================================"

# Block until Ctrl-C (trap -> cleanup). If a bg component exits on its own,
# keep waiting for the rest; the user tears down explicitly.
while true; do sleep 3600 & wait $!; done
