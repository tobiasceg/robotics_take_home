# Milestone 1: Get the simulation navigating

Goal: launch the TurtleBot3 House world in Gazebo with Nav2, send a manual
navigation goal, and watch the robot drive there. No VDA5050, no MQTT, no
code you wrote. This proves the foundation before anything is built on it.

## Environment notes (WSL2 specific)

Already configured in `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export ROS_DOMAIN_ID=30
export QT_QPA_PLATFORM=xcb   # WSLg: Qt must use X11, not Wayland
export GDK_BACKEND=x11
```

Two WSL gotchas worth documenting in the final README:

1. **`QT_QPA_PLATFORM=xcb` is mandatory.** Gazebo and RViz are Qt apps. Under
   WSLg they default to Wayland, which produces a taskbar entry whose window
   never presents. Forcing X11 fixes it.
2. **First load of the house world takes about 3 minutes.** Gazebo processes
   the house meshes once. The stock launch file only waits 30 seconds for the
   robot spawn service, so the very first launch fails to spawn the robot.
   Later launches are fast and succeed normally.

## Opening terminals

Each command below runs in its **own** Ubuntu terminal and keeps running.
Open a new one from the Start menu ("Ubuntu 22.04") or Windows Terminal's
dropdown. Stop any command with `Ctrl+C`.

## Stage 1: Launch the world and drive by hand

**Terminal 1** launches the simulator:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

Wait for a Gazebo window showing a furnished house. The robot spawns at
`x = -2.0, y = -0.5`.

**Terminal 2** gives you keyboard control:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Controls (this terminal must have keyboard focus):

| Key | Effect |
|-----|--------|
| `w` | more forward speed |
| `x` | more reverse speed |
| `a` | turn left |
| `d` | turn right |
| `s` | full stop |

Speeds are **incremental**: each press adds a little. Press `s` to stop.
Waffle tops out at 0.26 m/s.

What this teaches: teleop publishes to the ROS 2 topic `/cmd_vel`, and the
simulated robot subscribes to it. Your VDA5050 client will never touch
`/cmd_vel`; Nav2 does that. But it shows the bottom layer.

Useful inspection commands (any terminal):

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic hz /scan
```

## Stage 2: Build the house map with SLAM

Nav2 cannot plan without a floor plan, and only a map of the small test arena
ships with the packages. SLAM means Simultaneous Localization and Mapping: you
drive around, the laser scanner measures distances to walls continuously, and
the software stitches those into a top-down floor plan while tracking where
the robot is inside it.

Keep Terminal 1 (Gazebo) running. **Terminal 3** starts SLAM and opens RViz:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

RViz opens showing the map building live. Now drive with teleop (Terminal 2).

Mapping tips:

- Drive **slowly**. Fast motion smears the map.
- Turn slowly too, especially in doorways.
- Visit every room your delivery route will use.
- Re-driving a corridor you already mapped sharpens it.
- Watch RViz: grey is free space, black is walls, unknown stays blank.

Aim for a clean map of at least a few connected rooms. You need enough
distance between pickup and dropoff that a pause lands mid-drive later.

## Stage 3: Save the map

With Gazebo and cartographer still running, in **Terminal 4**:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/house_map
```

This writes two files:

- `~/maps/house_map.pgm` is the floor plan image
- `~/maps/house_map.yaml` describes its resolution and origin

Both matter later. Nav2 reads the yaml. The pgm is the image you trace over
in the LIF editor at milestone 2, and the yaml's `resolution` and `origin`
are what convert LIF pixel positions into the metre coordinates Nav2 expects.

Now stop everything with `Ctrl+C` in each terminal.

## Stage 4: Navigate to a clicked goal

**Terminal 1**, the simulator again:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Terminal 2**, Nav2 plus RViz, pointed at your map:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/house_map.yaml
```

In RViz:

1. Click **2D Pose Estimate** in the toolbar.
2. Click on the map where the robot actually is, and drag in the direction it
   is facing before releasing.
3. Check that the red laser points line up with the map's walls. If they do
   not, repeat step 2 more accurately.
4. Click **Nav2 Goal** (may be labelled "2D Goal Pose").
5. Click a destination, drag for final heading, release.

The robot should plan a route and drive there, avoiding furniture.

Why step 2 is needed: the robot has a map but no GPS. It locates itself by
comparing laser readings against the map, using a particle filter called
AMCL, which needs a rough starting hint. That estimated pose is exactly what
becomes `agvPosition` in your VDA5050 `state` messages.

## Milestone 1 is done when

The robot drives to a goal you clicked in RViz. Take a screenshot for the
README.

Worth noticing: clicking that goal is precisely what your VDA5050 client will
do automatically at milestone 3. When an order arrives, the client reads the
next node's coordinates and hands them to Nav2 as a `NavigateToPose` goal,
exactly as your mouse click does.
