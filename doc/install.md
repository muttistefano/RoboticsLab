# Installation

*How to set up the workspace — Ubuntu prerequisites, the build, and the Windows devcontainer.
Back to the [index](../README.md).*

## 1. Prerequisites

- Ubuntu 24.04 with **ROS 2 Kilted**
- Gazebo Sim 9 (installed as `ros-kilted-ros-gz-sim`, not as a standalone `gz` package)

Install the dependencies this workspace needs:

```bash
sudo apt update
sudo apt install -y \
  ros-kilted-ros-gz-sim ros-kilted-ros-gz-bridge ros-kilted-ros-gz-image \
  ros-kilted-gz-ros2-control ros-kilted-controller-manager \
  ros-kilted-joint-state-broadcaster ros-kilted-position-controllers \
  ros-kilted-joint-trajectory-controller ros-kilted-velocity-controllers \
  ros-kilted-pid-controller ros-kilted-effort-controllers \
  ros-kilted-moveit-servo ros-kilted-moveit-kinematics \
  ros-kilted-pinocchio python3-scipy \
  ros-kilted-ur-description ros-kilted-ur-simulation-gz ros-kilted-ur-controllers \
  ros-kilted-robotiq-description \
  ros-kilted-navigation2 ros-kilted-nav2-bringup ros-kilted-slam-toolbox \
  ros-kilted-robot-localization ros-kilted-twist-mux \
  ros-kilted-joy ros-kilted-teleop-twist-joy \
  ros-kilted-rviz2 ros-kilted-topic-tools ros-kilted-tf2-tools \
  ros-kilted-rqt-common-plugins ros-kilted-plotjuggler-ros
```

Or let `rosdep` work it out from `package.xml`:

```bash
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **`gz` is not on your PATH until you source ROS.** The binary ships inside the ROS vendor tree
> (`/opt/ros/kilted/opt/gz_tools_vendor/bin/gz`), and it needs `GZ_CONFIG_PATH` set. If `gz sim
> --versions` says *"I cannot find any available 'gz' command"*, you have not sourced
> `/opt/ros/kilted/setup.bash`.

> **If you have more than one ROS distro installed** (e.g. Jazzy *and* Kilted), be careful which one
> you source. Jazzy ships gz-sim **8**; this workspace targets gz-sim **9**.

## 2. Build

```bash
source /opt/ros/kilted/setup.bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` symlinks Python, launch and config files into `install/`, so edits to them take
effect without rebuilding. Changes to `package.xml`, `CMakeLists.txt` or the URDF still need a
rebuild.

**Every new terminal needs both `source` lines.** The workflow in [running.md](running.md) uses
three terminals.

## 3. Windows — devcontainer with VcXsrv

Linux and macOS users can open the devcontainer directly; `DISPLAY` and the X11 socket are passed
through automatically.

1. Install [VSCode](https://code.visualstudio.com/),
   [Docker Desktop](https://www.docker.com/products/docker-desktop),
   the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers),
   and [VcXsrv](https://sourceforge.net/projects/vcxsrv/).
2. **Configure VcXsrv:** launch XLaunch → "Multiple windows", display number `0` → "Start no
   client" → tick **Disable access control**. Save the config.
3. **Firewall:** allow VcXsrv through Windows Defender on both private and public networks.
4. **Point the container at your X server:** in `.devcontainer/devcontainer.json`, set
   `"DISPLAY": "host.docker.internal:0"`.
5. `F1` → *Dev Containers: Reopen in Container*, and wait for the build.
6. Build and run as in [running.md](running.md). Gazebo and RViz appear on your Windows desktop.

### Troubleshooting

- **The robot never spawns (`spawn_robot` repeats "Requesting list of world names"), or nodes
  silently cannot see each other's topics** (any OS): a VPN is up (WireGuard, NordVPN,
  Tailscale, ...) and it is swallowing the discovery multicast of *both* middlewares, with no
  error printed anywhere. Everything here runs on one machine, so pin both to the local host —
  in every terminal (or your `~/.bashrc`):

  ```bash
  export GZ_IP=127.0.0.1
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ```

  or disconnect the VPN. The test suite sets both automatically.
- **No window appears** — VcXsrv running, with *Disable access control* ticked?
- **Connection refused** — check the firewall rule for VcXsrv.
- **Wrong display** — the number in `DISPLAY` must match VcXsrv's (normally `:0`).
- **Gazebo is very slow** — you are almost certainly rendering on the wrong GPU. On an NVIDIA
  Optimus laptop the X server drives the Intel iGPU, so Gazebo's sensors render there or fall back
  to software. Check with:

  ```bash
  glxinfo | grep "OpenGL renderer"        # names the GPU actually in use
  nvidia-smi                              # a near-idle discrete GPU is the tell
  ```

  `lampo_sandbox.launch.py` sets the PRIME offload variables
  (`__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`) for exactly this reason; they
  are ignored on machines without an NVIDIA driver. This matters beyond frame rate: starved sensor
  rendering steals the CPU that Nav2's control loop needs, and goals start failing with *Failed to
  make progress*. On Windows, expect llvmpipe and use `gui:=false` where you can.
