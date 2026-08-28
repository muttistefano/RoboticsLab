# Control exercises

*Learn feedback control on a robot that behaves like a robot: tune real PID gains, live, and watch
the response change. Back to the [index](../README.md).*

The arm normally runs a *forward position controller*: you publish a joint position, the simulated
motor goes there, end of story. That hides everything a control course is about. These exercises
replace it with `ros2_control`'s chainable **`pid_controller`** so the loop is yours: you choose
the gains, the simulator provides the plant, and every term of the controller is published for
plotting.

Four arm modes, selected at launch:

| mode | what drives the joints | what it teaches |
|---|---|---|
| `arm_controller:=pid` | a PID outputting joint **velocity** | step response, overshoot, settling time, tuning method |
| `arm_controller:=effort_pid` | a PID outputting joint **torque** | gravity, steady-state error, the I term, integral windup |
| `arm_controller:=servo` | **MoveIt Servo**: Cartesian twist → joint velocity | kinematics as control — Jacobians, singularities, limits |
| `arm_controller:=effort` | **your node**, publishing raw torque | dynamics: gravity compensation, observers, LQR |

The PIDs read the joint *position* as feedback and re-read their gains **every update cycle**, so
`ros2 param set` retunes the running controller instantly — no restart, no rebuild. Servo's
speed scales and singularity thresholds are live in exactly the same way.

The curriculum, in order:

1. [The tuning lab](#exercise-1--the-tuning-lab-arm_controllerpid) — PID on an integrator plant
2. [Disturbance & windup](#exercise-2--disturbance--windup-arm_controllereffort_pid) — the I term earns its keep
3. [Cartesian control](#exercise-3--cartesian-drive-the-tool-not-the-joints-arm_controllerservo) — Jacobians and singularities
4. [Zero-G](#exercise-4--zero-g-gravity-compensation-arm_controllereffort) — gravity compensation from the model
5. [The observer](#exercise-5--the-luenberger-observer) — estimating what you cannot measure
6. [LQR](#exercise-6--lqr-control-you-derive-instead-of-tune) — control you derive instead of tune
7. [Kalman tuning](#exercise-7--tune-a-kalman-filter-against-ground-truth) — estimation against ground truth
8. [MPPI](#exercise-8--optimal-control-you-already-own-mppi) — the optimal controller already driving your base

## Setup

```bash
# Terminal 1 — the world (see running.md)
ros2 launch lampo_description lampo_sandbox.launch.py

# Terminal 2 — the mobile manipulator with the PID lab active
ros2 launch lampo_description lampo_gz_mm.launch.py mm:=true arm_controller:=pid
```

Check what is running:

```bash
ros2 control list_controllers -c /r1_/controller_manager
# arm_pid_controller             active
# forward_position_controller    inactive     <- spawned as the fallback
# joint_state_broadcaster        active
```

The forward controller is loaded but inactive, so you can flip between the two paths live:

```bash
ros2 control switch_controllers -c /r1_/controller_manager \
    --deactivate arm_pid_controller --activate forward_position_controller
```

(Never activate both: they would claim conflicting command interfaces on the same joints.)

## Exercise 1 — the tuning lab (`arm_controller:=pid`)

Send the controller a step reference. All six joints go in one message; the shoulder pan gets the
step, the rest hold the home pose:

```bash
ros2 topic pub --once /r1_/arm_pid_controller/reference control_msgs/msg/MultiDOFCommand \
  "{dof_names: [r1_shoulder_pan_joint, r1_shoulder_lift_joint, r1_elbow_joint,
                r1_wrist_1_joint, r1_wrist_2_joint, r1_wrist_3_joint],
    values: [1.0, -1.57, 0.0, -1.57, 0.0, 0.0]}"
```

Watch the loop work — plot these in PlotJuggler or `rqt_plot`:

- `/r1_/arm_pid_controller/controller_state` — reference, feedback, error and output per joint
  (`control_msgs/msg/MultiDOFStateStamped`)
- `/r1_/arm_pid_controller/r1_shoulder_pan_joint/pid_state` — the P, I and D contributions
  separately (`control_msgs/msg/PidState`)

Now tune, live:

```bash
ros2 param set /r1_/arm_pid_controller gains.r1_shoulder_pan_joint.p 8.0
```

Repeat the step after each change and measure **rise time**, **overshoot** and **settling time**
from the plot. Things to establish for yourself:

1. **P alone already converges with zero steady-state error.** Why? The plant here — velocity
   command in, position out — is a pure integrator. One integrator in the loop kills constant
   error for free. Remember this; exercise 2 takes it away.
2. **More P is faster, until it isn't.** Raise `p` until the response overshoots and rings. The
   output clamp (`u_clamp_max`, ±1.5 rad/s on the big joints) is doing part of the shaping —
   watch the output saturate in `pid_state`.
3. **D buys damping.** With `p` high enough to ring, add small `d` and watch the ringing die.
4. **A tuning recipe, not just knob-twiddling.** Classic Ziegler–Nichols: set `i` and `d` to
   zero, raise `p` until the joint oscillates steadily; call that gain *Ku* and the oscillation
   period *Tu*, then try `p = 0.6·Ku`, `i = 1.2·Ku/Tu`, `d = 0.075·Ku·Tu`. Compare it with your
   hand tuning.

### Tune it with sliders

Everything above works from the terminal; the graphical version is two commands:

```bash
ros2 run rqt_reconfigure rqt_reconfigure /r1_/arm_pid_controller
ros2 run rqt_plot rqt_plot \
    /r1_/arm_pid_controller/controller_state/dof_states[0]/reference \
    /r1_/arm_pid_controller/controller_state/dof_states[0]/feedback
```

**rqt_reconfigure** opens with the controller pre-selected (the argument is optional — without
it, pick the node from the list): every `gains.r1_<joint>.*` parameter appears as a live
control. **rqt_plot** draws the shoulder-pan reference against its feedback; PlotJuggler does
the same with more polish if you prefer it. Drag a gain, republish the step, watch the response
reshape — the whole tuning loop with no typing. The same two windows work unchanged for
`arm_effort_pid_controller` and for Servo below (`/r1_/servo_node`).

## Exercise 2 — disturbance & windup (`arm_controller:=effort_pid`)

```bash
ros2 launch lampo_description lampo_gz_mm.launch.py mm:=true arm_controller:=effort_pid
```

Same loop, one change: the PID now outputs **torque**, and the physics engine — inertia, gravity,
friction — is the plant. The shipped gains are deliberately rough starting values; tuning them is
the exercise.

1. **Gravity creates steady-state error.** Command a shoulder lift reference and watch the arm
   stop *short* of it: with P only, the joint settles exactly where the P term balances the
   gravity torque. The error is right there in `controller_state`, and it never goes away.
2. **The I term removes it.** Give the shoulder a small `i` and watch the error integrate away.
   Look at `pid_state`: the I contribution converges to the gravity torque — the integrator has
   *learned* the disturbance.
3. **Windup.** Command a large step. The output saturates at the joint's real torque limit
   (`u_clamp`, ±738 Nm on the shoulders) while the integrator keeps accumulating; when the joint
   finally arrives, the bloated I term drives it far past the target before unwinding. Then set
   `antiwindup_strategy` to `back_calculation` and repeat — same step, no blow-through. That
   single comparison is most of what anyone needs to know about windup.
4. **Disturbance rejection, physically.** In the Gazebo GUI, open the *Apply Force Torque*
   plugin, grab a wrist link and shove it. Watch the loop fight back in the plots — then detune
   `p` and shove again.

## Exercise 3 — Cartesian: drive the tool, not the joints (`arm_controller:=servo`)

```bash
ros2 launch lampo_description lampo_gz_mm.launch.py mm:=true arm_controller:=servo
```

Everything so far controlled *joints*. Real manipulation tasks are specified in *Cartesian*
space — "move the tool 10 cm down" — and something has to translate between the two: the
Jacobian. This mode runs **MoveIt Servo** in front of the velocity controller: it takes a stream
of Cartesian twists, solves for the joint velocities that realise them, and decelerates on its
own near singularities, joint limits and collisions.

**You spawn at a singularity. Two of them.** The arm comes up with its elbow dead straight —
a boundary singularity, where no joint motion can produce velocity along the arm's axis — AND
with `wrist_2` at zero, where the axes of joints 4 and 6 are parallel and a rotation degree of
freedom collapses (the classic *wrist-alignment* singularity). Try a twist command from here
and Servo refuses with *"Very close to a singularity, emergency stop"* — and it keeps
refusing if you fix only the elbow, which is worth discovering for yourself. The way out is
joint-space jogging, which never goes through the Jacobian inverse. Unfold both:

```bash
ros2 service call /r1_/servo_node/switch_command_type moveit_msgs/srv/ServoCommandType \
    "{command_type: 0}"    # 0 = JOINT_JOG
ros2 topic pub -r 20 /r1_/servo_node/delta_joint_cmds control_msgs/msg/JointJog \
    "{joint_names: [r1_elbow_joint, r1_wrist_2_joint], velocities: [0.4, 0.3]}"
# let it unfold for a couple of seconds, then Ctrl-C
```

Now switch to Cartesian twists:

```bash
ros2 service call /r1_/servo_node/switch_command_type moveit_msgs/srv/ServoCommandType \
    "{command_type: 1}"    # 1 = TWIST
```

and drive the tool tip — here, straight down at 5 cm/s for as long as the messages keep coming:

```bash
ros2 topic pub -r 20 /r1_/servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: r1_tool0}, twist: {linear: {z: -0.05}}}"
```

Stop publishing and the arm stops — commands expire after a quarter of a second. For interactive
jogging use the bundled keyboard teleop (key bindings printed at start):

```bash
ros2 run moveit_servo servo_keyboard_input --ros-args -r __ns:=/r1_
```

Things to establish for yourself:

1. **Straight lines are expensive.** Watch `joint_states` while the tool moves in a straight
   line: six joints doing carefully coordinated, very non-constant velocities. That coordination
   is the Jacobian inverse, recomputed every 10 ms.
2. **Singularities are real places.** You met one at spawn; now jog the arm back toward full
   stretch and watch it slow down and stop *before* the geometry degenerates, no matter how
   hard you command. Open rqt_reconfigure on `/r1_/servo_node` and lower
   `moveit_servo.lower_singularity_threshold` to feel where the guard band starts; the
   condition number of the Jacobian is doing the deciding.
3. **Speed is a parameter, not a habit.** `moveit_servo.scale.linear` caps the commanded m/s —
   drag it live while holding a jog key.
4. **It refuses to hit the robot.** Jog the wrist toward the chassis: Servo decelerates and
   halts on the collision check before contact. Which link pairs it watches — and which it
   deliberately ignores, like the arm's own mount — is written in `urdf/lampo_ur.srdf.xacro`.

## Exercise 4 — Zero-G: gravity compensation (`arm_controller:=effort`)

```bash
ros2 launch lampo_description lampo_gz_mm.launch.py mm:=true arm_controller:=effort
```

This mode is a raw torque passthrough with **nobody driving it** — watch the arm sag under its
own weight the moment it spawns. Every controller so far has been quietly fighting gravity
without ever knowing it exists; now that job is yours:

```bash
ros2 run lampo_description zero_g.py --ros-args -r __ns:=/r1_ -p use_sim_time:=true
```

The node parses the same URDF the simulator runs (pinocchio, from the `robot_description`
topic), evaluates the generalized gravity vector g(q) a hundred times a second, and commands
exactly that torque. The arm now *floats*: open the Gazebo **Apply Force/Torque** dialog, grab
a wrist link, and push a quarter-tonne-rated industrial arm around with 20 N. This is the
"freedrive" button on a real cobot, and it is one model evaluation per cycle.

Gravity compensation holds the arm *where it is* — it cannot un-fall it. If the arm has
collapsed, do not try to switch back to the position controller to lift it: activating a
position controller onto a fallen, moving arm trips `ros2_control`'s joint limiter, which
treats it as a hardware fault and **aborts the whole simulator** (the same failure mode the
gripper note in `urdf/mm_gripper.xacro` describes). The reliable reset is the cheap one —
Ctrl-C the robot's launch (terminal 2) and run it again; the world keeps running and the arm
respawns at home. Starting your torque node within a few seconds of the spawn keeps the fall
to a barely visible sag.

Things to establish for yourself:

1. **g(q) depends on q.** Push the arm horizontal and read the commanded efforts
   (`ros2 topic echo /r1_/forward_effort_controller/commands`): the shoulder torque grows
   toward hundreds of Nm as the lever arm grows, and collapses to almost nothing when the arm
   points straight up — which is exactly why it spawns that way.
2. **Model-based control is only as good as the model.** Edit a mass in the arm's xacro, do NOT
   rebuild, and the arm drifts: simulator and compensator now disagree. (Undo it.)
3. **This is the feedforward half of every serious manipulator controller.** Exercise 2's
   I-term *learned* the gravity torque by integrating error; this node just *computes* it, with
   zero error to integrate.

## Exercise 5 — the Luenberger observer

The simulator hands you joint velocity for free. A real encoder does not — you get position,
and velocity must be *estimated*. With Zero-G still running:

```bash
ros2 run lampo_description joint_observer.py --ros-args -r __ns:=/r1_ -p use_sim_time:=true
```

The observer knows the shoulder-lift's model (the same g(q) and inertia as Zero-G), listens to
the commanded torque and the measured *position only*, and integrates
x̂' = A x̂ + B u + **L**(y − ŷ). Plot all four published signals:

```bash
ros2 run rqt_plot rqt_plot /r1_/joint_observer/estimate/data[2] \
                           /r1_/joint_observer/estimate/data[3]
```

`data[2]` is the simulator's true velocity — the answer sheet the observer never reads —
and `data[3]` is the estimate. Shove the arm around in Zero-G and watch the estimate chase the
truth. Then move the poles, live:

```bash
ros2 param set /r1_/joint_observer poles "[-30.0, -40.0]"   # trust the measurement
ros2 param set /r1_/joint_observer poles "[-2.0, -3.0]"     # trust the model
```

Fast poles track sharply and amplify every measurement wiggle; slow poles are smooth and late.
That tradeoff has a name in the next exercise: it is what a Kalman filter optimizes.

## Exercise 6 — LQR: control you derive instead of tune

Exercises 1 and 2 tuned gains by feel. Optimal control flips the workflow: you state what you
care about — position error, velocity, actuator effort — as a cost

    J = ∫ q_pos·e² + q_vel·ė² + r·u² dt

and the Riccati equation *derives* the unique gain K that minimizes it. Stop Zero-G
(one torque source at a time), then:

```bash
ros2 run lampo_description lqr_joint.py --ros-args -r __ns:=/r1_ -p use_sim_time:=true
ros2 topic pub --once /r1_/lqr_joint/reference std_msgs/msg/Float64 "{data: -1.2}"
```

The node gravity-compensates all six joints, holds five of them softly, and runs LQR on the
shoulder-lift. K is re-solved and logged every time you touch a weight:

```bash
ros2 param set /r1_/lqr_joint r 0.001      # effort is cheap  -> stiff and fast
ros2 param set /r1_/lqr_joint r 1.0        # effort is costly -> gentle and slow
```

Things to establish for yourself:

1. **Only ratios matter.** Multiply `q_pos`, `q_vel` and `r` by the same factor — K does not
   move. The cost has no units; the tradeoff does.
2. **Compare with exercise 1.** Step the same joint under your best hand-tuned PID and under
   LQR with honest weights. Same plant, two philosophies.
3. **The observer closes the story.** LQR reads the simulator's velocity; on a real robot it
   would read exercise 5's estimate. LQR + Luenberger observer = LQG, the classical stack.

## Exercise 7 — tune a Kalman filter against ground truth

The base has run an EKF option all along (`robot_localization`, [running.md](running.md)) —
but the simulated sensors are so clean the filter had nothing to do. Give it something to do:

```bash
ros2 launch lampo_description lampo_slam.launch.py use_ekf:=true noisy:=true
ros2 launch lampo_description lampo_joy.launch.py     # drive around
```

`noisy:=true` inserts `sensor_noise.py` between the simulator and the filter: wheel odometry
velocities and IMU rates arrive corrupted by gaussian noise **with covariances inflated to
match** — a sensor's covariance is its honesty, and the EKF weighs measurements by it. Plot the
three x-velocities:

```bash
ros2 run rqt_plot rqt_plot /r1_/odom/twist/twist/linear/x \
    /r1_/odom_noisy/twist/twist/linear/x \
    /r1_/odometry/filtered/twist/twist/linear/x
```

Ground truth, what the filter sees, what the filter believes. Now tune:

1. **Crank the noise** (live): `ros2 param set /r1_/sensor_noise odom_vel_stddev 0.3` — but the
   injected covariance grows with it, so the filter leans harder on the IMU. Honest sensors
   degrade gracefully.
2. **Lie to the filter.** Edit `process_noise_covariance` in `config/ekf.yaml` and relaunch
   (robot_localization is not live-tunable — worth knowing in itself). Tiny process noise says
   "trust the model": smooth, confident, and slow to admit the robot actually turned. Huge
   process noise says "trust the sensors": you just bought the noise back.
3. **The observer connection.** This is exercise 5's L, chosen optimally at every step from the
   two covariances instead of by pole placement.

## Exercise 8 — optimal control you already own (MPPI)

Nav2's controller server — the thing that has been driving the base whenever you click a goal —
is **MPPI**: sampling-based model-predictive control. Every 50 ms it rolls out ~2000 candidate
velocity sequences through a model of the base, scores each against a cost function, and
executes the best blend. The cost function is right there in `config/nav2_params_omni.yaml`:
the `critics` list (GoalCritic, PathAlignCritic, CostCritic, ...), each with a `cost_weight`.

Run navigation as in [running.md](running.md), send goals, and experiment with the weights —
most MPPI parameters are dynamic, so try `ros2 param set` first and fall back to
edit-and-relaunch if the controller refuses:

- Drop `PathAlignCritic.cost_weight` toward zero: the robot stops caring about *how* it reaches
  the path and beelines. Raise it: it hugs the planner's line pedantically.
- Raise `CostCritic.cost_weight`: wider, more paranoid berths around obstacles.
- `PreferForwardCritic` is why the omni base still mostly drives nose-first: an omni robot
  *can* strafe, and this critic is the only reason it doesn't always.

There is no code to write. The point of the exercise is recognition: the "AI-looking" behavior
of the navigation stack is a cost function you can read, reason about, and retune — optimal
control, sampled.

## What this is, architecturally

`pid_controller` is a *chainable* controller: besides subscribing to a reference topic it exports
reference interfaces that another controller can claim, so a trajectory controller can run
upstream of it in the same 500 Hz update loop — controller → controller → simulated hardware, the
exact structure a real ros2_control robot uses. The controllers are defined in
`config/ur_controllers.yaml`; the launch wiring is in `lampo_gz_mm.launch.py`.

Servo sits one level up: it is an ordinary ROS node *outside* the ros2_control update loop,
publishing velocity setpoints into `forward_velocity_controller` over a topic — the classic
"planner-ish thing feeds a dumb fast controller" split. Its config is `config/ur_servo.yaml`;
what it knows about the robot's shape comes from `urdf/lampo_ur.srdf.xacro`.

The dynamics labs (4–6) use the same split one layer rawer: `forward_effort_controller` is a
torque passthrough, and the intelligence lives in small Python nodes
(`lampo_description/zero_g.py`, `joint_observer.py`, `lqr_joint.py`) built on a shared
pinocchio model of the arm (`lampo_description/arm_dynamics.py`) parsed from the same URDF the
simulator runs. They are deliberately short and commented — read them, then change them.

## Proposed future exercises

Designed but not yet implemented — in rough order of ambition:

1. **The base cascade.** Convert the wheels to `ros2_control`: Twist → mecanum drive controller →
   four chained wheel-velocity PIDs → wheel torque. Tune the inner loop and watch twist tracking
   degrade and recover. (Verified feasible on Kilted: `mecanum_drive_controller` chains onto
   `pid_controller` reference interfaces; the differential base cannot, so it would stay on its
   Gazebo plugin.)
2. **System identification.** Command a step, record the response, fit a first-order model — a
   gain and a time constant — then *compute* PID gains from the model and compare with what you
   found by hand in exercise 1.
3. **Write your own PID.** `nodo_prova.py` is a pure-P go-to-goal controller on the base. Extend
   it to a full PID, feed it through `twist_mux` as a third input instead of publishing directly,
   and race it against Nav2.
4. **Delay and instability.** Insert a configurable transport delay into the cmd_vel path and
   find, experimentally, how much delay your well-tuned loop tolerates before it goes unstable.
5. **Whole-body Cartesian.** Exercise 3 servos the arm relative to its own base; the base itself
   can translate at the same time. Nothing packaged on Kilted coordinates the two, so the
   exercise would be to write the split yourself: given a desired tool twist in the world frame,
   decide how much the base takes and how much the arm takes, and publish both. The redundancy —
   nine actuated freedoms for a six-freedom task — is the whole point.
