# ROS2_ardupilot — Technical Documentation

How the simulated ArduPilot drone swarm works inside, why it was built this way, and how to extend it.

| | |
|---|---|
| Stack | ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8.15) · ArduPilot Copter-4.7.2 SITL · MAVROS 2.16 · zenoh-bridge-ros2dds 1.10.2 · QGroundControl 5.1.5 |
| Companion document | [README.md](README.md) (in Portuguese): installation, quick start, every launch argument, CLI usage, validation results |
| This document | architecture, data paths, internals, design decisions, limitations, and extension recipes (more topics, other control levels) |

---

## Contents

1. [Reading guide](#1-reading-guide)
2. [The mental model](#2-the-mental-model)
3. [Runtime architecture](#3-runtime-architecture)
4. [Data paths in detail](#4-data-paths-in-detail)
5. [Frames, units and conventions](#5-frames-units-and-conventions)
6. [Design decisions and their rationale](#6-design-decisions-and-their-rationale)
7. [Known limitations](#7-known-limitations)
8. [Getting more ROS 2 topics out of the simulation](#8-getting-more-ros-2-topics-out-of-the-simulation)
9. [Controlling the drone at other levels](#9-controlling-the-drone-at-other-levels)
10. [Debugging toolkit](#10-debugging-toolkit)
11. [Where to change what](#11-where-to-change-what)
- [Appendix A — Non-obvious behaviours and pitfalls](#appendix-a--non-obvious-behaviours-and-pitfalls)

---

## 1. Reading guide

**Who this is for.** Engineers and students who need to understand, modify or extend this simulation. It assumes ROS 2 basics (nodes, topics, services, launch files, TF). Some MAVLink / ArduPilot knowledge helps but is not required: the relevant parts are explained where they matter.

**Suggested path.** Read sections 2–5 once to build the mental model. Section 6 explains *why* things are the way they are: read it before changing an architectural piece. Sections 8 and 9 are cookbooks; jump to them when you need them.

**Notation used throughout.**

| Symbol | Meaning |
|---|---|
| `id` | drone id, integer ≥ 1 |
| `i` | SITL instance = `id − 1` |
| `<name>` | drone name, default `drone<id>` (ROS namespace, Gazebo model name, TF prefix) |
| `<w>` | Gazebo world name (`runway`, `farm`, `village`, `disaster`) |
| FC | flight controller: ArduPilot (SITL in simulation, Pixhawk-class hardware on the real drone) |
| GCS | ground control station (ROS domain 0 + QGroundControl) |

Paths are relative to the repository root (`ROS2_ardupilot/`, mounted at `/ROS2_ardupilot/src` in the container). Commands are run inside the container (`docker exec -it ardupilot_jazzy_container bash`) unless stated otherwise.

---

## 2. The mental model

> **ArduPilot flies. ROS 2 decides.**

The system is split at the same boundary as a real drone: the **flight controller** stabilises and navigates the vehicle; the **companion computer** (ROS 2) decides *where* to go and sends targets. Everything above the MAVLink link is identical in simulation and on hardware; everything below it is replaced by the simulator.

```
 ┌──────────────── simulation only ────────────────┐          ┌──────────────── identical on the real drone ────────────────┐
  Gazebo physics ⇄ ArduPilotPlugin ⇄ ArduPilot SITL  ═MAVLink═  MAVROS ⇄ drone_commander ⇄ your nodes ⇄ Zenoh bridge ⇄ other drones / GCS
  (1 kHz)          (JSON over UDP)   (= FC firmware)   (TCP; serial    (MAVLink ↔ ROS)  (high-level API)                   ("radio")
                                                       on hardware)
```

Control happens in nested loops, each closed by a different component at a different rate:

```
 level                     rate          loop closed by                          runs in
 ─────────────────────────────────────────────────────────────────────────────────────────────────────────
 swarm / mission / tasks   1–10 Hz       your nodes, swarm_control               any ROS domain (via Zenoh)
 GUIDED targets            20 Hz         drone_commander → MAVROS                drone's ROS domain (companion)
 ══════════════════════════════════════  MAVLink link  ════════════════════════════════════════════════════
 position / velocity       400 Hz        ArduPilot AC_PosControl                 flight controller
 attitude / body rates     400 Hz        ArduPilot AC_AttitudeControl            flight controller
 motor mixing              400 Hz        ArduPilot AP_Motors                     flight controller
 rotor speed               1000 Hz       ArduPilotPlugin PID on the rotor joint  Gazebo (simulation only)
 rigid-body dynamics       1000 Hz       Gazebo physics (lock-step with SITL)    Gazebo (simulation only)
```

(400 Hz is ArduCopter's default main-loop rate, `SCHED_LOOP_RATE`.) Section 9 shows how to move your code down this ladder when GUIDED position/velocity targets are not enough.

---

## 3. Runtime architecture

### 3.1 Packages

| Package | Kind | Responsibility | Needed on the real drone? |
|---|---|---|---|
| [robot_description](robot_description/) | ament_cmake (data) | Drone models as SDF xacro (frames, rotors, IMU, camera, ArduPilotPlugin) | no |
| [robot_gazebo](robot_gazebo/) | ament_cmake + C++ + Python | Worlds, scenery models, spawn / bridge launches, `clock_relay`, `set_wind`, `world_info` helpers | no |
| [flight_controller](flight_controller/) | ament_cmake (data + launch) | ArduPilot parameter files, SITL launch, MAVROS launch + plugin allowlist | MAVROS part: yes |
| [drone_interfaces](drone_interfaces/) | rosidl | `DroneState`, `GlobalTarget`, `Takeoff`, `SetMode` | yes |
| [drone_control](drone_control/) | ament_python | `drone_commander` (the API node), `drone_cli`, `teleop_keyboard`, `geo` helpers | yes |
| [swarm_control](swarm_control/) | ament_python | `swarm_cli`, `swarm_monitor`, `formation_follower` example, formation geometry | yes |
| [drone_bringup](drone_bringup/) | ament_cmake (launch + config) | `drone`, `swarm`, `gcs`, `real_drone` launches, Zenoh config, RViz config | yes |

Dependency direction (no cycles): `drone_bringup` → {`robot_gazebo` → `robot_description`}, `flight_controller`, `drone_control` → `drone_interfaces`, `swarm_control` → `drone_interfaces`. The simulation-only packages are never imported by the packages that run on hardware.

### 3.2 Processes of one drone

`ros2 launch drone_bringup drone.launch.py id:=N` ([drone.launch.py](drone_bringup/launch/drone.launch.py)) starts, inside `ROS_DOMAIN_ID = N`:

| Process | Started by | When | Role |
|---|---|---|---|
| `ros_gz_sim create` (exits after spawning) | [spawn_drone.launch.py](robot_gazebo/launch/spawn_drone.launch.py) | t = 0 | runs xacro → `/tmp/ardupilot_drones/<name>.sdf`, removes any stale model with the same name, spawns it |
| `arducopter` | [sitl.launch.py](flight_controller/launch/sitl.launch.py) | t = 0 | the firmware; cwd `~/.ros/ardupilot_sitl/<name>/` (EEPROM, logs, Lua scripts) |
| `parameter_bridge` (`/<name>/gz_bridge`) | [drone_bridge.launch.py](robot_gazebo/launch/drone_bridge.launch.py) | t = 0 | camera + ground truth, lazy |
| `clock_relay` | drone_bridge.launch.py | t = 0 | Gazebo clock → `/clock` at 100 Hz in this domain |
| 2 × `static_transform_publisher` | drone.launch.py | t = 0 | `base_link → camera_link → camera_optical_frame` |
| `mavros_node` (respawn) | [mavros.launch.py](flight_controller/launch/mavros.launch.py) | t = 3 s | MAVLink ↔ ROS, topics under `/<name>/mavros/` |
| `drone_commander` | drone.launch.py | t = 3 s | high-level API under `/<name>/` |
| `zenoh-bridge-ros2dds` (peer) | drone.launch.py | t = 3 s | the drone's "radio" |
| *on Ctrl-C* | `OnShutdown` handler | exit | removes the model from Gazebo (important, see [Appendix A](#appendix-a--non-obvious-behaviours-and-pitfalls)) |

Shared processes: one `gz sim` ([world.launch.py](robot_gazebo/launch/world.launch.py)); the GCS ([gcs.launch.py](drone_bringup/launch/gcs.launch.py), domain 0): Zenoh **router**, `swarm_monitor`, RViz, optionally QGroundControl.

The launches write their generated files to `/tmp/ardupilot_drones/` (`<name>.sdf`, `<name>_gz_bridge.yaml`, `<name>_mavros.yaml`). Reading them is the quickest way to see what was actually configured.

### 3.3 Start-up timeline of one drone

```
 t = 0 s     model spawned; SITL boots and waits for the first JSON packet; Gazebo keeps running freely
 t ≈ 0.5 s   ArduPilotPlugin ⇄ SITL connected → from now on Gazebo and this SITL advance in lock-step
 t = 3 s     MAVROS connects to tcp:5760+10i; drone_commander starts; Zenoh bridge joins the router
 t ≈ 5 s     drone_commander requests the MAVLink stream rates ("MAVLink stream rates requested")
 t ≈ 45–65 s SITL GPS detected → EKF3 aligned, origin set ("EKF3 IMU0 is using GPS")
             drone_commander averages 10 samples → "local EKF origin at map (x, y): position valid"
             → takeoff and cmd/* are accepted from here on
```

The 3 s delay exists because MAVROS respawning against a closed TCP port only produces noise; SITL needs a moment to open its serial ports.

### 3.4 One integer gives the whole identity

Every per-drone resource is derived from `id`, so no per-drone configuration files exist and nothing collides:

| Resource | Formula | drone1 | drone3 |
|---|---|---|---|
| Name / namespace / TF prefix / Gazebo model | `drone<id>` | `drone1` | `drone3` |
| `MAV_SYSID` (MAVLink system id) | `id` | 1 | 3 |
| `ROS_DOMAIN_ID` | `id` (GCS = 0) | 1 | 3 |
| SITL instance `i` | `id − 1` | 0 | 2 |
| Gazebo ⇄ SITL JSON (UDP) | `9002 + 10·i` | 9002 | 9022 |
| SERIAL0 → MAVROS (TCP) | `5760 + 10·i` | 5760 | 5780 |
| SERIAL1 → QGC (UDP client) | `127.0.0.1:14550` (shared) | | |
| SERIAL2 → MAVProxy / scripts (TCP) | `5762 + 10·i` | 5762 | 5782 |
| Zenoh bridge listen (TCP) | `7447 + id` (GCS router = 7447) | 7448 | 7450 |
| Spawn position | `x = 3·(id − 1)`, `y = 0` (landing pads) | 0 | 6 |
| RTL altitude | `15 + 3·i` m | 15 m | 21 m |

The 10-port stride per instance is ArduPilot's own SITL convention (`--instance`), which leaves room for the three serial ports of each instance.

---

## 4. Data paths in detail

### 4.1 Physics ⇄ firmware: the lock-step JSON loop

Every 1 ms of simulated time:

```
   Gazebo (gz-sim, 1 ms step)                                        ArduPilot SITL (arducopter --model JSON)
 ┌─────────────────────────────────────────┐   JSON / UDP 9002+10i  ┌──────────────────────────────────────────────┐
 │ 1. integrate physics: rotor joints,     │ ─────────────────────► │ 3. synthesise the FC sensors from that truth:│
 │    LiftDrag forces on each blade,       │  time, gyro, accel,    │    2 IMUs, 2 baros, compass, GPS (+ SIM_*    │
 │    WindEffects force on base_link       │  position (NED),       │    noise, lag, bias)                         │
 │ 2. IMU sensor on imu_link (1 kHz)       │  attitude, velocity    │ 4. EKF3, flight mode, position / attitude /  │
 │                                         │                        │    rate controllers, mixer                   │
 │ 6. PWM → rotor velocity target →        │ ◄───────────────────── │ 5. servo outputs (PWM 1100–1900 µs)          │
 │    P-controller → joint torque          │   servo packet          └──────────────────────────────────────────────┘
 └─────────────────────────────────────────┘
```

Facts that follow from this loop and are easy to miss:

- **There are no Gazebo GPS, barometer or magnetometer sensors on the drone.** The only Gazebo sensor the FC uses is the IMU; the plugin also sends the link pose and velocity. SITL computes GPS position from that pose relative to its `--home`, which is why `--home` **must** equal the world's `<spherical_coordinates>`. [world_info.py](robot_gazebo/robot_gazebo/world_info.py) reads the world SDF so that the world file is the single source of truth for the SITL home and for the swarm `map` origin.
- **Sensor noise is added by SITL, not Gazebo.** `profile_realistic.parm` sets `SIM_ACC*_RND`, `SIM_GYR*_RND`, `SIM_BARO_RND`, `SIM_MAG_RND`, `SIM_GPS1_*`. The Gazebo IMU is noise-free on purpose.
- **Motor model.** Channel `c` PWM is normalised to 0..1, multiplied by `±motor_max_speed` (sign = spin direction) and tracked by a P-controller that applies a torque clamped to `±motor_torque_max` on the rotor joint ([ardupilot_plugin.xacro](robot_description/sdf/ardupilot_plugin.xacro)). This yields a first-order motor (≈ 7 ms X500, ≈ 15 ms hexa) with acceleration saturation. Thrust comes from two LiftDrag "blades" per rotor, `T ∝ ω²` ([rotor.xacro](robot_description/sdf/rotor.xacro)). `MOT_THST_EXPO 0.7` in `sitl_common.parm` linearises this for the FC.
- **Frame conversions happen in the plugin**: `modelXYZToAirplaneXForwardZDown` (model FLU → FRD) and `gazeboXYZToNED` (world ENU → NED). The IMU sensor is mounted rolled 180° so that it outputs FRD directly.
- **Lock-step blocks.** Once a SITL instance is connected, the plugin does not let Gazebo finish a step until that SITL answered, and in lock-step mode it never declares the link lost (it re-sends the state and keeps waiting). That is what makes the simulation deterministic and independent of host load — and also why a dead SITL whose model is still in the world freezes **every** drone (see Appendix A).

### 4.2 Firmware ⇄ ROS 2: MAVLink and MAVROS

**SITL serial ports** ([sitl.launch.py](flight_controller/launch/sitl.launch.py)):

| Port | Transport | Consumer | Real-drone equivalent |
|---|---|---|---|
| SERIAL0 | TCP server `5760+10i` | MAVROS | TELEM2 → companion computer (`/dev/ttyAMA0:921600`) |
| SERIAL1 | UDP client → `127.0.0.1:14550` | QGroundControl (auto-connects, sees all drones) | telemetry radio |
| SERIAL2 | TCP server `5762+10i` | MAVProxy (`mavlink_console N`), pymavlink scripts, bench test of `real_drone.launch.py` | spare TELEM / USB |

**MAVROS configuration** is layered (later wins) in [mavros.launch.py](flight_controller/launch/mavros.launch.py):

1. [apm_pluginlists.yaml](flight_controller/config/mavros/apm_pluginlists.yaml): `plugin_denylist: ['*']` + an explicit `plugin_allowlist` (see §8.4).
2. MAVROS's own `apm_config.yaml` (ArduPilot defaults shipped with MAVROS).
3. A generated override `/tmp/ardupilot_drones/<name>_mavros.yaml`: frame ids prefixed with the drone name (`<name>/odom`, `<name>/base_link`), **`tf.send: false`** everywhere (drone_commander owns the TF, §4.3), heartbeat type `ONBOARD_CONTROLLER`, rangefinder publisher config.
4. Inline parameters: `fcu_url`, `tgt_system = sysid`, `system_id = sysid` and `component_id = 191` (MAVROS identifies itself as the onboard computer *of that vehicle*).

MAVROS 2 makes its topics private to the node `mavros`, so it is launched with `namespace=<name>` → topics `/<name>/mavros/...`.

**Message rates.** ArduPilot only streams what it is asked for. Two mechanisms set the rates on the MAVROS link:
- `MAV1_*` parameters in [sitl_common.parm](flight_controller/config/ardupilot/sitl_common.parm) (SITL only; 4.6 and older call them `SR0_*`);
- `drone_commander` calls `mavros/set_stream_rate` for each stream group once connected (`_STREAMS` in [drone_commander.py](drone_control/drone_control/drone_commander.py)): POSITION and EXTRA1 at 20 Hz, EXTRA2 at 10 Hz, the rest at 2 Hz. This works on any firmware version and any serial port, so the real drone needs no `SRx_` setup.

Result: `local_position/*`, `global_position/*` and `imu/data` arrive at **20 Hz** by default. §8.3 shows how to raise them.

### 4.3 Inside `drone_commander`

[drone_commander.py](drone_control/drone_control/drone_commander.py) is the only node that talks to MAVROS. It translates a small, frame-explicit ROS API into GUIDED-mode MAVLink targets and publishes a compact state.

```
      your nodes / CLI / other drones (via Zenoh)
   cmd/velocity  cmd/position  cmd/global   takeoff land rtl hold arm set_mode
          │            │            │               │ (services)
          ▼            ▼            ▼               ▼
   ┌───────────────────────────────────────────────────────────┐   state (10 Hz), pose (20 Hz)
   │ accept_command(): connected? position_valid? GUIDED?      │──► TF map → <name>/odom → <name>/base_link
   │ active command (setpoint_type): NONE|VELOCITY|POSITION|GLOBAL
   │ control_loop() @ setpoint_rate (20 Hz)                    │
   │ map ⇄ local offset estimator                              │
   └──────┬──────────────────────────────┬─────────────────────┘
          │ setpoint_raw/local           │ set_mode, cmd/arming, cmd/takeoff, set_stream_rate
          │ setpoint_raw/global          ▼
          ▼                       MAVROS services
   MAVROS → SET_POSITION_TARGET_LOCAL_NED / _GLOBAL_INT → ArduPilot GUIDED
```

**Threading model.** A `MultiThreadedExecutor` (4 threads) with three callback groups:
- `sub_group` (mutually exclusive): all subscriptions and both timers → state updates and the control loop never run concurrently;
- `srv_group` (reentrant): the API services;
- `cli_group` (reentrant): MAVROS service clients and the stream-request timer.

The API services **block** (e.g. `set_mode` waits until `mavros/state` reports the new mode). That only works because the state subscription lives in another callback group and keeps being served by another thread while the service waits. An `RLock` protects the shared state. If you add a service that waits on FC state, put it in `srv_group`, never in `sub_group`.

**Active-command state machine.** "Last command wins":

| Event | Effect on `setpoint_type` |
|---|---|
| `cmd/velocity` accepted | → `VELOCITY` (body frame rotated to ENU with the current yaw; clamped by `max_horizontal_speed`, `max_vertical_speed`, `max_yaw_rate`) |
| `cmd/position` accepted | → `POSITION` (stored in `map`; all-zero quaternion = keep current yaw) |
| `cmd/global` accepted | → `GLOBAL` (yaw NaN = keep current yaw) |
| no `cmd/velocity` for `velocity_timeout` (0.5 s) | `VELOCITY` → `POSITION` at the current pose ("hold here") |
| `hold` service | → `POSITION` at the current pose |
| `takeoff` service, on the ground | → `NONE`; commands received during the climb are stored and applied once it completes |
| mode change **not requested by the node** (RC, QGC, failsafe) | → `NONE` (the ROS command is cancelled) |

A command is accepted only if the FC is connected, `position_valid` is true and the mode is GUIDED (or `auto_guided:=true` and the drone is armed, in which case the node switches to GUIDED).

**Control loop** (20 Hz, only while connected and in GUIDED):
1. If a takeoff is in progress, do nothing until `rel_alt ≥ 0.9·altitude`, the drone disarms, or a deadline (`15 s + altitude / 1 m/s`) passes.
2. `VELOCITY` → `PositionTarget` with only velocity + yaw rate enabled (`type_mask` ignores position, acceleration, yaw).
3. `POSITION` → target converted `map` → local EKF frame, `PositionTarget` with position + yaw enabled.
4. `GLOBAL` → `GlobalPositionTarget`, `FRAME_GLOBAL_REL_ALT`.

Values are written in ENU with `coordinate_frame = FRAME_LOCAL_NED`; MAVROS converts ENU → NED before sending.

**The `map` ⇄ local offset estimator** (`update_offset`). Each drone's EKF has its own local origin (where it obtained its GPS origin). To command all drones in one shared frame, the node needs the translation from `<name>/odom` to `map`:
1. For every EKF global position (`global_position/global`) that has a local position at most 0.5 s older, compute east/north of the fix relative to the swarm origin **with ArduPilot's flat-earth model** (`geo.ap_latlon_to_en`).
2. Time-align: `sample = (e, n) − (x + vx·dt, y + vy·dt)`, with `dt` the stamp difference (rejected if |dt| > 0.5 s).
3. The first 10 samples are averaged → offset known → `position_valid`.
4. Afterwards the offset is refined with an exponential filter (α = 0.01), only while horizontal speed < 1 m/s (non-simultaneous samples bias the estimate in motion).
5. The vertical offset is 0 by construction: `map` z = local EKF z = height above *this drone's* take-off point.

This method uses only messages that are always streamed (`GLOBAL_POSITION_INT`, `LOCAL_POSITION_NED`), so it does not depend on the one-off EKF-origin message having reached MAVROS. Measured agreement with Gazebo ground truth: < 3 cm horizontally.

`position_valid` = offset known **and** local pose younger than 1 s **and** global fix younger than 2 s.

**Why position targets are re-sent at 20 Hz** although ArduPilot holds a position target indefinitely:
- the local target is recomputed from the `map` target every cycle, so offset refinements are applied;
- a single lost message on a real serial link cannot leave a stale target;
- all command types share one code path and one timing.

**Two timeout layers for velocity:** the node falls back to "hold here" after `velocity_timeout` (0.5 s) without `cmd/velocity`; if the companion computer itself dies, ArduPilot stops after `GUID_TIMEOUT` (3 s) without targets.

**Takeoff sequence** (`srv_takeoff`): checks (altitude > 0.5 m, connected, `position_valid`) → `set_mode GUIDED` → if already flying, become a `POSITION` target at the new altitude → else arm → `MAV_CMD_NAV_TAKEOFF` → start the takeoff gate. The gate exists because a GUIDED target sent while ArduPilot is executing a takeoff *replaces* the takeoff: the drone stays on the ground and auto-disarms.

### 4.4 Between drones: ROS domains and Zenoh

```
  domain 0 (GCS)                 domain 1 (drone1)            domain 2 (drone2)
  zenoh-bridge  router           zenoh-bridge  peer           zenoh-bridge  peer
  listen tcp/0.0.0.0:7447 ◄───── connect :7447                connect :7447 ─────► (router)
                                 listen  :7448 ◄────────────► listen  :7449        (direct peer link, learnt by gossip)
```

- Each drone's DDS traffic stays inside its domain. The **only** path between domains is the bridges, which forward only interfaces matched by the allowlist in [swarm.json5](drone_bringup/config/zenoh/swarm.json5):

  | Kind | Allowed (regex on the full name) |
  |---|---|
  | publishers / subscribers | `/[^/]+/(state\|pose)`, `/[^/]+/cmd/(velocity\|position\|global)`, `/[^/]+/swarm/.*`, `/swarm/.*` |
  | service servers / clients | `/[^/]+/(takeoff\|land\|rtl\|hold\|arm\|set_mode)`, `/[^/]+/swarm/.*` |
  | action servers / clients | `/[^/]+/swarm/.*` |

- `pub_max_frequencies` downsamples `state` and `pose` to 10 Hz on the radio.
- Service calls become Zenoh queries: `queries_timeout` is 30 s for `takeoff` (arming + mode changes), 10 s otherwise.
- Multicast scouting is disabled (no accidental links with other Zenoh nodes on a LAN); gossip is enabled so that peers learn each other's locators from the router and connect directly (mesh). With `zenoh_mode:=client` all traffic goes through the router (star).
- Names are **not** rewritten: `/drone1/state` is `/drone1/state` in every domain, because the nodes are already namespaced.

### 4.5 Time

- Gazebo sim time is the master clock. SITL is lock-stepped to it, so firmware time = sim time.
- Each drone domain gets its own `/clock` from its `clock_relay` (100 Hz). In single-domain mode only one drone may publish it (`clock:=false` on the others).
- The GCS runs with `use_sim_time:=false` (a real GCS has no simulation clock); it only consumes stamped data.
- MAVROS (`sys_time` plugin) synchronises FC time with ROS time, so MAVROS stamps are in sim time.
- In `drone_commander`, command timeouts use the node clock (sim time); the blocking waits inside services use `time.monotonic()` (wall time). With a real-time factor well below 1, service waits are therefore shorter in sim time than they look.

---

## 5. Frames, units and conventions

| Frame | Convention | Origin | Published by |
|---|---|---|---|
| Gazebo world | ENU (x East, y North, z Up) | world `<spherical_coordinates>` | — |
| `map` (swarm frame) | ENU | swarm origin = world origin in simulation, `origin_lat/lon` on hardware | `drone_commander` (`map → <name>/odom`) |
| `<name>/odom` | ENU | the drone's EKF origin (first GPS lock) | `drone_commander` (`<name>/odom → <name>/base_link`) |
| `<name>/base_link` | FLU (x forward, y left, z up) | body | — |
| `<name>/camera_link` | FLU, pitched down by `camera_pitch` | camera mount | static TF (drone.launch.py) |
| `<name>/camera_optical_frame` | optical (z forward, x right, y down) | camera | static TF |
| ArduPilot internal | NED / FRD | EKF origin | — (MAVROS converts) |

TF tree in a drone's domain:

```
map ──► <name>/odom ──► <name>/base_link ──► <name>/camera_link ──► <name>/camera_optical_frame
```

The GCS sees a separate tree published by `swarm_monitor`: `map → <name>/base_link` for each drone (built from `state`, since TF does not cross the radio).

Conventions that apply to every interface:
- **Yaw** is ENU: 0 = East, counter-clockwise positive, radians. (Compass heading = 90° − yaw.)
- **Altitude**: `map` z and `cmd/position` z are height above the drone's own take-off point; `cmd/global.altitude` is metres above home (like QGC "fly here"); `relative_altitude` in `DroneState` is the FC's height above home.
- **Body-frame velocity** (`frame_id: base_link`): rotated to ENU with the yaw at the moment the message is received; z is always vertical (heading-relative, not attitude-relative). Stream it so that turning while moving keeps working.
- **Geodesy**: lat/lon ⇄ metres uses ArduPilot's spherical flat-earth model (`LATLON_TO_M`, longitude scaled by cos(mean latitude)). With it, `map` is a pure translation of each EKF frame and coincides with the Gazebo world. Exact WGS84 differs by ≈ 0.5 % (≈ 0.26 m at 50 m). `geo.py` also provides exact WGS84 ↔ ENU for other uses (e.g. geolocating detections far away).
- **Ground truth** (`/<name>/ground_truth/odometry`) is expressed in the Gazebo world frame (= `map`), child `<name>/base_link`. Expect a constant vertical offset versus `pose`/`state` equal to the height of `base_link` when standing on the pad (≈ 0.27 m for the X500), because the EKF's zero is the take-off point.

---

## 6. Design decisions and their rationale

Each entry: the decision, why it was taken, and what it costs or implies.

### D1 — ArduPilot does the flight control; ROS 2 only sends GUIDED targets
**Why.** The firmware is flight-proven: tuned cascaded controllers, EKF3, failsafes, geofence, landing detection. ROS 2 nodes (especially Python over DDS) are not real-time and should not stabilise a multirotor. The same split is used on the real drone, so the simulation exercises exactly the code that will fly. If the companion computer crashes, the FC degrades gracefully (`GUID_TIMEOUT`, RC failsafe) instead of falling.
**Implies.** High-level code is limited to what GUIDED accepts. Section 9 explains how to go lower when needed.

### D2 — The real firmware (SITL), lock-stepped with Gazebo at 1 kHz
**Why.** SITL is the FC code compiled for Linux: parameters, modes, bugs and quirks carry over to hardware (build it with the same `ARDUPILOT_VERSION` as the drone). The JSON backend is ArduPilot's maintained interface to external simulators. Lock-step makes results independent of host load: with many drones or cameras the real-time factor drops, but every controller still sees a consistent 1 ms step — a slowdown, not a loss of fidelity.
**Implies.** All worlds must use a 1 ms physics step. A stalled SITL stalls the world (Appendix A).

### D3 — The real EKF3, not SITL's perfect AHRS
**Why.** The ArduPilotPlugin defaults to `no_time_sync=1`, which makes SITL switch to `AHRS_EKF_TYPE 10` (perfect attitude and position from the simulator). That hides estimation latency and error, which are exactly what break high-level algorithms on hardware. The model sets `<no_time_sync>0` and the parameters force `AHRS_EKF_TYPE 3`.
**Implies.** ~45–65 s start-up until the EKF uses GPS; positions have realistic small errors (especially with `profile:=realistic`).

### D4 — MAVROS, not ArduPilot's native DDS (AP_DDS / micro-ROS)
**Why.** AP_DDS requires firmware built with DDS support and exposes a fixed set of `/ap/...` topics without per-vehicle namespaces. MAVROS works with the standard firmware on any FC, covers everything needed (local/global targets, missions, parameters, RTK injection, gimbal) and accepts a namespace per drone.
**Implies.** One `mavros_node` per drone (≈ 17 % of a core each, with the plugin allowlist). Because user code only talks to `drone_commander`, the bridge could be swapped later without touching user nodes.

### D5 — One high-level API node per drone, built on `setpoint_raw`
**Why.**
- A single choke point makes safety rules (GUIDED only, timeouts, limits, takeoff gating, pilot override) impossible to bypass by accident.
- `setpoint_raw` exposes the full `SET_POSITION_TARGET_*` semantics with explicit type masks; the convenience plugins (`setpoint_position`, `setpoint_velocity`) send the same MAVLink messages with less control.
- The API uses standard ROS types where one fits (`PoseStamped`, `TwistStamped`, `std_srvs`), so generic tools (teleop, planners, `ros2 topic pub`) can drive it. Custom types exist only where nothing standard fits (`DroneState`, `GlobalTarget`, `Takeoff`, `SetMode`).
- The same node runs unchanged on hardware.
**Implies.** Adding a new command type means extending this node (pattern in §9.10).

### D6 — A mode change made outside the node cancels the ROS command
**Why.** The safety pilot's mode switch (RC, QGC) and FC failsafes must always win, and switching back to GUIDED must not make the vehicle fly to a stale target. The node distinguishes its own GUIDED requests (5 s window) from external changes.
**Implies.** After an external mode change, a new command (or `hold`) is required.

### D7 — Commands are held during takeoff
**Why.** ArduPilot replaces an in-progress GUIDED takeoff with any new GUIDED target; the drone then never leaves the ground and auto-disarms. Holding commands until 90 % of the takeoff altitude lets a script call `takeoff` then `goto` without sleeping.

### D8 — A shared `map` frame computed with ArduPilot's geodesy
**Why.** Swarm algorithms need all drones in one metric frame, but each EKF has its own origin. Converting lat/lon with exact WGS84 left a ≈ 0.5 % scale error relative to the EKF frames (which use ArduPilot's flat-earth model); using the same model makes the relation an exact translation and makes `map` equal to the Gazebo world in simulation.
**Implies.** On hardware, inter-drone accuracy is that of each GPS (≈ 1–2 m standard, cm with RTK). `map` z is per-drone (height above own take-off point).

### D9 — One ROS domain per drone, connected by `zenoh-bridge-ros2dds`
**Alternatives considered.** (a) One DDS domain for everything: simplest, but hides the network boundary — code that accidentally depends on another drone's MAVROS topics works in simulation and fails in the field — and DDS discovery/multicast scales poorly over Wi-Fi. (b) `rmw_zenoh` (installed in the image): routes every topic through Zenoh, with no per-interface allowlist or rate limit at the drone boundary, and requires every node on every machine to use the same RMW.
**Why the bridge.** It reproduces "separate computers + radio" on one host; only allowlisted interfaces cross, with rate limits; on-board traffic stays on the default DDS (CycloneDDS); router, mesh and client topologies are configuration.
**Implies.** Anything not allowlisted is invisible to other drones and to the GCS — deliberately (§8.8). Each terminal must choose a domain (`drone_domain N` / `gcs_domain`).

### D10 — No `namespace` option on the bridge; the nodes are namespaced instead
**Why.** The bridge's `namespace` option prefixes everything a robot publishes. That suits robot ↔ base station, but then robot A cannot subscribe to robot B's topics under their real names. Since the nodes already live in `/<name>/`, the bridge only filters.

### D11 — `/clock` relayed at 100 Hz by a C++ node
**Why.** Gazebo publishes its clock every physics step (1 kHz). Every node with `use_sim_time` subscribes to `/clock`, and MAVROS creates one ROS node per plugin: at 1 kHz this alone cost about one CPU core per drone. 100 Hz (10 ms resolution) is ample for 20 Hz control, timers and TF; sensors keep their exact sim-time stamps. C++ because the relay itself must handle 1 kHz cheaply.
**Implies.** `rate()`-based loops have 10 ms granularity in sim time. Raise `clock_rate` if you need finer timer resolution.

### D12 — MAVROS plugin allowlist
**Why.** MAVROS loads ≈ 60 plugins by default; most are irrelevant for an ArduPilot copter. Denying all and allowing ≈ 25 reduces CPU, memory, topics and discovery traffic per drone.
**Implies.** A MAVROS feature you expect may simply be disabled; check the allowlist first (§8.4).

### D13 — Drone models are SDF written in xacro (not URDF); no `robot_state_publisher`
**Why.** The models rely on Gazebo features the URDF → SDF conversion does not preserve well: LiftDrag plugins per blade, fixed joints kept as separate links (URDF conversion lumps them), and sensors referenced by scoped name from the ArduPilotPlugin. Without a URDF there is no `robot_state_publisher`; the only dynamic TF is the drone pose (from `drone_commander`) and static TFs are published by the launch file.
**Implies.** When you add a sensor you also add its static TF and keep `frames.yaml` consistent with the xacro (§8.6).

### D14 — Rotor model copied from the official ArduPilot iris
**Why.** It is the model the ArduPilot developers validate SITL + Gazebo with: PWM → velocity PID on the joint → LiftDrag thrust. Sizing (blade area, centre of pressure, max speed, damping, motor torque) was computed per airframe to match real thrust-to-weight, hover throttle, yaw authority (≈ 0.016 N·m per N of thrust) and motor time constant; the derivations are in the headers of [frame_x500v2.xacro](robot_description/sdf/frame_x500v2.xacro) and [frame_hexa.xacro](robot_description/sdf/frame_hexa.xacro).
**Implies.** The provided gains are a good *starting point* for hardware, not a tune (use AutoTune / QuikTune on the real airframe).

### D15 — Layered parameter files, EEPROM wiped at each start
**Why.** Load order: ArduPilot `copter.parm` → `sitl_common.parm` → `<frame>.parm` → `profile_<p>.parm` → generated `instance.parm` (per-drone RTL altitude) → `extra_params`. Each concern lives in one file, and `extra_params` can load a parameter dump from the real drone last. `wipe:=true` (default) guarantees that a run starts from the files, not from whatever was changed in QGC last time.
**Implies.** Changes made in QGC are lost on restart unless `wipe:=false`. Parameters that need a reboot (e.g. `SCR_ENABLE`, `CC_TYPE`) belong in a `.parm` file.

### D16 — ArduPilot 4.7 SI parameter names
**Why.** 4.7 renamed many parameters to SI units (`WP_SPD` m/s, `RTL_ALT_M`, `LAND_SPD_MS`, `ATC_ACC_*_MAX` deg/s², `MAV_SYSID`, `MAVn_*`). SITL **silently ignores** unknown names in `--defaults`, so wrong-version names fail without error. `instance.parm` writes both `RTL_ALT_M` and `RTL_ALT` so it works on both sides of the rename.
**Implies.** If you build another `ARDUPILOT_VERSION`, check the important parameters with `ros2 param get /<name>/mavros/param <NAME>`.

### D17 — Wind is applied by Gazebo, not by SITL
**Why.** `SIM_WIND_*` only affects SITL's internal physics models, which are not used with the JSON backend. The Gazebo `WindEffects` system pushes the drone body, so ArduPilot only perceives wind through its effect on the vehicle — as in reality — and its own wind estimate (`mavros/wind_estimation`) can be validated. The force coefficient (`k = 0.37`) was calibrated in flight because the measured force grows with k² (≈ 1.1·k²·m·v), giving ≈ 3 N (X500) / 9 N (hexa) at 10 m/s.
**Implies.** The system is inactive until the first wind message; `world.launch.py` sends one after 5 s, and `set_wind` verifies it through `/world/<w>/wind_info`.

### D18 — Static scenery and in-house building models
**Why.** Dynamic Fuel models (cones, barrels) cut the real-time factor to ≈ 0.65; several Fuel buildings use OGRE-1 materials that render black in ogre2. Scenery is forced `<static>true</static>` and the black houses were replaced by models in `robot_gazebo/models`.

### D19 — Launch architecture: `OpaqueFunction` + scoped groups
**Why.** Per-drone values (ports, domains, generated files) are computed in Python at launch time. Included launch files share launch configurations with their parent, so arguments of one drone leaked into the next; `swarm.launch.py` wraps each include in `GroupAction(scoped=True, forwarding=False)`. A `TimerAction` inside a scoped group fires after the group's environment was popped, so the delayed actions re-set `ROS_DOMAIN_ID` inside their own scoped group.

### D20 — Docker conventions
**Why.** Host networking (QGC UDP 14550, Zenoh TCP, DDS), NVIDIA runtime for ogre2 rendering, X11 forwarding; the repository is mounted at `/ROS2_ardupilot/src` so edits on the host are live. MAVProxy lives in its own venv and pymavlink is installed with `--no-deps` so pip never replaces the apt numpy / OpenCV used by ROS (`cv_bridge`). QGC runs as user `ubuntu` because it refuses to run as root. Fuel models are downloaded at build time so the simulation runs offline.

### D21 — GCS failsafe off; RTL altitudes staggered
**Why.** In a swarm the drones are commanded by ROS 2, not by one GCS link, so `FS_GCS_ENABLE 0`; RC, EKF and crash failsafes stay enabled. RTL altitude is `15 + 3·i` m so that drones returning together do not converge at the same altitude.

### D22 — Lazy bridges for camera and ground truth
**Why.** A camera costs GPU rendering and DDS bandwidth only if someone subscribes. With `lazy: true`, `ros_gz_bridge` creates the Gazebo subscription only when a ROS subscriber exists.

---

## 7. Known limitations

| Area | Limitation | Workaround / pointer |
|---|---|---|
| Battery | Constant 12.6 V: the JSON backend only simulates consumption if the simulator sends voltage/current, and the plugin does not | Test battery failsafes on the bench or with a node that publishes a battery model |
| GPS-denied flight | No rangefinder, optical flow or vision position source by default | §8.7 (feed sensors to ArduPilot), §9.5 (`GUIDED_NOGPS`) |
| GPS / baro / compass | Synthesised by SITL from ground truth; buildings cause no multipath or occlusion | Use `profile:=realistic`; raise `SIM_GPS1_NOISE` for stress tests |
| Altitude reference | `map` z is height above each drone's own take-off point | Take off from pads at the same height (true in all worlds) |
| Terrain | Worlds are flat; SITL terrain following is off | Add a heightmap and enable terrain in ArduPilot if needed |
| Aerodynamics | No ground effect, vortex-ring state or blade flapping; body drag is isotropic (wind model) | Acceptable for guidance-level work; tune on hardware |
| Camera | Fixed mount, no simulated gimbal (the `mount_control` plugin is enabled for real drones) | Add a gimbal joint + `JointPositionController` if needed |
| Radio | Zenoh links are loopback: no latency, loss or range limits | Shape only the Zenoh ports with `tc netem` if you need it (not provided) |
| Collision avoidance | None; collisions are simulated and ArduPilot disarms on crash | Separate altitudes (`--alt-step`), add avoidance in your swarm layer |
| Scale | All drones share one host; RTF drops with N (correctness is kept by lock-step) | Disable cameras (`camera:=false`), use `runway`, `gui:=false` |
| QGC | Only one program can bind UDP 14550 | Close the host QGC or change `gcs_udp` |

---

## 8. Getting more ROS 2 topics out of the simulation

### 8.1 First: where does the data live?

Every piece of data in this system has exactly one source. Pick the recipe from the source, not from the topic you wish existed.

| You want… | Source | Recipe |
|---|---|---|
| FC estimates and status: attitude, EKF position/velocity, RC in, servo outputs, vibration, wind estimate, EKF status, home… | ArduPilot → MAVLink → MAVROS | Usually already published (§8.2). Faster: §8.3. Missing: §8.4 |
| Simulator truth: true pose/velocity, true IMU, poses of every model, rotor speeds, real-time factor | Gazebo transport | §8.5 (bridge an existing Gazebo topic) |
| A new physical sensor: lidar, depth/RGB-D camera, second camera, thermal, Gazebo GPS/magnetometer | Gazebo sensor added to the drone SDF | §8.6 |
| ArduPilot should *use* a sensor (rangefinder, proximity, external navigation) | ROS → MAVROS → ArduPilot | §8.7 |
| Other drones / the GCS should receive it | Zenoh bridge allowlist | §8.8 |
| Any MAVLink message, or high-rate data after the fact | raw MAVLink, dataflash logs | §8.9 |

### 8.2 Look before you add

```bash
drone_domain 1                      # everything on board drone1
ros2 topic list -t                  # ~80 topics: /drone1/mavros/..., camera, ground truth, /tf, /clock
gz topic -l                         # Gazebo topics, bridged or not
gcs_domain                          # what crosses the radio
ros2 topic list
```

Gazebo topics that exist today without being bridged to ROS (measured on `runway` with two drones):

| Gazebo topic | Type | Content |
|---|---|---|
| `/world/<w>/model/<name>/link/imu_link/sensor/imu_sensor/imu` | `gz.msgs.IMU` | true IMU at 1 kHz, noise-free, **FRD** (sensor rolled 180°) |
| `/model/<name>/pose` | `gz.msgs.Pose_V` | model pose (from the OdometryPublisher system) |
| `/model/<name>/odometry_with_covariance` | `gz.msgs.OdometryWithCovariance` | ground truth with covariance |
| `/world/<w>/dynamic_pose/info` | `gz.msgs.Pose_V` | poses of all dynamic models (all drones) |
| `/world/<w>/stats` | `gz.msgs.WorldStatistics` | sim time, real-time factor, iterations |
| `/world/<w>/wind` | `gz.msgs.Wind` | **input**: wind command (use `set_wind`) |
| `/model/<name>/joint/rotor_<k>_joint/cmd_force` | `gz.msgs.Double` | **input**: rotor torque (ApplyJointForce). Do not publish while the ArduPilotPlugin drives the rotors |

### 8.3 Recipe A — faster MAVROS data

`imu/data`, `local_position/*` and `global_position/*` arrive at 20 Hz because of the stream rates (§4.2). Ask ArduPilot for a specific message at a specific rate:

```bash
drone_domain 1
# ATTITUDE (id 30) at 100 Hz → /drone1/mavros/imu/data follows it
ros2 service call /drone1/mavros/set_message_interval mavros_msgs/srv/MessageInterval \
  "{message_id: 30, message_rate: 100.0}"
ros2 topic hz /drone1/mavros/imu/data
```

| MAVLink message (id) | MAVROS topic(s) fed |
|---|---|
| `ATTITUDE` (30) | `imu/data` (orientation, angular velocity) |
| `RAW_IMU` (27), `SCALED_IMU` (26) | `imu/data_raw`, `imu/mag` |
| `LOCAL_POSITION_NED` (32) | `local_position/pose`, `velocity_local`, `odom` |
| `GLOBAL_POSITION_INT` (33) | `global_position/global`, `rel_alt`, `local`, `compass_hdg` |
| `SERVO_OUTPUT_RAW` (36) | `rc/out` (motor PWM) |
| `RC_CHANNELS` (65) | `rc/in` |
| `VFR_HUD` (74) | `vfr_hud` |

To make a rate permanent:
- either add the call to `drone_commander.request_streams()` right after the stream-group requests (stream requests issued later can reset per-message intervals, and `drone_commander` re-requests streams after every reconnection);
- or change a whole group: edit `_STREAMS` in [drone_commander.py](drone_control/drone_control/drone_commander.py) (and the matching `MAV1_*` in `sitl_common.parm`).

On hardware, budget the serial link (921600 baud ≈ 90 kB/s): 100 Hz ATTITUDE is fine; high-rate everything is not.

### 8.4 Recipe B — enable another MAVROS plugin

Plugins available in this image include, among others: `actuator_control`, `adsb`, `altitude`, `debug_value`, `esc_status`, `esc_telemetry`, `fake_gps`, `gimbal_control`, `landing_target`, `manual_control`, `mocap_pose_estimate`, `nav_controller_output`, `obstacle_distance`, `odometry`, `optical_flow`, `play_tune`, `rangefinder`, `terrain`, `trajectory`, `vision_pose`, `vision_speed`, `wheel_odometry`. List them all with:

```bash
grep -h 'class name' /opt/ros/jazzy/share/mavros*/mavros_plugins.xml | sed 's/.*name="\([^"]*\)".*/\1/' | sort
```

Add the name to `plugin_allowlist` in [apm_pluginlists.yaml](flight_controller/config/mavros/apm_pluginlists.yaml), then restart the drone (with `--symlink-install`, edits to existing installed files are picked up without rebuilding; new files need `colcon build`). Example: `nav_controller_output` publishes the FC's own navigation errors (`/drone1/mavros/nav_controller_output/output`), `esc_status` publishes per-ESC RPM/current on hardware.

Plugins that need configuration (frame ids, sensor ids) get it in the override dictionary of [mavros.launch.py](flight_controller/launch/mavros.launch.py) — see how `distance_sensor` is configured there. Note: `actuator_control` sends `SET_ACTUATOR_CONTROL_TARGET`, which ArduPilot does not implement (PX4 only).

### 8.5 Recipe C — bridge a Gazebo topic that already exists

All per-drone bridging lives in one list in [drone_bridge.launch.py](robot_gazebo/launch/drone_bridge.launch.py). Example: the true 1 kHz IMU, for comparing the FC's estimate against ground truth.

1. Pass the world name to the bridge launch. In [drone.launch.py](drone_bringup/launch/drone.launch.py), add `'world_name': winfo['name']` to the `launch_arguments` of the `drone_bridge.launch.py` include. In `drone_bridge.launch.py`, declare `DeclareLaunchArgument('world_name', default_value='runway')`.
2. Add the entry in `bridge()`:

```python
world = LaunchConfiguration('world_name').perform(context)
entries.append(
    {'ros_topic_name': f'/{name}/ground_truth/imu',
     'gz_topic_name': f'/world/{world}/model/{name}/link/imu_link/sensor/imu_sensor/imu',
     'ros_type_name': 'sensor_msgs/msg/Imu', 'gz_type_name': 'gz.msgs.IMU',
     'direction': 'GZ_TO_ROS', 'lazy': True})
```

3. Rebuild if the file is new, restart the drone, check: `drone_domain 1; ros2 topic hz /drone1/ground_truth/imu`.

Remember that this IMU is FRD (the sensor is rolled 180° for ArduPilot) and noise-free; and that 1 kHz over DDS is not free — keep it `lazy`.

**Example: rotor speeds.** No system publishes joint states yet. Add a `JointStatePublisher` to the model, e.g. inside the `gazebo_essentials` macro of [gazebo_essentials.xacro](robot_description/sdf/gazebo_essentials.xacro):

```xml
<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
  <topic>${name}/rotor_states</topic>
  <update_rate>50</update_rate>
</plugin>
```

and bridge it (`gz.msgs.Model` → `sensor_msgs/msg/JointState`):

```python
{'ros_topic_name': f'/{name}/rotor_states', 'gz_topic_name': f'/{name}/rotor_states',
 'ros_type_name': 'sensor_msgs/msg/JointState', 'gz_type_name': 'gz.msgs.Model',
 'direction': 'GZ_TO_ROS', 'lazy': True}
```

Without `<joint_name>` elements it publishes every joint of the model; the rotor velocities are in `velocity[]` for `rotor_<k>_joint` (rad/s, sign = spin direction).

**World-level topics** (`/world/<w>/stats`, `/world/<w>/dynamic_pose/info`) are not per drone: bridge them once, e.g. from `world.launch.py` with a `ros_gz_bridge parameter_bridge` node running in the GCS domain or in a dedicated domain.

### 8.6 Recipe D — add a new sensor to the drone model

Example: a 3D lidar (a 2D scanner is the same with `vertical samples = 1`). Five steps.

**1. Write the sensor macro** — `robot_description/sdf/lidar.xacro`:

```xml
<?xml version="1.0"?>
<sdf version="1.9" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="lidar" params="name x z pitch:=0 hsamples:=360 vsamples:=16
                                    vmin:=-0.26 vmax:=0.26 range_max:=40 rate:=10">
    <link name="lidar_link">
      <pose>${x} 0 ${z} 0 ${pitch} 0</pose>
      <xacro:inertial_box mass="0.05" x="0.05" y="0.05" z="0.04"/>
      <sensor name="lidar" type="gpu_lidar">
        <gz_frame_id>${name}/lidar_link</gz_frame_id>
        <topic>${name}/lidar</topic>
        <update_rate>${rate}</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <lidar>
          <scan>
            <horizontal><samples>${hsamples}</samples><resolution>1</resolution>
                        <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
            <vertical><samples>${vsamples}</samples><resolution>1</resolution>
                      <min_angle>${vmin}</min_angle><max_angle>${vmax}</max_angle></vertical>
          </scan>
          <range><min>0.2</min><max>${range_max}</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
        </lidar>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>
  </xacro:macro>
</sdf>
```

**2. Mount it.** Include `lidar.xacro` in [drone.sdf.xacro](robot_description/sdf/drone.sdf.xacro) and instantiate it inside `drone_frame` in the frame file (next to the camera), e.g. `<xacro:lidar name="${name}" x="0" z="0.10"/>`. If it should be optional, add an xacro arg like the existing `camera` arg and forward it from `spawn_drone.launch.py` / `drone.launch.py`. Keep added masses small: the IMU must stay at the centre of gravity and the hover throttle shifts with mass (`MOT_HOVER_LEARN 2` re-learns it).

**3. Check the world has the systems the sensor needs.** All worlds load `Sensors` (ogre2: cameras, depth cameras, `gpu_lidar`), `Imu`, `Physics`, `UserCommands`, `SceneBroadcaster`, `WindEffects`. Sensors with their own system must have it added to **every** world SDF:

| Sensor type | World system needed |
|---|---|
| `camera`, `depth_camera`, `rgbd_camera`, `gpu_lidar`, `segmentation_camera` | `gz-sim-sensors-system` (present) |
| `navsat` | `gz-sim-navsat-system` (noise stddev is in **degrees**) |
| `magnetometer` | `gz-sim-magnetometer-system` |
| `air_pressure` | `gz-sim-air-pressure-system` |
| `altimeter` | `gz-sim-altimeter-system` |
| `contact` | `gz-sim-contact-system` |

**4. Bridge it** — add to the entries in [drone_bridge.launch.py](robot_gazebo/launch/drone_bridge.launch.py):

```python
{'ros_topic_name': f'/{name}/lidar/scan', 'gz_topic_name': f'/{name}/lidar',
 'ros_type_name': 'sensor_msgs/msg/LaserScan', 'gz_type_name': 'gz.msgs.LaserScan',
 'direction': 'GZ_TO_ROS', 'lazy': True},
{'ros_topic_name': f'/{name}/lidar/points', 'gz_topic_name': f'/{name}/lidar/points',
 'ros_type_name': 'sensor_msgs/msg/PointCloud2', 'gz_type_name': 'gz.msgs.PointCloudPacked',
 'direction': 'GZ_TO_ROS', 'lazy': True},
```

Gazebo topic naming for the common sensors, given `<topic>T</topic>`:

| Sensor | Gazebo topics | ROS type |
|---|---|---|
| `gpu_lidar` | `T` / `T/points` | `LaserScan` / `PointCloud2` |
| `camera` | `T` / `T/../camera_info` (sibling) | `Image` / `CameraInfo` |
| `depth_camera` | `T` (32FC1) / `T/points` | `Image` / `PointCloud2` |
| `rgbd_camera` | `T/image`, `T/depth_image`, `T/points`, `T/camera_info` | `Image`, `Image`, `PointCloud2`, `CameraInfo` |
| `navsat` | `T` | `NavSatFix` (`gz.msgs.NavSat`) |
| `magnetometer` | `T` | `MagneticField` |
| `air_pressure` | `T` | `FluidPressure` |

**5. Publish its TF.** There is no `robot_state_publisher` (D13). Add the mount to [frames.yaml](robot_description/config/frames.yaml) (e.g. `lidar_x`, `lidar_z`) and a `static_transform_publisher` in `drone.launch.py`, exactly like the camera: parent `<name>/base_link`, child `<name>/lidar_link`. The mount values in `frames.yaml` and in the xacro must match.

Verify: `xacro robot_description/sdf/drone.sdf.xacro frame:=x500v2 > /tmp/x.sdf && gz sdf -k /tmp/x.sdf`, then `gz topic -l | grep lidar`, then `drone_domain 1; ros2 topic hz /drone1/lidar/points`, then RViz with fixed frame `drone1/odom`.

### 8.7 Recipe E — feed a ROS sensor into ArduPilot

Publishing a topic does not make ArduPilot use it. ArduPilot consumes external sensors through MAVLink, which MAVROS produces from ROS topics.

**Rangefinder (downward).** Two options:
- *Simplest, no Gazebo involved*: SITL's own rangefinder, `RNGFND1_TYPE 100`. It is computed by SITL from the height above home, so it does not see Gazebo geometry (roofs, trees). Good for altitude-hold experiments on flat worlds.
- *A Gazebo sensor that sees geometry*: a `gpu_lidar` pitched down 90° with 1 × 1 samples (§8.6), a small node converting `LaserScan` → `sensor_msgs/Range`, and MAVROS's `distance_sensor` plugin in subscriber mode. Extend the `distance_sensor` config in `mavros.launch.py`:

  ```python
  '/**/distance_sensor': {'ros__parameters': {'config': (
      'rangefinder_pub:\n'            # existing: FC → ROS
      '  id: 0\n'
      f'  frame_id: "{name}/rangefinder"\n'
      '  field_of_view: 0.0\n'
      '  send_tf: false\n'
      '  sensor_position: {x: 0.0, y: 0.0, z: -0.1}\n'
      'rangefinder_sub:\n'            # new: ROS → FC
      '  subscriber: true\n'
      '  id: 1\n'
      '  orientation: PITCH_270\n'    # pointing down
      '  covariance: 1\n')}},         # cm
  ```

  Publish `sensor_msgs/Range` to `/<name>/mavros/rangefinder_sub` and set on the FC (`extra_params` file): `RNGFND1_TYPE 10` (MAVLink), `RNGFND1_ORIENT 25` (down), `RNGFND1_MIN`, `RNGFND1_MAX` (metres in 4.7). Check it arrives: `/<name>/mavros/rangefinder_pub` (if the `rangefinder` plugin is enabled) or MAVProxy `status RANGEFINDER`.

**Proximity / obstacle avoidance.** Enable the `obstacle_distance` plugin and publish a 360° `LaserScan` to it; on the FC: `PRX1_TYPE 2` (MAVLink), `AVOID_ENABLE`. ArduPilot's own avoidance then works in LOITER / GUIDED.

**External navigation (GPS-denied).** Enable `vision_pose` (or `odometry`) and publish a pose from your VIO/SLAM; on the FC: `VISO_TYPE 1` (MAVLink), `EK3_SRC1_POSXY 6`, `EK3_SRC1_VELXY 6` / `EK3_SRC1_POSZ` as appropriate (6 = ExternalNav). In simulation, a quick test source is the ground-truth odometry with added noise. Note that `drone_commander` computes the `map` offset from the EKF *global* position; without GPS you must also set the EKF origin (`/<name>/mavros/global_position/set_gp_origin`) or adapt the offset logic.

### 8.8 Recipe F — share a topic with other drones and the GCS

Topics leave the drone only if they match the allowlist in [swarm.json5](drone_bringup/config/zenoh/swarm.json5) (D9). Two ways:

1. **No configuration change:** publish under `/<name>/swarm/<anything>` or `/swarm/<anything>` — already allowed for topics, services and actions.
2. **A dedicated rule:** add a regex to `publishers` **and** `subscribers` (and `service_servers` / `service_clients` for services), plus a rate limit:

```json5
allow: {
  publishers:  [ /* existing */ "/[^/]+/detections" ],
  subscribers: [ /* existing */ "/[^/]+/detections" ],
},
pub_max_frequencies: [ "/[^/]+/pose=10", "/[^/]+/state=10", "/[^/]+/detections=5" ],
```

Restart the bridges (GCS and drones). Check from the GCS: `gcs_domain; ros2 topic list | grep detections`.

Guidelines: share **decisions and compact data** (states, detections, targets), not raw sensor streams — the radio is a constrained link on hardware. Images: compressed and rate-limited, or not at all. Never share control loops below the position/velocity level (§9.2).

### 8.9 Recipe G — raw MAVLink and flight logs

- **Every MAVLink message, in ROS.** MAVROS republishes the raw link on `/uas1/mavlink_source` (`mavros_msgs/Mavlink`, in the drone's domain, outside the drone namespace). Decode any message without a plugin using pymavlink (installed system-wide). Useful for messages no plugin covers (e.g. `EKF_STATUS_REPORT` details, `NAMED_VALUE_FLOAT` from Lua scripts).
- **Interactive MAVLink.** `mavlink_console N` (MAVProxy on SERIAL2): `status`, `param show WP_*`, `graph`, `messages`.
- **Dataflash logs.** SITL writes `.BIN` logs (400 Hz attitude, controller internals, EKF innovations) to `~/.ros/ardupilot_sitl/<name>/logs/` while armed (`LOG_DISARMED 0`). Open them in UAV Log Viewer or MAVExplorer. This is the right tool for anything faster than a few tens of Hz.

---

## 9. Controlling the drone at other levels

### 9.1 The control ladder

The current API sits at levels 4–5. Every level below hands one more loop from ArduPilot to you; every level above hands more decision-making to software.

| Level | What you command | ROS interface | ArduPilot mode | ArduPilot still closes | You must stream at | Exists today |
|---|---|---|---|---|---|---|
| 7 Tasks / swarm | goals, formations, roles | your nodes, ROS actions under `/<name>/swarm/` | GUIDED (via level 4–5) | everything below | event-driven | `swarm_control` examples |
| 6 Mission | waypoint list stored on the FC | `mavros/mission/push` + `set_mode AUTO` | AUTO | everything | once | MAVROS only (§9.3) |
| 5 Position / geo | target position + yaw | `cmd/position`, `cmd/global` | GUIDED | position, velocity, attitude, rate | once (node re-sends) | **yes** |
| 4 Velocity | velocity + yaw rate | `cmd/velocity` | GUIDED | velocity, attitude, rate | ≥ 2 Hz (timeout 0.5 s in node) | **yes** |
| 3 Acceleration / full state | pos + vel + acc feed-forward | `mavros/setpoint_raw/local` | GUIDED | the remaining loops, with your feed-forward | 20–50 Hz | MAVROS only (§9.4) |
| 2 Attitude + thrust | quaternion + thrust or climb rate | `mavros/setpoint_raw/attitude` | GUIDED, GUIDED_NOGPS | attitude, rate | 20–50 Hz (timeout `GUID_TIMEOUT`) | MAVROS only (§9.5) |
| 1 Body rates + thrust | roll/pitch/yaw rates + thrust | `mavros/setpoint_raw/attitude` | GUIDED, GUIDED_NOGPS | rate | 50 Hz+ | MAVROS only (§9.6) |
| 1′ Pilot sticks | RC channels | `mavros/rc/override` | STABILIZE, ALT_HOLD, LOITER, ACRO… | depends on the mode | 10–50 Hz (timeout `RC_OVERRIDE_TIME`) | MAVROS only (§9.7) |
| 0 Inside the FC | anything, at 400 Hz | Lua script / C++ custom controller | any | what you leave to it | — | §9.8 |
| −1 No FC | rotor speeds | Gazebo joint topics | — (ArduPilot removed) | nothing | 200 Hz+ | §9.9 |

Lower levels give more authority and remove more safety nets. Use the highest level that can express your algorithm.

### 9.2 Ground rules for every level

1. **One writer per vehicle.** `drone_commander` re-sends its active command at 20 Hz. If you publish to `mavros/setpoint_raw/*` directly while it has an active command (`state.setpoint_type != 0`), the two fight. Either make sure the node is idle (no `cmd/*` or `hold` since the last external mode change; after `takeoff` completes it is idle), or — better — add your level to the node (§9.10) so that it inherits the safety rules.
2. **GUIDED family only for levels 1–5.** ArduPilot ignores `SET_POSITION_TARGET_*` and `SET_ATTITUDE_TARGET` outside GUIDED / GUIDED_NOGPS (and Auto-Guided).
3. **Keep the pilot override.** A mode switch from RC/QGC must always take control back. `drone_commander` does this for you; a direct publisher must stop publishing when `mavros/state.mode` changes.
4. **Loops below level 4 never cross the radio.** Attitude/rate commands need low, bounded latency: run those nodes in the drone's domain (on the companion computer). The Zenoh allowlist deliberately admits only levels 4–7.
5. **Mind the timeouts.** Velocity, acceleration and attitude targets expire after `GUID_TIMEOUT` (3 s): velocity/accel → stop; attitude → level attitude, zero climb rate. RC overrides expire after `RC_OVERRIDE_TIME` (3 s).
6. **Test at the bottom of the ladder with `profile:=realistic`** and some wind: an ideal sensor profile hides problems that will appear on hardware.

### 9.3 Going up: missions (AUTO) and tasks

**AUTO missions** let ArduPilot fly a stored waypoint list without any companion involvement — robust to radio loss, good for surveys. Upload with MAVROS (the `waypoint` plugin is enabled). ArduPilot reserves item 0 for home, so the first item is a placeholder:

```bash
drone_domain 1
ros2 service call /drone1/mavros/mission/push mavros_msgs/srv/WaypointPush "{start_index: 0, waypoints: [
  {frame: 0, command: 16, is_current: false, autocontinue: true, x_lat: -35.363262, y_long: 149.165237, z_alt: 0.0},
  {frame: 3, command: 22, autocontinue: true, z_alt: 10.0},
  {frame: 3, command: 16, autocontinue: true, x_lat: -35.36300, y_long: 149.16540, z_alt: 15.0},
  {frame: 3, command: 16, autocontinue: true, x_lat: -35.36280, y_long: 149.16580, z_alt: 15.0},
  {frame: 3, command: 20, autocontinue: true}]}"
ros2 service call /drone1/takeoff drone_interfaces/srv/Takeoff "{altitude: 10.0}"     # GUIDED takeoff
ros2 service call /drone1/set_mode drone_interfaces/srv/SetMode "{mode: AUTO}"         # mission starts
ros2 topic echo /drone1/mavros/mission/reached
```

(`command`: 16 = NAV_WAYPOINT, 20 = RETURN_TO_LAUNCH, 22 = NAV_TAKEOFF; `frame` 3 = global, altitude relative to home.) Taking off in GUIDED first avoids ArduPilot's arming-in-AUTO requirements; the NAV_TAKEOFF item then completes immediately. Switching to AUTO cancels any `drone_commander` command (D6). To expose missions in the high-level API, add a `mission` service to `drone_commander` that takes waypoints in the `map` frame, converts them with `geo.ap_en_to_latlon` and calls `mavros/mission/push`.

**Tasks** (go-to with feedback, search an area, follow a target) are best exposed as **ROS 2 actions** built on top of `cmd/position` / `cmd/velocity` and `state`: they add progress feedback and cancellation. Name them `/<name>/swarm/<task>` and they cross the radio with no configuration change (actions under `/<name>/swarm/` are allowlisted). [formation_follower.py](swarm_control/swarm_control/formation_follower.py) is the template for a decentralised behaviour running on board: it subscribes to another drone's `state` by its absolute name and publishes its own `cmd/position` with a relative name.

### 9.4 Same level, richer: position + velocity + acceleration targets

For trajectory tracking (splines, minimum-snap, MPC references), send the full reference state so ArduPilot's position controller uses your velocity and acceleration as feed-forward instead of chasing a moving point. ArduPilot 4.7 accepts these combinations in `SET_POSITION_TARGET_LOCAL_NED`: position only; position + velocity (+ acceleration); velocity (+ acceleration); acceleration only. Position + acceleration without velocity is not accepted, nor is the "force" bit.

```python
from mavros_msgs.msg import PositionTarget

def full_state_target(stamp, p, v, a, yaw):
    """p, v, a in the drone's local EKF frame (<name>/odom), ENU; yaw ENU [rad]."""
    sp = PositionTarget()
    sp.header.stamp = stamp
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED        # MAVROS converts ENU → NED
    sp.type_mask = PositionTarget.IGNORE_YAW_RATE               # use pos + vel + acc + yaw
    sp.position.x, sp.position.y, sp.position.z = p
    sp.velocity.x, sp.velocity.y, sp.velocity.z = v
    sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = a
    sp.yaw = yaw
    return sp
# publish on /<name>/mavros/setpoint_raw/local at 20–50 Hz, in GUIDED
```

Positions here are in the **local EKF frame**. If your reference is in `map`, subtract the `map → <name>/odom` translation (from TF) — or implement it inside `drone_commander` where the offset is already known (§9.10).

### 9.5 Attitude + thrust (level 2)

Message: `mavros_msgs/AttitudeTarget` on `/<name>/mavros/setpoint_raw/attitude`. Write the orientation in ROS conventions (ENU world, FLU body); MAVROS converts to NED/FRD. In FLU, a **positive pitch tilts the nose down** (rotation about the left-pointing y axis).

```python
import math
from mavros_msgs.msg import AttitudeTarget

def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2),
                              math.cos(y/2), math.sin(y/2))
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)

msg = AttitudeTarget()
msg.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE |
                 AttitudeTarget.IGNORE_YAW_RATE)               # attitude + thrust only
x, y, z, w = quat_from_rpy(0.0, math.radians(5.0), current_yaw)  # 5° nose down → accelerate forward
msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = x, y, z, w
msg.thrust = 0.5                                                # meaning depends on GUID_OPTIONS
# publish at ≥ 20 Hz with msg.header.stamp set
```

**What `thrust` means** is set by `GUID_OPTIONS` bit 3 (value 8):
- bit clear (default here): `thrust` is a **climb-rate command**: 0.5 = hold altitude, 1.0 = climb at `WP_SPD_UP`, 0.0 = descend at `WP_SPD_DN`. ArduPilot keeps altitude control — the easiest way to start.
- bit set (`GUID_OPTIONS 8`): `thrust` is **normalised collective thrust** 0..1 (hover ≈ `MOT_THST_HOVER`, ≈ 0.5 for both airframes). You now own altitude control.

The quaternion must be unit length and either all three body rates are given or none, otherwise ArduPilot rejects the target and holds position. Without new targets for `GUID_TIMEOUT` (3 s) the vehicle levels and holds altitude.

**GUIDED_NOGPS** accepts only attitude targets and needs no position estimate — the mode for indoor or GPS-denied experiments. `drone_commander` requires `position_valid` for every command, so NOGPS work needs either a direct publisher or a relaxed gate in your extension.

### 9.6 Body rates + thrust (level 1)

Same message, attitude ignored and all three rates given (FLU, rad/s; MAVROS converts to FRD):

```python
msg = AttitudeTarget()
msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
msg.body_rate.x, msg.body_rate.y, msg.body_rate.z = 0.0, 0.0, 0.3   # yaw left 0.3 rad/s
msg.thrust = 0.5
```

This is ACRO-like control from ROS: ArduPilot only runs the rate PIDs and the mixer. Stream at 50 Hz or more, from the drone's domain, and keep a fallback (stop publishing → after 3 s ArduPilot levels and holds altitude).

### 9.7 Pilot-style sticks: RC override (level 1′)

Drive ArduPilot's manual and semi-manual modes as a pilot would (the `rc_io` plugin is enabled):

```bash
drone_domain 1
# start streaming BEFORE switching mode, with throttle centred (1500 = hold altitude in ALT_HOLD / LOITER)
ros2 topic pub -r 20 /drone1/mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}" &
ros2 service call /drone1/set_mode drone_interfaces/srv/SetMode "{mode: ALT_HOLD}"
```

Channels 1–4 are roll, pitch, throttle, yaw (`RCMAP_*` defaults); 0 releases a channel, 65535 leaves it unchanged. Overrides expire after `RC_OVERRIDE_TIME` (3 s). Start the override stream before entering a stick-driven mode, otherwise the mode reads whatever the simulated RC throttle is. In STABILIZE/ACRO the throttle is raw collective (1500 is not "hover"). The `manual_control` plugin (`MANUAL_CONTROL` message, joystick axes −1000..1000) is an alternative; add it to the allowlist first.

### 9.8 Inside the flight controller (level 0)

When the MAVLink round trip is too slow or you need to modify ArduPilot's own loops, run code on the FC itself. Both options below are available in this SITL build and run identically on hardware that supports them.

**Lua scripting** — high-level logic or 50–100 Hz outer loops without a companion link. SITL loads scripts from `./scripts` relative to its working directory, i.e. `~/.ros/ardupilot_sitl/<name>/scripts/`. Enable it in a parameter file passed with `extra_params` (because `wipe:=true` resets parameters at every start, and `SCR_ENABLE` needs a reboot):

```
# my_lua.parm
SCR_ENABLE 1
```

```lua
-- ~/.ros/ardupilot_sitl/drone1/scripts/roll_doublet.lua
-- In GUIDED and armed: alternates +5 / -5 deg of roll every 2 s, keeping heading and altitude.
-- Angles are ArduPilot conventions (NED / FRD, degrees); climb rate in m/s, up positive.
local MODE_GUIDED = 4
local sign = 1
function update()
  if vehicle:get_mode() == MODE_GUIDED and arming:is_armed() then
    local yaw_deg = math.deg(ahrs:get_yaw_rad())
    vehicle:set_target_angle_and_climbrate(5 * sign, 0, yaw_deg, 0, false, 0)
    sign = -sign
  end
  return update, 2000           -- run again in 2000 ms
end
return update()
```

```bash
ros2 launch drone_bringup drone.launch.py id:=1 extra_params:=/ROS2_ardupilot/src/my_lua.parm
```

Useful bindings: `vehicle:set_target_pos_NED`, `vehicle:set_target_velaccel_NED`, `vehicle:set_target_angle_and_climbrate`, `vehicle:set_target_rate_and_throttle` (deg/s, throttle 0..1). Examples ship with ArduPilot in `/opt/ardupilot/libraries/AP_Scripting/examples/`. A script can talk to ROS through MAVLink (e.g. `gcs:send_named_float` → read it from `/uas1/mavlink_source`, §8.9).

**Custom attitude/rate controller (`AC_CustomControl`)** — replace ArduPilot's rate controller output (roll/pitch/yaw mixer inputs) with your own C++ controller, running in the 400 Hz main loop, while keeping ArduPilot's estimation, modes, navigation and safety around it. The SITL build compiles it in by default (on hardware boards: `./waf configure --enable-custom-controller`). Parameters: `CC_TYPE` (0 off, 1 empty template, 2 PID example; reboot required), `CC_AXIS_MASK` (bit 0 roll, 1 pitch, 2 yaw). It is switched in flight by an RC channel with `RCx_OPTION 109` — in simulation, drive that channel with an RC override (§9.7).

Workflow:
1. Copy `libraries/AC_CustomControl/AC_CustomControl_Empty.{h,cpp}` to a new backend, register it in `AC_CustomControl.cpp` (see the README in that folder).
2. Rebuild SITL in the container: `cd /opt/ardupilot && ./waf copter`. This change lives in the container only; to make it permanent, point the Dockerfile at your ArduPilot fork/branch (`ARDUPILOT_VERSION` is a git ref).
3. Load `CC_TYPE`, `CC_AXIS_MASK`, `RC7_OPTION 109` (for example) through `extra_params`, fly in STABILIZE/ALT_HOLD/GUIDED, toggle with the RC channel. Switching back to the main controller is bumpless.

### 9.9 Replacing ArduPilot: direct rotor control in Gazebo (level −1)

For research on the flight controller itself (geometric control, learned controllers) you may want to command rotor speeds directly. ArduPilot has no MAVLink path for in-flight motor commands (no `SET_ACTUATOR_CONTROL_TARGET` support; `MAV_CMD_DO_MOTOR_TEST` is for ground tests), so this level bypasses ArduPilot entirely.

1. Make the autopilot optional in the model: add an xacro arg (e.g. `autopilot:=ardupilot|none`) to [drone.sdf.xacro](robot_description/sdf/drone.sdf.xacro) and wrap the `<xacro:ardupilot_plugin>` block of the frame file in `<xacro:if>`.
2. When `none`, add one `JointController` per rotor:

   ```xml
   <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
     <joint_name>rotor_0_joint</joint_name>
     <initial_velocity>0</initial_velocity>
   </plugin>
   ```

   It listens on `/model/<name>/joint/rotor_0_joint/cmd_vel` (`gz.msgs.Double`, rad/s). By default it sets the joint velocity directly (an ideal motor); with `<use_force_commands>true</use_force_commands>` and a `<p_gain>` it applies torque instead (motor dynamics, like the ArduPilotPlugin does). Signs: `dir = 1` rotors spin positive (CCW seen from above), `dir = −1` negative. Hover speeds: ≈ 615 rad/s (X500), ≈ 420 rad/s (hexa).
3. Bridge `std_msgs/msg/Float64` → `gz.msgs.Double` (`ROS_TO_GZ`) for each rotor; use `/<name>/ground_truth/odometry` and the true IMU (§8.5) as feedback, or build your own estimator.
4. Add a launch option (e.g. `fc:=none`) to `drone.launch.py` that skips SITL, MAVROS and `drone_commander`.

You lose everything ArduPilot provided (EKF, failsafes, modes, MAVROS, QGC) and sim-to-real portability: this is a research path, not a deployment path. Rotor dynamics come from the same LiftDrag model, so thrust curves stay realistic.

### 9.10 Pattern: adding a new command level to `drone_commander`

Extending the node keeps every safety rule (GUIDED gate, pilot override cancellation, takeoff gating, timeouts) for the new level. Example: an attitude command `cmd/attitude`.

1. **Interface.** In [DroneState.msg](drone_interfaces/msg/DroneState.msg) add `uint8 SETPOINT_ATTITUDE = 4`. Use an existing type for the command (`mavros_msgs/AttitudeTarget`) or define one in `drone_interfaces/msg/` and add it to `rosidl_generate_interfaces` in its `CMakeLists.txt`.
2. **Subscription, publisher and parameters** in `DroneCommander.__init__`:

   ```python
   self.declare_parameter('attitude_timeout', 0.3)
   self.att_timeout = float(self.get_parameter('attitude_timeout').value)
   self.sp_attitude, self.sp_att_time = None, 0.0
   self.pub_raw_att = self.create_publisher(AttitudeTarget, 'mavros/setpoint_raw/attitude', 10)
   self.create_subscription(AttitudeTarget, 'cmd/attitude', self.on_cmd_attitude, 10,
                            callback_group=sub_group)
   ```

3. **Accept and store** (pass-through for latency, the loop handles the timeout):

   ```python
   def on_cmd_attitude(self, msg):
       if not self.accept_command('cmd/attitude'):
           return
       with self.lock:
           self.sp_attitude, self.sp_att_time = msg, self.now()
           self.sp_type = DroneState.SETPOINT_ATTITUDE
       msg.header.stamp = self.get_clock().now().to_msg()
       self.pub_raw_att.publish(msg)
   ```

4. **Timeout in `control_loop`**, next to the velocity branch:

   ```python
   if self.sp_type == DroneState.SETPOINT_ATTITUDE:
       if self.now() - self.sp_att_time > self.att_timeout:
           self.get_logger().info(f'[{self.name}] cmd/attitude timeout -> holding position')
           self.hold_here()
       return
   ```

5. **Decide the gate.** `accept_command` requires `position_valid`. For GUIDED_NOGPS work, relax it for this command type only and accept mode `GUIDED_NOGPS` as well as `GUIDED`.
6. **Consumers.** Add the new value to `SETPOINT_NAMES` in [swarm_cli.py](swarm_control/swarm_control/swarm_cli.py) and to the state printers in `drone_cli.py`.
7. **Do not allowlist `cmd/attitude` in Zenoh** (§9.2 rule 4).
8. Rebuild `drone_interfaces` first (`colcon build --packages-select drone_interfaces drone_control swarm_control`), restart the drones, and test in GUIDED at a safe altitude with small angles.

The same pattern gives `cmd/trajectory` (§9.4: compute the local target with `map_to_local` inside the node, so callers can stay in `map`).

---

## 10. Debugging toolkit

| Layer | Look at | Commands |
|---|---|---|
| Gazebo | topics, RTF, models, wind | `gz topic -l`, `gz topic -e -t /world/<w>/stats -n 1`, `gz service -s /world/<w>/wind_info --reqtype gz.msgs.Empty --reptype gz.msgs.Wind --req ''` |
| Generated config | what the launch actually produced | `ls /tmp/ardupilot_drones/` (`<name>.sdf`, `<name>_gz_bridge.yaml`, `<name>_mavros.yaml`); `~/.ros/ardupilot_sitl/<name>/instance.parm` |
| ArduPilot | pre-arm, EKF, failsafe messages | drone terminal, QGC messages, `ros2 topic echo /<name>/mavros/statustext/recv` |
| ArduPilot parameters | effective values | `ros2 param get /<name>/mavros/param WP_SPD`, `ros2 param set /<name>/mavros/param WP_SPD 3.0` |
| ArduPilot internals | MAVLink, graphs | `mavlink_console N` → `status`, `graph`, `param show` |
| ArduPilot history | 400 Hz controller and EKF data | `.BIN` logs in `~/.ros/ardupilot_sitl/<name>/logs/` → UAV Log Viewer / MAVExplorer |
| MAVROS | link and mode | `ros2 topic echo /<name>/mavros/state`, `ros2 topic hz /<name>/mavros/local_position/pose` |
| drone_commander | gate decisions | its terminal output (throttled warnings say *why* a command was ignored); `drone_cli <name> state --watch` |
| Zenoh | what crosses | `gcs_domain; ros2 topic list`; restart with `zenoh_log:=info` |
| Estimation quality | EKF vs truth | `ros2 run plotjuggler plotjuggler` in the drone domain: `/<name>/pose` vs `/<name>/ground_truth/odometry` (expect the constant z offset of §5) |
| Images | camera | `ros2 run rqt_image_view rqt_image_view /<name>/camera/image_raw` |

---

## 11. Where to change what

| I want to… | Change | Notes |
|---|---|---|
| Tune gains, speeds, accelerations | [flight_controller/config/ardupilot/](flight_controller/config/ardupilot/) `<frame>.parm`, `sitl_common.parm` | or an `extra_params` file; 4.7 names (D16) |
| Change sensor noise / GPS quality | `profile_realistic.parm` / `profile_ideal.parm` | `SIM_*` parameters |
| Change API limits and rates | [drone_commander.yaml](drone_control/config/drone_commander.yaml) | `setpoint_rate`, `velocity_timeout`, speed limits, `auto_guided` |
| Add an API command or service | [drone_commander.py](drone_control/drone_control/drone_commander.py) + [drone_interfaces](drone_interfaces/) | pattern in §9.10 |
| Enable a MAVROS feature | [apm_pluginlists.yaml](flight_controller/config/mavros/apm_pluginlists.yaml) | config in [mavros.launch.py](flight_controller/launch/mavros.launch.py) |
| Change MAVLink rates | `_STREAMS` in `drone_commander.py`, `MAV1_*` in `sitl_common.parm` | §8.3 |
| Bridge a Gazebo topic | [drone_bridge.launch.py](robot_gazebo/launch/drone_bridge.launch.py) | §8.5 |
| Add a sensor | [robot_description/sdf/](robot_description/sdf/), [frames.yaml](robot_description/config/frames.yaml), [drone.launch.py](drone_bringup/launch/drone.launch.py) (TF), drone_bridge | §8.6 |
| Share data between drones | [swarm.json5](drone_bringup/config/zenoh/swarm.json5) or the `/<name>/swarm/` prefix | §8.8 |
| New airframe | `frame_<name>.xacro`, `frames.yaml`, `<name>.parm`, `choices` in three launches | README §10.4 |
| New world / GPS location | [robot_gazebo/worlds/](robot_gazebo/worlds/) (copy the header: 1 ms physics, systems, wind, `spherical_coordinates`) | README §11 |
| Wind and gusts | `WindEffects` block of each world; `ros2 run robot_gazebo set_wind` | D17 |
| Swarm composition | [swarm.yaml](drone_bringup/config/swarm.yaml) | any `drone.launch.py` argument per drone |
| Software versions | build args of [dockerfile_nvidia_ardupilot_jazzy](dockerfile_nvidia_ardupilot_jazzy) | `ARDUPILOT_VERSION` = your FC's version |
| Real drone | [real_drone.launch.py](drone_bringup/launch/real_drone.launch.py) | README §13 |

---

## Appendix A — Non-obvious behaviours and pitfalls

Each of these cost real debugging time while building the system. Check this list first when something misbehaves.

| Symptom | Cause | Fix / rule |
|---|---|---|
| A parameter "does nothing" | Wrong name for the firmware version (4.7 SI renames); SITL silently ignores unknown names in `--defaults` | Verify with `ros2 param get /<name>/mavros/param <NAME>` |
| Position estimate is suspiciously perfect | ArduPilotPlugin `no_time_sync=1` → SITL uses `AHRS_EKF_TYPE 10` | Keep `<no_time_sync>0` and `AHRS_EKF_TYPE 3` |
| ~1 CPU core per drone burnt for nothing | 1 kHz `/clock` × one ROS node per MAVROS plugin | Keep `clock_relay` (100 Hz); never bridge the raw Gazebo clock |
| Wind has no effect | `WindEffects` waits for a first wind message; `gz topic -p` can drop it | Use `set_wind` (it verifies through `/world/<w>/wind_info`) |
| Whole world freezes, every drone stops | A SITL died while its model stayed in the world: in lock-step the plugin waits for it forever | Stop drones with Ctrl-C (removes the model) or remove it: `gz service -s /world/<w>/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --req 'name: "droneN" type: MODEL'` |
| Buildings render black | Fuel models with OGRE-1 materials under ogre2 | Use the models in `robot_gazebo/models` |
| Real-time factor ≈ 0.65 with an empty-looking world | Dynamic Fuel models (cones, barrels) | `<static>true</static>` on scenery |
| MAVROS topics at `/droneN/mavros/mavros/...` | MAVROS 2 topics are private to node `mavros` | Launch with `namespace=<name>`, not `<name>/mavros` |
| A drone cannot see another drone's topics | The bridge's `namespace` option prefixes everything | Namespace the nodes, not the bridge (D10) |
| `zenoh-bridge-ros2dds -m router` fails | The mode is a positional argument in 1.x | `zenoh-bridge-ros2dds router ...` |
| Drone 2 gets drone 1's arguments | Launch configurations leak between includes | `GroupAction(scoped=True, forwarding=False)` per include |
| Delayed actions run in the wrong ROS domain | `TimerAction` inside a scoped group fires after the group's environment was popped | Re-set `ROS_DOMAIN_ID` inside the timer's own scoped group |
| `map` positions off by ~0.5 % far from the origin | WGS84 conversion vs ArduPilot flat-earth | Use `geo.ap_latlon_to_en` / `ap_en_to_latlon` for anything compared with the EKF |
| Drone stays on the ground after `takeoff` + `goto` | A GUIDED target during takeoff cancels it | Use the node's takeoff gating; when publishing to MAVROS directly, wait for the takeoff altitude |
| Commands ignored ("mode X") | Not in GUIDED, or an external mode change cancelled the command | `takeoff` / `hold` / `set_mode GUIDED` |
| QGroundControl exits immediately | QGC refuses to run as root | `qgc` alias (runs as `ubuntu`) |
| Gazebo NavSat noise far too large | `navsat` noise stddev is in **degrees** | ≈ 3e-6 deg ≈ 0.33 m |
| An automated test kills itself | `pkill -f pattern` inside `docker exec bash -c "..."` matches its own command line | Use `pkill -f '[p]attern'`; escape `\$VAR` inside host double quotes |
