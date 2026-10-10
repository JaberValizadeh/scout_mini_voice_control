# scout_mini_voice_control

Offline voice control for the AgileX Scout Mini. Recognizes spoken commands with
[Vosk](https://alphacephei.com/vosk/) (no network needed) and publishes
`geometry_msgs/Twist` on `/cmd_vel`.

This is the Scout Mini counterpart of `tb3_voice_control`; the recognizer is the
same, the teleop layer is retuned for the Mini's speed envelope and skid-steer
odometry.

## Nodes

| Node | Subscribes | Publishes | Purpose |
| --- | --- | --- | --- |
| `voice_recognizer.py` | microphone (via `arecord`) | `/voice_commands` (`std_msgs/String`) | Offline speech-to-text, restricted to a fixed grammar |
| `voice_teleop.py` | `/voice_commands`, `/odom` | `/cmd_vel` (`geometry_msgs/Twist`) | Maps phrases to velocity commands |

## Running in simulation

Two terminals, environment first. Pick the chassis with `robot:=`, and pass the
same value to both halves:

```bash
# terminal 1 - the robot and its world
source ~/scout_ws/setup_scout.bash
roslaunch scout_mini_voice_control environment.launch  robot:=scout_mini   

# or robot:=scout_v2
roslaunch scout_mini_voice_control environment.launch  robot:=scout_v2


# terminal 2 - the voice stack
source ~/scout_ws/setup_scout.bash
roslaunch scout_mini_voice_control voice_control.launch robot:=scout_mini   

# or robot:=scout_v2
roslaunch scout_mini_voice_control voice_control.launch robot:=scout_v2


`robot` defaults to `scout_mini`. Any other value is rejected outright rather
than quietly bringing up an empty world.

Wait for the environment to report its controllers before starting the voice
stack. Keeping them apart means you can restart the voice nodes freely — retune
a speed, change `gain`, swap the mic — without disturbing the simulation.

### What `robot` changes

Both chassis present the same `/cmd_vel` and `/odom`, so the command mapping is
identical. `robot` only picks the world to spawn and the speed ceilings: the V2
is the slower machine (~1.5 m/s, ~0.79 rad/s against the Mini's 3.0 and 2.5),
so its `max_angular_speed` drops to 0.78 and `faster` will not ask for rotation
the real V2 cannot deliver. Note the V2 spawns as Gazebo model `scout`, the Mini
as `scout_mini`, which matters if you query `/gazebo/get_model_state`.

`voice_control.launch` also works unchanged against real hardware; it only
needs `/cmd_vel` and `/odom`.

Or everything in one shot:

```bash
rosrun scout_mini_voice_control start_sim.sh              # start
rosrun scout_mini_voice_control start_sim.sh --restart    # restart cleanly
rosrun scout_mini_voice_control start_sim.sh --env        # environment only
```

**Never start a second environment.** `environment.launch` (and `sim.launch`,
which includes it) is the half that owns Gazebo and the controllers. Running it
twice at once is silently destructive: the
second instance registers nodes the first already owns, ROS shuts the first's
down, and the displaced `controller_spawner` unloads *every* wheel controller on
its way out. Since `controller_manager` lives inside Gazebo and is shared, the
surviving instance loses its controllers too. The symptom is baffling — speech
is still recognized and `/cmd_vel` is still published, but the robot will not
move. `start_sim.sh` checks for a running stack before starting anything.

`voice_control.launch` carries no such risk — it owns only the two voice nodes,
so restart it as often as you like.

A guard *inside* the launch file cannot prevent this: roslaunch starts nodes
concurrently, so Gazebo and the spawner are already up by the time such a guard
could abort. The check has to happen before roslaunch runs at all.

If the robot stops responding, check the stack is intact:

```bash
rosnode list                                       # expect 8 nodes
rosservice call /controller_manager/list_controllers | grep -c running   # expect 5
```

Missing `/scout_skid_steer` or fewer than 5 running controllers means the stack
was clobbered — `start_sim.sh --restart`.

To run the voice stack alone — against real hardware, or a sim you already have
up:

```bash
roslaunch scout_mini_voice_control voice_control.launch
```

## Commands

| Say | Effect |
| --- | --- |
| "move forward" / "go" | Drive forward until told otherwise |
| "go back" / "reverse" / "turn around" | Rotate 180 degrees, then drive forward |
| "turn left" / "turn right" | Rotate ~90 degrees using odometry, then stop |
| "stop" / "halt" | Stop |
| "faster" / "speed up" | Multiply speed by 3 (capped) |
| "slower" / "slow down" | Divide speed by 3 (floored) |

Recognition is restricted to the grammar in `voice_recognizer.py`. Restricting it
this way is far more reliable than free-form dictation, but it also means
anything outside the list will not be recognized — extend `GRAMMAR` to add
phrases.

## Speeds

Starting speeds are 0.3 m/s and 0.6 rad/s. The ceilings (`max_linear_speed` 1.5
m/s, `max_angular_speed` 2.0 rad/s) sit deliberately below the Mini's hardware
limits of roughly 3.0 m/s and 2.5 rad/s — voice is an imprecise input and the
Mini is quick. Raise them in `voice_control.launch` if you want the full range.

## Measured behaviour

Verified in the Gazebo Mini sim by injecting phrases on `/voice_commands` and
comparing against Gazebo ground truth (measured in **sim time** — this machine
runs Gazebo at a real-time factor of about 0.39, so wall-clock numbers
understate speeds by ~2.6x):

| Command | Expected | Measured |
| --- | --- | --- |
| `move forward` | 0.30 m/s | 0.25 m/s (window includes the ramp from rest) |
| `stop` | 0 | 0.005 m/s |
| `faster` + `move forward` | 0.90 m/s | 0.90 m/s |
| `slower` + `move forward` | 0.30 m/s | 0.30 m/s |
| `turn left` | +90 deg | +89 / +87 deg (V2 / Mini) |
| `turn right` | -90 deg | -88 / -87 deg (V2 / Mini) |
| `go back` | +180 deg then forward | +179 / +180 deg, then drives |

### Why the turns need ground-truth odometry in simulation

The turns close a loop on `/odom`, so they are only as good as `/odom` is.
Integrated *wheel* odometry is not good enough on a skid-steer chassis: turning
scrubs the wheels sideways, and encoders cannot see that scrub. The wheels reach
commanded speed at once while the body is still working against friction, so
integrated yaw runs ahead of reality and the turn stops early — measured around
**67 deg for a commanded 90**.

So in simulation `scout_gazebo`'s skid-steer node publishes the pose Gazebo
already knows, via its `ground_truth_model` parameter (set by default to
`scout_mini` / `scout` in the two world launches). That takes 90-degree turns
from ~67 deg to within a couple of degrees. Set the parameter to an empty string
to go back to wheel odometry and see the original behaviour.

On real hardware the equivalent is an IMU fused with the wheels
(`robot_localization`); `scout_base`'s own odometry is wheel-derived and carries
the same bias. Nothing in `voice_teleop.py` is simulation-specific — give it a
trustworthy `/odom` and the turns land.

## Notes

- The Vosk model (~68 MB) is not in the repo. Unpack it into `models/` once:
  ```bash
  cd models
  wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
  unzip vosk-model-small-en-us-0.15.zip && rm vosk-model-small-en-us-0.15.zip
  ```
  Override with the `model_path` arg to use a different one.
- `voice_teleop.py` needs `/odom` for its 90/180-degree turns. Without it, it
  warns once and falls back to open-loop timed turns, which are less accurate.
  In simulation `/odom` comes from `scout_gazebo`'s skid-steer node.
- If nothing is recognized, the microphone is the usual cause. Check
  `arecord -l`, then pass `device:=hw:0,0` (or whatever your card is) and raise
  `gain`.
