# Technical Dossier — Closed-Loop LLM Task Planning for a Mobile Manipulator

> **Purpose of this file.** This is the complete, factual source-of-truth for the
> project. It is written so that a strong language model (or a co-author) can
> regenerate, extend, or improve the paper *without inventing any system detail*.
> Every number, name, version, and design decision here was extracted from the
> actual codebase, configs, and experiment logs — not from memory or assumption.
> Where something is a limitation or was only partially validated, it says so.
>
> When writing prose from this file: do **not** round success rates up, do not
> claim 100%/"5/5" anywhere, and preserve the honest "development vs. frozen
> system" distinction described in §7.

---

## 1. One-paragraph summary

We built a simulated mobile manipulator (differential-drive base + 6-DOF arm +
2-finger gripper + RGB-D camera) in ROS 2 Jazzy / Gazebo Harmonic, and drove it
with a **closed-loop LLM task planner**: a large language model (Anthropic Claude
Haiku 4.5) receives a natural-language command plus a structured scene state, and
issues actions by calling a fixed set of robot **tool services** (navigate, pick,
place, detect, etc.). After each action the planner is fed a fresh scene state,
so it can react to failure and **replan**. We compare this against a **hardcoded
baseline controller** that performs the same task with a fixed, pre-programmed
action sequence and no perception feedback. The central finding: the LLM planner
autonomously recovers from execution failures (chiefly the arm's initial
out-of-reach grasp) and succeeds on ~81% of single-object delivery runs on the
frozen system, whereas the baseline is deterministic but rigid; reliability
degrades on longer multi-object sequences, which we report as a limitation.

---

## 2. System / software stack (verified versions)

| Component | Value |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| ROS 2 | Jazzy Jalisco |
| Simulator | Gazebo Sim (gz) 8.11.0 ("Harmonic") — **not** Gazebo Classic |
| Physics engine | DART (gz default) |
| Motion planning | MoveIt 2, `MoveGroupInterface` (C++), OMPL **RRTConnect** |
| Navigation | Nav2, **MPPI** controller (`FollowPath`), static map + identity `map→odom` TF (no live SLAM during experiments) |
| Perception | Ultralytics **YOLOv8n** (`yolov8n.pt`), ultralytics 8.4.66 |
| LLM | **Anthropic Claude Haiku 4.5** (`claude-haiku-4-5-20251001`), anthropic SDK 0.109.1 |
| (Other LLM SDKs present) | openai 2.41.1, google-genai 2.9.0 (Gemini was an earlier default; **paper experiments used Anthropic**) |
| Language rule | All ROS 2 nodes in **C++** (rclcpp/ament_cmake) except exactly **two Python** nodes: the YOLO perception node and the LLM client node (rclpy) |

### 2.1 ROS 2 packages

| Package | Language | Role |
|---|---|---|
| `robotic_arm_description` | URDF/xacro/meshes | Robot model |
| `arm_moveit_config` | MoveIt 2 config | Planning groups, kinematics |
| `mobile_manipulator_core` | C++ | Tool server, state manager, **baseline controller**, **experiment runner** |
| `mm_perception` | Python (rclpy) | YOLOv8 detection node |
| `mm_llm_planner` | Python (rclpy) | LLM client / closed-loop planner |
| `mm_bringup` | launch + worlds | Launch files, `warehouse.sdf` |
| `mm_interfaces` | msg/srv/action | Custom interfaces (below) |

### 2.2 Custom interfaces (`mm_interfaces`)

- **action:** `ExecuteTask.action` — in: `command` (string); out: `success,
  summary, num_llm_calls, num_replans, duration_sec, log_path`; feedback:
  `status_message`.
- **msg:** `Detection.msg`, `DetectionArray.msg`.
- **srv:** `NavigateTo`, `DriveDistance`, `DetectObjects`, `MoveArmToNamed`,
  `MoveArmToPose`, `Pick`, `PickAtPose`, `Place`, `OpenGripper`, `CloseGripper`,
  `GetSceneState`, `CloseGripper`.
  - `PickAtPose.srv` — in: `object_label, x, y, z`; out: `success, message`.
    Used **only by the baseline** to pick at hardcoded coordinates, bypassing
    perception. The LLM planner uses `Pick` (perception-driven) instead.

---

## 3. Robot model (from URDF — do not invent joint limits)

- **Mobile base:** differential drive. Controlled via `diff_drive_controller`;
  a single `twist_stamper` node bridges `/cmd_vel` (`Twist`) to the controller's
  `TwistStamped` input.
- **Arm:** 6 revolute joints `joint0`–`joint5`, each limited **lower −3.14 rad /
  upper 3.14 rad**.
- **Gripper:** 2-finger, `joint6` and `joint7`, each limited **−0.093 / 1.57 rad**.
  Gripper velocity scaling is set to **0.5** (see bug §8.4).
- **Sensor:** RGB-D camera used for YOLO detection. (Sensors are declared at the
  **world** level in the SDF — see bug §8.6.)
- Grasping in sim uses a **DetachableJoint** plugin: a grasp is signalled by
  double-publishing an attach command with staggered sleeps (≈500 ms + ≈1000 ms)
  so the joint reliably latches under DART.

---

## 4. World (`warehouse.sdf`) — exact object poses

| Object | `<pose>` (x y z r p y) | Notes |
|---|---|---|
| `pick_table` | `2.05 0 0` | Table the cubes sit on |
| `dispatch_zone` | `-2.0 0 0.005` | Delivery destination |
| `red_cube` | `2.0 0.12 0.40` | top face ≈ z 0.42 (4 cm cube) |
| `green_cube` | `2.0 0.00 0.40` | " |
| `blue_cube` | `2.0 -0.12 0.40` | " |
| `shelf_A/B/C` | — | storage scenery |

Named navigation locations known to the robot: `table`, `dispatch_zone`,
`shelf_area`, `home`. The base navigates to a **standoff in front of the table**
(~x 1.4 m, goal tolerance 0.25 m); because that standoff leaves the cubes at the
edge of arm reach, a short forward **`drive_distance(0.30)` "approach nudge"** is
required before the pick succeeds (this is the single most important reliability
fix — see §8.2).

---

## 5. The LLM planner (`mm_llm_planner`)

### 5.1 Architecture

- Exposes a ROS 2 action `/execute_task`. Per task: fetch initial scene state,
  start the LLM conversation with a system prompt (§5.3), then loop up to
  `max_turns` (**default 30**), dispatching each requested tool call through a
  generic `_call_tool_service()` that builds the request from the tool's
  `request_fields` and polls the async future with a `tool_timeout_sec` deadline
  (**default 200 s**, set above the worst observed ~154 s `navigate_to` to
  `dispatch_zone`).
- After any action in `RESCAN_AFTER` (`navigate_to`, `drive_distance`, `pick`,
  `place`) a **fresh scene state is appended** to the conversation — this is what
  makes the loop *closed*.
- `num_replans` is incremented on every failed tool call. If the exact same
  `(tool, args, message)` failure signature repeats twice in a row, a hint is
  injected into the tool result to break loops.
- Uses a `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup` so the
  blocking service-poll loop doesn't deadlock against response callbacks.
- `task_logger.py` writes one JSON per task to `~/aset_ws/llm_task_logs/`
  containing the command, provider/model, every LLM turn (tool calls, token
  counts, latency), every tool call (args, success, message, duration,
  `is_replan`), and the full provider-native conversation for replay.

### 5.2 Tool set (11 tools exposed to the LLM)

`tool_schemas.py` is the single source of truth; converter functions emit
Anthropic / OpenAI / Gemini native tool formats from the same list.

1. `navigate_to(location_name)` — Nav2 to a fixed named location.
2. `drive_distance(distance_m)` — blind dead-reckoning straight-line move,
   +forward/−back, **capped ±2 m**, no obstacle avoidance. Used for the approach
   nudge and small corrections.
3. `detect_objects()` — run YOLO from current camera view.
4. `move_arm_to_named(pose_name ∈ {home, ready})`.
5. `move_arm_to_pose(...)` — Cartesian arm target.
6. `pick(object_label)` — perception-driven grasp (moves above, descends, closes,
   attaches). Fails with "out of reach (pre-grasp IK failed)" when the base is too
   far from the cube.
7. `place(location_name)`.
8. `open_gripper()`.
9. `close_gripper()`.
10. `get_state()` — scene state (normally auto-appended, rarely called).
11. `task_complete(success, summary)` — ends the loop; no ROS call.

### 5.3 System prompt (behavioral spec — full text in `mm_llm_planner/config/system_prompt.txt`)

Key contents the paper should cite: the robot description; the fixed warehouse
layout; the rule that the base must navigate before the arm can reach; the
canonical "pick up X and bring it to Y" workflow (navigate → confirm detection →
pick → navigate → place → task_complete); the instruction to **read error
messages and try a corrected action rather than repeating the same call**; and an
explicit instruction to call `task_complete(success=false)` promptly for
genuinely impossible requests instead of looping to the turn limit. The prompt is
a plain text file (not embedded in code) so it can be edited without recompiling.

---

## 6. The baseline controller (`mobile_manipulator_core/baseline_controller.cpp`)

**Design intent (for the comparison to be fair):** it uses the **same underlying
tool services** as the LLM, so the difference measured is *decision-making*, not
capability. Differences from the LLM:

- Fixed, pre-programmed sequence; no branching on scene state.
- Picks at a **hardcoded position** via `PickAtPose` — **never** calls
  `detect_objects`. (A traditional robot is programmed once against a measured
  workspace.) Constants: red `(2.0, 0.12, 0.42)`, green `(2.0, 0.00, 0.42)`.
- **No replanning:** any step failure aborts the run immediately —
  `num_recovery_attempts` in its log is always 0.
- Per-cube sequence: `navigate_to(table)` → `drive_distance(APPROACH_NUDGE_M =
  0.30)` → `pick_at_pose` → `navigate_to(dispatch_zone)` → `place`.
- `two_cubes:=true` ROS param runs the sequence for red then green (used for the
  two-cube baseline).
- This is also the intended foundation for a **disturbance experiment**: because
  the cube pose is a compile-time constant, moving the real cube mid-task would
  not be reflected, and the pick would fail at stale coordinates — the intended
  contrast against the LLM's perception-driven recovery. *(Not run for the
  current dataset — see §9.)*

---

## 7. Experimental methodology

- **Runner:** `ros2 run mobile_manipulator_core experiment_runner <S#> <system>
  <trials>`. Each trial resets the world (release/open gripper, arm to home,
  restore cube spawn poses via `gz service .../set_pose`, send robot home), then
  runs the command and logs result + path length to
  `experiment_logs/experiment_results.csv`. LLM per-task detail also goes to
  `llm_task_logs/*.json`; baseline to `baseline_logs/baseline_results.csv`.
- **Metrics:** success (task actually completed), wall-clock duration (s), number
  of LLM calls, number of replans (LLM), path length (m).
- **CRITICAL honesty note — "development" vs "frozen system":** the raw log
  directory spans the whole build-and-debug period. Early runs (through
  2026-06-24) fail frequently because the core reliability fixes (§8) were still
  being written — e.g. the approach nudge was still 0.15 m, `drive_back` still had
  the sign bug. **Those runs must not be reported as system performance.** The
  reportable dataset is the runs on the **stabilized configuration, from
  2026-06-25 onward.** Report the true success rate of that window (below), and
  state plainly that even on the frozen system some runs still failed. Do **not**
  present a hand-picked "5/5".

### 7.1 Scenarios

Scenario IDs follow `scenarios.yaml`, not the accepted paper's table (which
relabelled S2 as "baseline"):

| ID | Command (natural language) | What it tests |
|---|---|---|
| S1 | "take the red cube to dispatch" | Single-object pick-and-deliver + closed-loop recovery |
| S2 | "bring the red cube to dispatch" (3 cubes present) | Colour grounding — pick the right one |
| S3 | "move the red cube to dispatch, then the green cube" | Multi-step **ordered** sequential planning |
| S3′ | "move all cubes to the dispatch zone, red one first" | Three-object sequence (harder variant) |
| S4 | S1 + cube teleported ~18 cm, 5 s after departure | Mid-task disturbance |
| S5 | "tidy up the warehouse" | **Ambiguous** command interpretation (LLM-only) |

The baseline is perception-blind and runs one fixed sequence regardless of
command, so it is only meaningful on the single-object task.

*(CORRECTED 2026-09-19: S4, the mid-task disturbance scenario, **was** run —
12 trials are in `experiment_logs/experiment_results.csv` and the LLM runs
also have per-task logs. The earlier claim here that it "was never run" was
wrong. See §9.1a for what it does and does not show.)*

---

## 8. Key engineering problems and fixes (the "lessons learned" contribution)

These are real, hard-won, and belong in the paper's Discussion/Implementation
section. Each is a genuine sim-robotics interaction bug.

### 8.1 `drive_back` used unsigned distance (worst offender)
The post-pick reverse used `sqrt(dx²+dy²)`, which is always positive. With the
arm extended, its inertia could pull the base **forward**, yet the unsigned metric
counted that as "backed up," leaving the robot at/inside the table. **Fix:**
signed projection onto the backward unit vector from the starting yaw
(`traveled = dx·bx + dy·by`), so forward drift reads negative.

### 8.2 Approach nudge 0.15 → 0.30 m
The Nav2 standoff in front of the table left the cube at the edge of arm reach;
0.15 m was insufficient and pre-grasp IK failed. LLM logs confirmed 0.30 m was
needed. This single constant is the dominant reliability factor for the pick.

### 8.3 Pick IK made position-only + tighter nav goal + smaller pre-grasp
Full 6-DOF pose IK for the grasp was over-constrained; switching to a
position-only tolerance (plus a tighter nav goal and a smaller pre-grasp offset)
let the arm actually reach the cube (commit `d49978b`).

### 8.4 Gripper velocity 1.0 → 0.5
A single gripper joint oscillated at its limit under the physics engine and hung;
halving the velocity scaling fixed the hang (commit `6548d15`).

### 8.5 Post-pick sequence order (arm-home-first, slow)
Reversing with the arm extended let inertia drag the base. **Fix:** move the arm
to `home` first at reduced (50%) velocity — small reaction force — *then*
`drive_back`. Also: register the pick-table collision object in MoveIt **before**
the arm home reset, or the arm clips the table.

### 8.6 Map distortion from grasp impulse (localization)
The gripper attach impulse jolted the chassis, which injected phantom lidar
obstacles and corrupted the pose graph. **Fix:** operate on a **frozen static
map** with an identity `map→odom` TF for experiments (no live SLAM), keeping
localization stable.

### 8.7 `twist_stamper` dual-publisher bug
Two publishers to the diff-drive controller conflicted; consolidated to a single
`Twist→TwistStamped` bridge.

### 8.8 `navigate_to` timeout 120 → 300 s
First navigation to `dispatch_zone` took ~154 s; the 120 s service timeout aborted
it. Raised to 300 s (and Nav2 client placed in a reentrant callback group to avoid
deadlock).

### 8.9 `planner_node` crashed: `self._clients` collided with rclpy
`rclpy.node.Node` already uses `self._clients` internally; the planner overwrote
it with a dict, crashing on the second `create_client`. Renamed to
`self._tool_clients`. (General lesson: prefix attributes when subclassing
`rclpy.node.Node`.)

### 8.10 Gemini `Part.from_function_response(id=...)` (only if Gemini is discussed)
The installed google-genai 2.9.0 does not accept the `id=` kwarg the live docs
showed; removed it. Lesson: verify `inspect.signature()` against the *installed*
SDK, not the docs.

---

## 9. Results (FROZEN SYSTEM, runs on/after 2026-06-25)

> All numbers below are computed directly from the log files. Report them as-is.

### 9.0 Two datasets, and which one is which

- `llm_task_logs/*.json` — written by the planner, one file per task. **This
  is the source of the paper's LLM numbers.**
- `baseline_logs/baseline_results.csv` — written by `baseline_controller`.
  Source of the paper's baseline numbers.
- `experiment_logs/experiment_results.csv` — written by `experiment_runner`,
  one row per trial. Used here only to recover **which scenario** each LLM
  run belongs to (by timestamp). Its own success counts are lower than the
  paper's and must not be quoted as system performance without the
  explanation in §9.0a.

**§9.0a — why the runner CSV disagrees.** Until 2026-09-19 `experiment_runner`
carried its *own copy* of the baseline sequence with `APPROACH_NUDGE_M = 0.15`,
while `baseline_controller` used the calibrated `0.30`. So "baseline" meant two
different controllers depending on how it was launched. The runner-logged
baseline (S1 4/13, S2 1/6, S4 1/5) is that 0.15 variant, and its failures are
dominated by Nav2 timeouts. Both now share `baseline_params.hpp`.

Ten LLM runs on 07-01/02 were launched directly at the planner action, so they
have task logs but no runner row. Since only the runner applies the
disturbance, those are nominal-condition runs by construction.

### 9.1 Single-object delivery — LLM: **13/16 = 81% self-reported**

Disaggregated by scenario (frozen window, on/after 2026-06-25):

| Scenario | Success | Wilson 95% | Mean time (success) | Mean calls |
|---|---|---|---|---|
| S1 nominal | 4/5 (80%) | [38, 96]% | 430 s (243–714) | 10.2 |
| S2 grounding | 5/7 (71%) | [36, 92]% | 471 s (232–714) | 9.0 |
| S4 disturbance | 4/4 (100%) | [51, 100]% | 457 s (299–546) | 6.5 |
| **pooled** | **13/16 (81%)** | [57, 93]% | 454 s (232–714) | 8.8 |

The accepted paper reported the pooled figure as if it were S1 alone. Keep it
pooled and labelled, or report the three rows; do not present 13/16 as a
no-disturbance single-scenario result.

- 10 of the 13 successes include ≥1 autonomous recovery (not "nearly every").
- The 3 failures are planner loops, not mechanical.

**§9.1a — S4 does not show what it looks like it shows.** The `pick` tool takes
an object *label*; the tool server resolves coordinates from live perception at
call time, so the planner never holds a stale position. In 2 of the 4 S4 runs
the post-teleport pick simply succeeded with no corrective action; the other 2
ran the ordinary S1 out-of-reach nudge. **No S4 run called `detect_objects`.**
S4 evidences an *architectural* property (a perception-resolved action
interface absorbs displacement), not LLM replanning. Report descriptively.

**§9.1b — self-reported success is not verified.** `place` returns success
unconditionally, including the fallback drop; and the planner's 200 s tool
timeout is client-side, so a "failed" call may still have completed. Of the 13
successes: 9 end with `place` succeeding, 3 end with a 200 s `place` timeout
(unknown), 1 reports "No object currently held" (cannot be a delivery). **True
rate is bounded 9/16 (56%) – 13/16 (81%).** Two of the four S4 runs are in the
ambiguous class. `gz_pose.hpp` + `verify_delivery()` now check the cube's true
pose against the dispatch-zone box; applies to future runs only.

### 9.2 Single-object delivery — Baseline: **5/6 = 83%**

- `baseline_controller` on the 0.30 nudge, 2026-07-02. Durations 143–249 s
  (mean 194 s), measured by its own logger. The same task through the runner
  takes 256–507 s wall-clock — do not compare the two directly.
- The one failure was an abort at navigation start. A seventh run that day
  deliberately re-tested the 0.15 nudge and failed at the grasp; it is a
  configuration probe, not a trial.
- Two-cube baseline: 0/1.

### 9.3 Two-cube ordered delivery — LLM: **5/7 = 71%** (not 2/4)

- The accepted paper used only the four "move the red cube to dispatch, then
  the green cube" runs (2/4). Three further two-cube runs on 07-01 with more
  explicit phrasing all succeeded — **including one with the order reversed
  (green first), executed in the stated order.** That reversal is the best
  evidence in the dataset that ordering comes from the command.
- Mean 551 s (477–648), 19.7 calls. Both failures hit the turn limit.

### 9.4 Three-cube ordered delivery — LLM: **0/9**

- "move all cubes to the dispatch zone, red one first", 24.4 mean calls, never
  completed. A further 5 runner trials aborted before any LLM call (excluded).
- Typical trajectory: first cube delivered, then nav/pick failures accumulate
  until the turn budget is exhausted. **This is the scalability limitation** —
  it was omitted from the accepted paper entirely.

### 9.5 S5 — "tidy up the warehouse": attempted once, failed (case study)

- Interpretation worked (tidying → move cubes to dispatch). Execution failed:
  after the nudge a `detect_objects` cycle returned nothing, starving `pick`;
  the LLM then flailed (28 calls) and wrongly concluded the arm's reach was
  insufficient — contradicted by every other scenario.
- N=1. It is a case study, not a rate.

### 9.6 Cost (frozen runs, Haiku 4.5 list pricing)

| | single | two-cube | three-cube |
|---|---|---|---|
| Mean calls | 8.8 | 19.7 | 24.4 |
| Mean latency/call | 1.8 s | 2.0 s | 2.0 s |
| Input tokens | ~42,400 | ~138,000 | ~239,500 |
| Output tokens | ~820 | ~1,720 | ~2,010 |
| Est. cost/task | ~$0.047 | ~$0.147 | ~$0.250 |

No prompt caching — the full history is re-sent every turn, which is why cost
grows super-linearly.

## 10. Claims the paper can honestly make

1. A closed-loop LLM planner can drive a full ROS 2 mobile-manipulation stack
   (Nav2 + MoveIt 2 + YOLO) end-to-end from a single natural-language sentence.
2. The closed loop provides **genuine failure recovery**: on ~every S1 success the
   LLM autonomously recovers from an initial out-of-reach grasp by nudging forward
   and retrying — behavior a fixed script does not have.
3. On the frozen system, single-object delivery is **reported complete in 81%
   (13/16)** of runs, strictly bounded to [9/16, 13/16] because outcomes were
   self-reported; the deterministic baseline succeeds 5/6 on its calibrated
   configuration but is rigid and perception-blind.
4. Reliability **degrades with sequence length** — 71% on two cubes, **0/9 on
   three** — because the flexible planner can enter replan loops.
5. The LLM **interprets ambiguous commands** ("tidy up") into concrete subgoals,
   though live-perception robustness after local base motion currently limits
   execution (S5).
6. A catalogue of **sim-robotics integration lessons** (§8) — the grasp-impulse
   map-distortion coupling, the signed-distance reverse bug, the approach-nudge
   reachability margin — that are reusable by others building similar systems.

## 11. Claims the paper must NOT make

- No "100%" or "5/5" success claims.
- No disturbance-*recovery* claim from S4. The experiment was run, but the
  displacement is absorbed by the perception-resolved `pick` tool, not by the
  planner (§9.1a). Report S4 descriptively.
- No claim that the comparison shows LLM planning beats closed-loop
  alternatives — the baseline is non-reactive by construction. A reactive
  FSM/behavior-tree baseline is the missing control.
- Do not quote a success rate without noting it is self-reported (§9.1b).
- Do not attribute S1/S3 residual failures to hardware/reach — they are LLM
  planner loops (except the pre-2026-06-25 development failures, which were
  reachability and are excluded).
- Do not claim live SLAM during experiments — a frozen static map was used.

## 12. Threats to validity (for the paper)

- **Simulation only** (Gazebo/DART); no real-robot transfer shown.
- **Small N** (16 / 4 / 1 across scenarios) — proof-of-concept scale.
- **Single LLM** (Claude Haiku 4.5); no cross-model comparison in the dataset.
- **Frozen static map** removes real localization drift that a deployed system
  would face (and that longer runs began to expose — motivates future sensor
  fusion / relocalization).
- Baseline "success" measured on its tuned config; denominators differ between
  baseline (5) and LLM (16), so compare qualitatively (determinism vs.
  flexibility) rather than as a single head-to-head percentage.
