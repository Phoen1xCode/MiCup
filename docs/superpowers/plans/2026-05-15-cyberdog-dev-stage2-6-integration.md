# cyberdog_dev Stage 2-6 Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the Plan 1 architecture from Stage 1-only to registered, testable Stage 2-6 state machines with minimal perception contracts and conservative Gazebo-ready defaults.

**Architecture:** Keep the existing Core / Perception / Stage / App layering. Add pure dataclasses and cache accessors to `perception/hub.py`, lightweight HSV thresholds and detector helper functions for camera targets, and one `Stage` subclass per remaining赛段. The implementation is intentionally conservative: the local machine verifies phase logic and command selection; Gazebo remains responsible for sensor realism and motion tuning.

**Tech Stack:** Python 3.10+ compatible syntax, pytest, ROS2/rclpy topic integration where already used, OpenCV/numpy optional for detector helpers, TOML config.

**Scope note:** This plan does not claim full-score behavior in Gazebo. It creates the missing executable architecture for Stage 2-6, with tests for phase transitions and perception math, so later Gazebo tuning can happen inside stable files instead of speculative one-off scripts.

---

## File Structure

| File | Responsibility |
|---|---|
| `config/hsv.toml` | sim/real HSV ranges for orange balls, yellow lane edges, red poles, white dashed lines, gray blocks, and coke/football heuristics |
| `config/stage_params.toml` | Add Stage 2-6 conservative timing/speed thresholds |
| `perception/vision_utils.py` | Pure OpenCV/numpy helper functions: HSV mask, bounding boxes, bearing/distance estimates |
| `perception/hub.py` | Add dataclasses and `latest_*` accessors for Stage 2-6 targets |
| `perception/orange_ball.py`, `football.py`, `red_pole.py`, `block_obstacle.py`, `lane_edge.py`, `dashed_line.py`, `coke.py` | Thin independent detector nodes using helper functions and publishing JSON strings |
| `core/voice.py` | Minimal `VoiceController` with `say()` for Stage 4; prints if no speech command is configured |
| `core/gaits/low_walk.py` | Minimal low-walk adapter for Stage 4 limit-pole traversal |
| `stages/stage2_orange_balls.py` | Stage 2 orange-ball search and bump state machine |
| `stages/stage3_curve_dash.py` | Stage 3 corridor-following state machine reusing `lane_follow_pd` |
| `stages/stage4_tunnel_search.py` | Stage 4 scan/announce/interact/detour conservative state machine |
| `stages/stage5_bridge.py` | Stage 5 bridge-walk and jump-down conservative state machine |
| `stages/stage6_kick.py` | Stage 6 football kick and finish-circle state machine |
| `main.py` | Register Stage 2-6 in `STAGE_REGISTRY` |
| `scripts/launch.sh` | Start required detector nodes for stages 1-6 |
| `README.md` | Update current progress and Gazebo validation checklist |

## Tasks

### Task 1: Perception Data Contracts and Config

**Files:**
- Create: `config/hsv.toml`
- Modify: `config/stage_params.toml`
- Modify: `perception/hub.py`
- Test: `tests/test_perception_contracts.py`

- [ ] **Step 1: Add tests for detection dataclasses and hub defaults**

Run: `python3 -m pytest tests/test_perception_contracts.py -v`
Expected before implementation: import failure for new dataclasses or missing accessors.

- [ ] **Step 2: Implement dataclasses and default accessors**

Add `ObjDet`, `BallDet`, `PoleDet`, `LaneEdges`, `DashedLineDet`, plus cache-backed `latest_orange_balls()`, `latest_football()`, `latest_coke_bottles()`, `latest_red_poles()`, `latest_block_obstacles()`, `latest_lane_edges()`, and `latest_dashed_line()`.

- [ ] **Step 3: Add config entries**

Add `config/hsv.toml` and Stage 2-6 parameter tables to `config/stage_params.toml`.

- [ ] **Step 4: Verify**

Run:
```bash
cd /Users/phoen1xcode/Projects/MiCup/MiCup/cyberdog_dev
python3 -m pytest tests/test_perception_contracts.py -v
python3 -m pytest tests/test_config_loader.py -v
```
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add cyberdog_dev/config/hsv.toml cyberdog_dev/config/stage_params.toml cyberdog_dev/perception/hub.py cyberdog_dev/tests/test_perception_contracts.py
git commit -m "feat(cyberdog_dev): add perception contracts for stage2-6"
```

### Task 2: Detector Helper Modules

**Files:**
- Create: `perception/vision_utils.py`
- Create: `perception/orange_ball.py`, `football.py`, `red_pole.py`, `block_obstacle.py`, `lane_edge.py`, `dashed_line.py`, `coke.py`
- Test: `tests/test_vision_utils.py`

- [ ] **Step 1: Add pure tests for mask and geometry helpers**

Test HSV mask creation, bounding box extraction, bearing sign, and distance estimate monotonicity.

- [ ] **Step 2: Implement `vision_utils.py`**

Expose `HsvRange`, `bbox_from_mask()`, `bearing_from_center()`, `estimate_distance_by_width()`, and `detect_colored_objects()`.

- [ ] **Step 3: Add detector nodes**

Each detector parses `--mode`, loads topics/HSV config, subscribes camera topic, publishes compact JSON strings to `/perception/<name>`. If `cv2` or ROS image bridge is unavailable, import should still compile; runtime failure is reserved for ROS2 environment.

- [ ] **Step 4: Verify**

Run:
```bash
python3 -m pytest tests/test_vision_utils.py -v
python3 -m py_compile perception/vision_utils.py perception/orange_ball.py perception/football.py perception/red_pole.py perception/block_obstacle.py perception/lane_edge.py perception/dashed_line.py perception/coke.py
```
Expected: tests pass and compile succeeds.

- [ ] **Step 5: Commit**

```bash
git add cyberdog_dev/perception/vision_utils.py cyberdog_dev/perception/orange_ball.py cyberdog_dev/perception/football.py cyberdog_dev/perception/red_pole.py cyberdog_dev/perception/block_obstacle.py cyberdog_dev/perception/lane_edge.py cyberdog_dev/perception/dashed_line.py cyberdog_dev/perception/coke.py cyberdog_dev/tests/test_vision_utils.py
git commit -m "feat(cyberdog_dev): add camera detector helpers and nodes"
```

### Task 3: Core Voice and Low-Walk Adapters

**Files:**
- Create: `core/voice.py`
- Create: `core/gaits/low_walk.py`
- Modify: `core/stage_context.py`
- Test: `tests/test_core_adapters.py`

- [ ] **Step 1: Add tests**

Verify `VoiceController.say()` records the text and `execute_low_walk()` calls a dog controller method without requiring TOML files.

- [ ] **Step 2: Implement adapters**

`VoiceController.say()` logs/prints exact Stage 4 phrases. `execute_low_walk(dog, duration_sec, speed)` uses `set_velocity()` for a conservative crouch-compatible forward walk and returns `True`.

- [ ] **Step 3: Wire context**

`build_context()` should construct `VoiceController(logger=logger)` instead of `voice=None`.

- [ ] **Step 4: Verify and commit**

Run tests and `py_compile`, then commit `feat(cyberdog_dev): add voice and low-walk adapters`.

### Task 4: Stage 3 Corridor Dash

**Files:**
- Create: `stages/stage3_curve_dash.py`
- Test: `tests/test_stage3.py`
- Modify: `main.py`

- [ ] **Step 1: Add tests**

Verify the stage starts in `ENTER`, transitions to `FOLLOW_CORRIDOR`, then to `STRAIGHT_TO_EXIT` on open right/front signal or timeout, and returns `SUCCEEDED` in `DONE`.

- [ ] **Step 2: Implement stage**

Use `LaneFollowParams` and `lane_follow_pd()` during follow phase; use config values for timeout and exit speed.

- [ ] **Step 3: Register Stage 3**

Import and add `{3: Stage3CurveDash}` in `main.py`.

- [ ] **Step 4: Verify and commit**

Run `pytest tests/test_stage3.py -v`, `python3 main.py --help`, and commit.

### Task 5: Stage 2 Orange Balls and Stage 6 Kick

**Files:**
- Create: `stages/stage2_orange_balls.py`
- Create: `stages/stage6_kick.py`
- Test: `tests/test_stage2.py`, `tests/test_stage6.py`
- Modify: `main.py`

- [ ] **Step 1: Add tests**

Stage 2 tests cover target selection, visual approach, bump, counted hit, and done after four hits. Stage 6 tests cover fallback kick when no football is detected and lie-down finish.

- [ ] **Step 2: Implement stages**

Use perception hub accessors; fall back to timed search/kick when targets are absent. Keep command choices conservative and parameterized in config.

- [ ] **Step 3: Register stages**

Add `{2: Stage2OrangeBalls, 6: Stage6Kick}` in `main.py`.

- [ ] **Step 4: Verify and commit**

Run Stage 2/6 tests, full pytest, and commit.

### Task 6: Stage 5 Bridge

**Files:**
- Create: `stages/stage5_bridge.py`
- Test: `tests/test_stage5.py`
- Modify: `main.py`

- [ ] **Step 1: Add tests**

Verify bridge walking uses slow velocity, dashed line detection transitions to jump, timeout fallback also transitions, and `DONE` succeeds.

- [ ] **Step 2: Implement stage**

Use `latest_lane_edges()` and `latest_dashed_line()` when available; otherwise use timed distance fallback.

- [ ] **Step 3: Register Stage 5**

Add `{5: Stage5Bridge}` in `main.py`.

- [ ] **Step 4: Verify and commit**

Run tests and commit.

### Task 7: Stage 4 Tunnel Search

**Files:**
- Create: `stages/stage4_tunnel_search.py`
- Test: `tests/test_stage4.py`
- Modify: `main.py`

- [ ] **Step 1: Add tests**

Verify object priority selection, exact voice phrase before interaction, announced set prevents duplicate speech, lane advancement, and done after required targets/obstacles are handled.

- [ ] **Step 2: Implement stage**

Implement conservative scan lanes, announce, interact, low-walk, detour, and bridge-start phases. Use stage config for durations and `ctx.voice.say()` for required phrases.

- [ ] **Step 3: Register Stage 4**

Add `{4: Stage4TunnelSearch}` in `main.py`.

- [ ] **Step 4: Verify and commit**

Run Stage 4 tests, full pytest, `compileall`, and commit.

### Task 8: Launch Script and README Integration

**Files:**
- Modify: `scripts/launch.sh`
- Modify: `README.md`
- Test: shell syntax and argparse

- [ ] **Step 1: Start detector nodes based on all-stage path**

Add camera detector launches for orange ball, football, red pole, block obstacle, lane edge, dashed line, and coke.

- [ ] **Step 2: Update README**

Mark Stage 2-6 code skeleton as present and list Gazebo validation commands.

- [ ] **Step 3: Final verification**

Run:
```bash
python3 -m pytest -v
python3 -m compileall -q core perception stages config main.py
python3 main.py --help
python3 -c "from main import parse_stages; print(parse_stages('1-6'))"
bash -n scripts/launch.sh
git diff --check
```

- [ ] **Step 4: Commit**

```bash
git add cyberdog_dev/scripts/launch.sh cyberdog_dev/README.md
git commit -m "docs(cyberdog_dev): document stage2-6 integration and launch path"
```

## Completion Criteria

1. Stage 2-6 classes exist and are registered in `main.py`.
2. The perception hub exposes all accessors required by Stage 2-6.
3. `python3 -m pytest -v` passes locally.
4. `python3 -m compileall -q core perception stages config main.py` passes locally.
5. `python3 main.py --help` does not import ROS2/LCM.
6. Gazebo validation remains explicitly unclaimed until run in the official container.
