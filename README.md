# GRAIC — Autonomous Racing Stack for CARLA

**A planning-and-control stack for a simulated autonomous race car, built against the GRAIC 2023 (Generalized Racing Intelligence Competition) framework on top of the CARLA simulator.**

![Python](https://img.shields.io/badge/Python-3.x-blue)
![CARLA](https://img.shields.io/badge/Simulator-CARLA-orange)
![ROS](https://img.shields.io/badge/ROS-not%20used-lightgrey)

Demo video: **https://youtu.be/n1QGdyXPFZI**

---

## Overview

This project implements and benchmarks a series of planning and control agents for an autonomous race car navigating closed-loop tracks in CARLA, under the GRAIC 2023 competition harness. Each agent exposes a single `run_step(filtered_obstacles, waypoints, vel, transform, boundary) -> carla.VehicleControl` entry point that the simulator calls once per control step, and is responsible for turning live perception/localization data (nearby actors, upcoming waypoints, ego velocity/pose, track boundary) into throttle/steer/brake commands in real time.

Across the `Race/` directory (~8,600 lines of Python), the stack explores multiple planning and control paradigms rather than committing to a single approach:

- **Global path planning:** Hybrid A* (via an external C++ shared library bound through `ctypes`), plain A* / grid search, Dynamic Programming, RRT (two variants), and cubic-spline-based path generation.
- **Local/reactive planning:** Dynamic Window Approach (DWA).
- **Trajectory tracking / control:** Stanley controller, Stanley + PID, LQR, MPC (via `cvxpy`), and a Proportional-Derivative (PD) controller.
- **Final integrated agent** (`agent_FINAL.py`) combining the above into the submission used for scoring.

The competition harness (`automatic_control_GRAIC.py`, adapted from the Intel/CARLA "automatic control" example) drives the simulation loop, spawns the ego vehicle, and streams state to the active agent; `wrapper.py` launches it for a selected track and `scenario.py` layers in dynamic traffic/pedestrian scenarios via CARLA's `scenario_runner`.

## Results

Scores are the GRAIC scoring-harness output per track, read directly from this repo's `Race/*_score.txt` files. **Lower is better** — this matches the convention used in the framework's own scoring, consistent with the repo's original benchmark table, which reported "percent improvement" as `(baseline − ours) / baseline`.

| Track                  | This stack's score | Naive baseline score | Result vs. baseline |
|------------------------|--------------------:|----------------------:|----------------------|
| `shanghai_intl_circuit`| 94.5                | 122                   | Better               |
| `t1_triple`            | 33.05               | 45                     | Better               |
| `t2_triple`            | 98.25               | 74                     | **Worse**            |
| `t3`                   | 64.2                | 82                     | Better               |
| `t4`                   | 44.45               | 57                     | Better               |

Notes / caveats (read before quoting these numbers):

- The "naive baseline" is a simple reference controller provided by the GRAIC framework, run on the same tracks, included here for scale — not a competing submission.
- This stack beats the baseline on 4 of 5 tracks; `t2_triple` regresses (98.25 vs. 74), likely a track-specific failure mode. See `Race/t2_triple_collision.txt` for the collisions recorded on that run.
- It is **not confirmed** whether the baseline figures (45 / 74 / 82 / 57 / 122) and this repo's current scores were captured under identical run conditions (with vs. without dynamic scenarios). The repo's original benchmark table shows the baseline itself shifting materially between "no scenario" and "with scenario" runs (e.g. shanghai baseline 122 vs. 156). Treat the comparison as directional context, not an exact percentage. `TODO: confirm scenario configuration used for the scored runs above.`
- Per-track collision logs (`Race/*_collision.txt`) are included in the repo and record collision actor ID/type/timestamp for each run.
- The repo's previous README quoted different "our score" numbers (94.2 / 40 / 67.6 / 48, no `t2_triple` entry) than the current `Race/*_score.txt` files (94.5 / 33.05 / 64.2 / 44.45, plus 98.25 for `t2_triple`). The table above trusts the score files as the source of truth.

## Controllers / Planners Implemented

| File                          | Technique                                   | Role in stack                      |
|--------------------------------|----------------------------------------------|-------------------------------------|
| `agent_astar.py`, `agent_astar2.py`, `a_star.py` | A* grid search                     | Global path planning               |
| `hybrid_astar_wrapper.py`, `hybridtest_VR.py`    | Hybrid A* (calls external `libHybridAStar.so` C++ library via `ctypes`) | Global path planning with vehicle kinematics |
| `agent_DP.py`, `dynamic_programming_heuristic.py`| Dynamic Programming                | Global path planning / heuristic cost-to-go |
| `agent3_DWA.py`, `dynamic_window_approach.py`    | Dynamic Window Approach            | Local/reactive obstacle avoidance  |
| `agentRRT.py`, `agentRRT2.py`, `rrt.py`, `rrt_star.py`, `RRTStar.py` | RRT / RRT* (two agent variants)   | Sampling-based path planning       |
| `agent_SPLINE.py`, `cubicspline.py`              | Cubic-spline path generation       | Smooth reference-path generation   |
| `agent_Stanley.py`                               | Stanley controller                 | Lateral trajectory tracking        |
| `agent_stanley_PID.py`                           | Stanley + PID                      | Combined lateral/longitudinal control |
| `agentLQR.py`                                    | LQR                                 | Optimal-control trajectory tracking |
| `agentMPC.py`                                    | MPC (via `cvxpy`)                  | Receding-horizon trajectory tracking |
| `agent.py`                                       | GRAIC-provided agent template/stub (constant throttle, no planning/control logic) | Starting skeleton, not part of the scored stack |
| `agent_FINAL.py`                                  | Integrated final agent (includes the project's PD controller: lookahead-based heading error, `kp`/`kd` gains, speed clamps) | Combines planning + control for scored runs |

Supporting modules: `car.py` (vehicle/bicycle model helpers), `angle.py` (angle utilities), `format_path.py`, `reeds_shepp_path_planning.py`, `probabilistic_road_map.py`, `py_cpp_struct.py` (ctypes structs for the Hybrid A* C++ bridge).

## Repo Structure

```
GRAIC/
├── README.md                  # this file
├── paper.tex                  # LaTeX (IEEEtran) source for the paper-style write-up
├── paper.pdf                  # compiled paper (approach, experiments, results)
├── GRAIC.mp4                  # local copy of the demo recording
├── GRAIC 2023 Installation.docx  # framework installation notes
├── waypoints/                 # per-track waypoint files (shanghai_intl_circuit, t1_triple, t2_triple, t3, t4)
└── Race/
    ├── agent_FINAL.py          # integrated agent used for scoring
    ├── agent*.py               # individual planner/controller variants (see table above)
    ├── automatic_control_GRAIC.py  # CARLA simulation harness (adapted from Intel's CARLA example)
    ├── wrapper.py              # launches the harness for a chosen track
    ├── scenario.py             # dynamic traffic/pedestrian scenario injection (scenario_runner)
    ├── hybrid_astar_wrapper.py, py_cpp_struct.py  # ctypes bridge to external Hybrid A* C++ library
    ├── *_score.txt             # per-track scoring output
    ├── *_collision.txt         # per-track collision logs
    └── *.txt / *.csv           # cached waypoints, boundaries, planned paths (planner intermediate output)
```

## Setup & Run

Framework installation notes are included in this repo as `GRAIC 2023 Installation.docx`. `TODO: transcribe the key install steps (CARLA version, GRAIC framework install) from that document into this README.`

General run flow, inferred from the code:

1. Install and run a CARLA simulator server compatible with the GRAIC 2023 framework (`TODO: confirm exact CARLA version`).
2. Install Python dependencies: `carla` (Python API), `numpy`, `scipy`, `matplotlib`, `cvxpy` (for `agentMPC.py`), `pygame` (for `automatic_control_GRAIC.py`'s HUD), and CARLA's `scenario_runner` / `srunner` package (for `scenario.py`).
   `TODO: no requirements.txt is present in this repo — pin exact versions.`
3. Select a track by editing line 6 of `Race/wrapper.py` (`map = "shanghai_intl_circuit"`, or `t1_triple` / `t2_triple` / `t3` / `t4`).
4. Point the harness at the desired agent implementation — the GRAIC submission convention is a single `agent.py` file containing an `Agent` class with `run_step(...)`; swap in any of the `agent_*.py` variants under this name to run that controller.
5. Run:
   ```bash
   cd Race
   python3 wrapper.py
   ```
   This starts `automatic_control_GRAIC.py --sync -m <map>` as a subprocess against a running CARLA server.
6. To also inject dynamic traffic/pedestrian scenarios, uncomment the `scenario_process` line in `wrapper.py` to launch `scenario.py -m <map>` (via `scenario_runner`).
7. Scoring and collision logs are written to `Race/<map>_score.txt` and `Race/<map>_collision.txt` by the framework's evaluation harness. `TODO: confirm which script/hook in the full GRAIC framework produces these files — the scoring code itself is not present in this snapshot.`

## Tech Stack

- **Language:** Python 3
- **Simulator:** CARLA (via the CARLA Python API and the GRAIC 2023 competition harness)
- **Scenario injection:** CARLA `scenario_runner` (`srunner`)
- **Numerics/optimization:** NumPy, SciPy, `cvxpy` (MPC), ctypes (bridge to an externally compiled Hybrid A* planner)
- **Visualization/debugging:** Matplotlib, Pygame (simulation HUD)
- **ROS:** not used — the harness is explicitly the "NO ROS VERSION" of the GRAIC framework (see header comment in `automatic_control_GRAIC.py`).

## Author & Collaborators

- **Vaibhav Raheja**
- Darian Irani
- Mahi Ranka

Presented as an independent autonomous-racing systems project built on the public GRAIC 2023 framework.
