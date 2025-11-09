# Archive Directory

This directory contains experimental agent implementations and test files that were developed during the GRAIC 2023 competition.

## Experimental Agent Controllers

These agent implementations explore different path planning and control algorithms:

- **agent3_DWA.py** - Dynamic Window Approach agent
- **agentLQR.py** - Linear Quadratic Regulator implementation
- **agentMPC.py** - Model Predictive Control implementation
- **agentRRT.py** / **agentRRT2.py** - RRT-based path planning agents
- **agent_SPLINE.py** - Agent using spline-based path smoothing
- **agent_Stanley.py** - Stanley controller implementation
- **agent_stanley_PID.py** - Stanley + PID hybrid controller
- **agent_astar.py** / **agent_astar2.py** - A* path planning integration agents

## Test Files

- **hybridtest_VR.py** - Hybrid A* testing and visualization script

## Note

The production agent controllers are located in the parent directory:
- `agent.py` - Submission stub
- `agent_DP.py` - Currently active agent (Dynamic Programming)
- `agent_FINAL.py` - Most complete implementation with PD controller and pure pursuit

These archived implementations were kept for reference and may be useful for future development or research.
