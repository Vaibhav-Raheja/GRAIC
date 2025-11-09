# GRAIC 2023 - Autonomous Racing System

An advanced control system for autonomous racing cars in the CARLA simulation environment, developed for the GRAIC 2023 competition. This project implements sophisticated path planning and control algorithms to enable optimal navigation through complex racetrack environments.

## Demo

Watch the system in action: [YouTube Demo](https://www.youtube.com/watch?v=n1QGdyXPFZI)

## Features

- **Advanced Path Planning**: Hybrid A* search and Dynamic Programming for optimal path generation
- **Intelligent Control**: PD controller with Pure Pursuit for precise steering, speed, and braking
- **Obstacle Avoidance**: Integrated obstacle detection and avoidance capabilities
- **Multiple Control Strategies**: Various agent implementations (LQR, MPC, Stanley, RRT, A*)
- **Production-Ready**: Comprehensive error handling, documentation, and modular architecture

## Project Structure

```
GRAIC/
├── README.md                    # This file
├── requirements.txt             # Python dependencies
├── .gitignore                   # Git ignore patterns
├── waypoints/                   # Track waypoint data
│   ├── shanghai_intl_circuit/
│   ├── t1_triple/
│   ├── t2_triple/
│   ├── t3/
│   └── t4/
└── Race/                        # Main source code
    ├── agent.py                 # Competition submission stub
    ├── agent_DP.py              # Active agent (Dynamic Programming)
    ├── agent_FINAL.py           # Production-ready agent (Pure Pursuit + PD)
    ├── automatic_control_GRAIC.py  # Main simulation controller
    ├── wrapper.py               # Launcher script
    ├── a_star.py                # A* path planning
    ├── rrt.py, rrt_star.py      # RRT path planning
    ├── car.py                   # Vehicle model
    ├── cubicspline.py           # Spline interpolation
    └── archive/                 # Experimental implementations
        ├── README.md            # Archive documentation
        ├── agent3_DWA.py        # Dynamic Window Approach
        ├── agentLQR.py          # Linear Quadratic Regulator
        ├── agentMPC.py          # Model Predictive Control
        └── ...                  # Other experimental agents
```

## Installation

### Prerequisites

- Python 3.7+
- CARLA Simulator 0.9.13+
- Git

### Installation Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/Vaibhav-Raheja/GRAIC.git
   cd GRAIC
   ```

2. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Install CARLA**

   Detailed installation instructions can be found in the [Installation Documentation](https://docs.google.com/document/d/1O0thKd-WcQzPpEvyfJZmjEr0xCWvgUkzzftlyZxOi_A/edit?usp=sharing)

## Usage

### Running the Simulation

1. Start the CARLA simulator server
2. Run the agent controller:
   ```bash
   cd Race
   python wrapper.py
   ```

### Configuration

- **Change Map**: Modify line 6 in `wrapper.py` to select a different track
  - Available maps: `shanghai_intl_circuit`, `t1_triple`, `t2_triple`, `t3`, `t4`

- **Disable Scenarios**: Comment out line 10 in `wrapper.py` to test without obstacles

- **Change Agent**: Modify the import in `automatic_control_GRAIC.py` (line 67) to use different agent implementations

### Submission

For competition submission, implement your controller in `agent.py`. All implementation should be contained in this single file.

## Agent Implementations

### Production Agents

- **agent_DP.py** (Currently Active)
  - Uses Dynamic Programming for optimal path planning
  - Divides track into grid and finds minimum-cost path
  - Handles obstacles and track boundaries

- **agent_FINAL.py** (Recommended)
  - Pure Pursuit algorithm for path following
  - PD controller for steering
  - Adaptive speed control based on turn sharpness
  - Comprehensive error handling

- **agent.py** (Submission Stub)
  - Minimal implementation template
  - Default full throttle behavior

### Experimental Agents (in archive/)

Various alternative implementations exploring different control strategies:
- Dynamic Window Approach (DWA)
- Linear Quadratic Regulator (LQR)
- Model Predictive Control (MPC)
- RRT/RRT* path planning
- Stanley controller
- A* integration

## Performance Benchmarks

### Without Scenarios

| Track                   | Our Score | Baseline | Improvement |
|-------------------------|-----------|----------|-------------|
| triple_t1               | 40        | 45       | 11.11%      |
| t3                      | 67.6      | 82       | 17.56%      |
| t4                      | 48        | 57       | 15.79%      |
| shanghai_intl_circuit   | 94.2      | 122      | 22.70%      |

### With Scenarios

| Track                   | Our Score | Baseline | Improvement |
|-------------------------|-----------|----------|-------------|
| triple_t1               | 61.6      | 70       | 12.0%       |
| t3                      | 90.9      | 105      | 13.3%       |
| t4                      | 74.5      | 82       | 9.1%        |
| shanghai_intl_circuit   | 92.4      | 156      | 40.8%       |

## Development

### Code Structure

- **Path Planning**: `a_star.py`, `rrt.py`, `dynamic_programming_heuristic.py`
- **Control**: Agent files implement the control loop
- **Simulation**: `automatic_control_GRAIC.py` manages the CARLA interface
- **Utilities**: `car.py`, `cubicspline.py`, `angle.py`

### Adding a New Agent

1. Create a new agent file in `Race/` or `Race/archive/`
2. Implement the `Agent` class with `run_step()` method
3. Update the import in `automatic_control_GRAIC.py` if you want to make it active

### Testing

Run different agents by modifying the import statement in `automatic_control_GRAIC.py`:
```python
from agent_DP import Agent      # Dynamic Programming agent
from agent_FINAL import Agent   # Pure Pursuit agent
from agent import Agent         # Submission stub
```

## Architecture

### Control Loop

1. **Perception**: Receive waypoints, obstacles, boundaries from CARLA
2. **Planning**: Calculate optimal path considering constraints
3. **Control**: Compute throttle, steering, brake commands
4. **Execution**: Send commands to CARLA vehicle

### Key Components

- **Waypoint Processing**: Parse and interpolate track waypoints
- **Collision Detection**: Check for obstacles and boundary violations
- **Path Optimization**: Find minimum-cost path through track
- **PD Control**: Proportional-Derivative steering control
- **Pure Pursuit**: Look-ahead based path following
- **Speed Management**: Adaptive throttle and braking

## Dependencies

- `carla>=0.9.13` - CARLA simulator
- `numpy>=1.20.0` - Numerical computations
- `scipy>=1.7.0` - Scientific computing
- `matplotlib>=3.3.0` - Visualization
- `pygame>=2.0.0` - Rendering
- `cvxpy>=1.1.0` - Convex optimization (for MPC)

See `requirements.txt` for complete list.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Implement your changes
4. Test thoroughly on multiple tracks
5. Submit a pull request

## License

This project is part of the GRAIC 2023 competition.

## Acknowledgments

- GRAIC 2023 Competition Organizers
- CARLA Simulator Team
- All contributors and testers

## Contact

For questions or issues, please open a GitHub issue or refer to the competition documentation.

## References

- [CARLA Documentation](https://carla.readthedocs.io/)
- [Pure Pursuit Algorithm](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- [Hybrid A* Path Planning](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
