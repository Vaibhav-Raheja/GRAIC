# GRAIC 2023
Engineered an advanced control system for an autonomous racing car in the CARLA simulation environment.
Utilized Hybrid A* search for sophisticated path planning, enabling the car to navigate optimally through complex racetrack environments.
Implemented a Proportional-Derivative (PD) controller, dynamically adjusting steering, speed, and braking for efficient real-time navigation.
Enhanced the system by integrating obstacle avoidance capabilities into the path planning and control algorithms, further refining the car's autonomous operation.

https://www.youtube.com/watch?v=n1QGdyXPFZI



# Install
Installation Documentation could be found [Here](https://docs.google.com/document/d/1O0thKd-WcQzPpEvyfJZmjEr0xCWvgUkzzftlyZxOi_A/edit?usp=sharing)

1. All you need to submit is `agent.py`. And all you implementation should be contained in this file.
2. If you want to change to different map, just modify line 6 in `wrapper.py`. 5 maps (shanghai_intl_circuit, t1_triple, t2_triple, t3, t4) have been made available to public, while we hold some hidden maps for testing.
3. If you would like to test your controller without the scenarios, comment out line 10 in `wrapper.py`.

Benchmark score using our very naive controllers on below tracks with **no scenarios**:

| Track                   | Our Score (Hybrid A*) | Baseline Score (Yan's Score) | Percent Improvement |
|-------------------------|-----------------------|------------------------------|----------------------|
| triple_t1               | 40                    | 45                           | 11.11%               |
| t3                      | 67.6                  | 82                           | 17.56%               |
| t4                      | 48                    | 57                           | 15.79%               |
| shanghai_intl_circuit   | 94.2                  | 122                          | 22.70%               |

Benchmark score using our very naive controllers on below tracks with **with scenarios**:


| Track                   | Our Score (Hybrid A*) | Baseline Score (Yan's Score) | Percent Improvement |
|-------------------------|-----------------------|------------------------------|---------------------|
| triple_t1               | 61.6                  | 70                           | 12.0%               |
| t3                      | 90.9                  | 105                          | 13.3%               |
| t4                      | 74.5                  | 82                           | 9.1%                |
| shanghai_intl_circuit   | 92.4                  | 156                          | 40.8%               |




