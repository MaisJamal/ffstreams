# FFStreams++: Fast Heuristic Search for Autonomous Maneuver Planning

🚗 A fast and flexible planning framework combining configuration and trajectory streams for real-time maneuver planning.

## 🔍 Features
- Hybrid planning with fast heuristics
- Trajectory prediction using QCNet
- Integration with CommonRoad benchmark
- Apollo-compatible trajectory publishing

## 📁 Contents
- `ffstreams/`: core FFStreams++ implementation (Python)
- `ffplanner/`: FastForward heuristic planner on PDDL2.1 (C++)
- `auto_driving/`: different maneuvers PDDL2.1 domains
- `scenarios/`: benchmark scripts for CommonRoad
- `metrics/`: metrics for evaluation (OPM,TTC)
- `prediction/`: QCNet Python inference


## 📽 Demo
![Demo](./images/intersection.gif)

## 🧠 Algorithms
- Weighted A* on configuration graph
- Trajectory stream refinement in Frenet
- Prediction-aware planning with uncertainty

## 📚 Related Paper
> Jamal, M. et al. FFStreams: Fast Search With Streams for Autonomous Maneuver Planning. IEEE RA-L, 2024.

<!-- ## ⚙ Installation
```bash
cd planner/
mkdir build && cd build
cmake ..
make -->
