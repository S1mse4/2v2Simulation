# 2v2Simulation

This repository includes a small Python physics engine that simulates a ball
bouncing inside a square boundary.

## Run the simulation

```bash
python /home/runner/work/2v2Simulation/2v2Simulation/S1mse4/2v2Simulation/physics_engine.py
```

## What it does

- Defines a `Ball` state (`x`, `y`, `vx`, `vy`, `radius`)
- Simulates movement with a fixed time step
- Detects and resolves wall collisions with configurable restitution