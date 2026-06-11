# 2v2Simulation

This repository includes a small Python physics engine that simulates a ball
bouncing inside a square boundary.

## Run the simulation

```bash
python physics_engine.py
```

## What it does

- Defines a `Ball` state (`x`, `y`, `vx`, `vy`, `radius`)
- Simulates movement with a fixed time step
- Detects and resolves wall collisions with configurable restitution