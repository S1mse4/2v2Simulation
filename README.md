# 2v2Simulation

This repository includes a small Python physics engine with a Tkinter window
that animates an orange ball bouncing inside a square boundary.

## Run the simulation

```bash
python physics_engine.py
```

The app opens a simulation window with a menu of live controls for:
- ball diameter
- weight
- velocity speed
- velocity direction
- force in x/y
- restitution and time step

In headless environments (no display), it falls back to console output.

## What it does

- Defines a `Ball` state (`x`, `y`, `vx`, `vy`, `radius`)
- Simulates movement with a fixed time step
- Detects and resolves wall collisions with configurable restitution