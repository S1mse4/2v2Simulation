# 2v2Simulation

This repository includes a small Python physics engine with a Tkinter window
that animates a ball bouncing inside a rectangular field.

## Run the simulation

```bash
python physics_engine.py
```

The app opens a fullscreen-capable simulation window with a menu of live controls for:
- ball diameter (default: 22 cm)
- weight (default: 1.5 kg)
- velocity speed
- velocity direction
- force in x/y
- restitution and time step

You can drag the ball to choose its custom reset position. Pressing **Reset ball**
returns the ball to that dragged set point.

In headless environments (no display), it falls back to console output.

## What it does

- Defines a `Ball` state (`x`, `y`, `vx`, `vy`, `radius`)
- Simulates movement with gravity and air drag for more realistic motion
- Detects and resolves wall collisions inside a rectangular field
- Renders a green field with scaled 4 cm black walls