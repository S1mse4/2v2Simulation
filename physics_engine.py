from __future__ import annotations

"""Simple 2D physics engine for a ball bouncing inside a square."""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import math
try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None
    ttk = None

if tk is not None:
    TK_TCL_ERROR = tk.TclError
else:
    class TK_TCL_ERROR(Exception):
        """Placeholder error type for environments without tkinter."""

        pass


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 0.0


class SquareBounceEngine:
    def __init__(self, square_size: float, restitution: float = 1.0) -> None:
        """Create a square-boundary bounce simulator.

        `square_size` is the side length of the square world.
        `restitution` is the collision energy retention factor (0..1).
        """
        if square_size <= 0:
            raise ValueError("square_size must be > 0")
        if not 0 <= restitution <= 1:
            raise ValueError("restitution must be between 0 and 1")
        self.square_size = square_size
        self.restitution = restitution

    def step(self, ball: Ball, dt: float) -> Ball:
        """Advance the simulation by one fixed time step.

        Returns a new `Ball` state after movement and wall collision resolution.
        Raises `ValueError` for non-positive `dt` or invalid radius/bounds setup.
        """
        if dt <= 0:
            raise ValueError("dt must be > 0")

        min_bound = ball.radius
        max_bound = self.square_size - ball.radius
        if min_bound > max_bound:
            raise ValueError("ball radius is too large for the square")

        x = ball.x + ball.vx * dt
        y = ball.y + ball.vy * dt
        vx = ball.vx
        vy = ball.vy

        x, vx = self._bounce_1d(x, vx, min_bound, max_bound)
        y, vy = self._bounce_1d(y, vy, min_bound, max_bound)
        return Ball(x=x, y=y, vx=vx, vy=vy, radius=ball.radius)

    def simulate(self, ball: Ball, dt: float, steps: int) -> List[Ball]:
        """Run repeated `step` updates and return the full trajectory.

        `steps` is the number of updates to perform, so the returned list
        contains `steps + 1` states including the initial state.
        """
        if steps < 0:
            raise ValueError("steps must be >= 0")
        states = [ball]
        current = ball
        for _ in range(steps):
            current = self.step(current, dt)
            states.append(current)
        return states

    def _bounce_1d(
        self, position: float, velocity: float, low: float, high: float
    ) -> Tuple[float, float]:
        """Reflect one axis into inclusive bounds using mirrored periodicity.

        If position is outside `[low, high]`, it is folded back into range by
        wrapping over a period of `2 * (high - low)`. The velocity is updated
        with the local reflection derivative and scaled by restitution.
        """
        if low <= position <= high:
            return position, velocity

        span = high - low
        period = 2 * span
        wrapped = (position - low) % period

        if wrapped <= span:
            reflected_position = low + wrapped
            reflection_derivative = 1.0
        else:
            reflected_position = high - (wrapped - span)
            reflection_derivative = -1.0

        reflected_velocity = (
            velocity * reflection_derivative * self.restitution
        )
        return reflected_position, reflected_velocity


class SimulationApp:
    """Tkinter app that renders an animated orange ball with live controls."""

    VALUE_LABEL_WIDTH = 9
    ANIMATION_FRAME_MS = 16

    def __init__(self) -> None:
        if tk is None or ttk is None:
            raise RuntimeError("tkinter is not available in this environment")

        self.world_size = 10.0
        self.canvas_size = 520
        self.margin = 10
        self.default_dt = 0.02

        self.root = tk.Tk()
        self.root.title("2D Ball Bounce Simulation")
        self.root.resizable(False, False)

        self.diameter_var = tk.DoubleVar(value=0.5)
        self.weight_var = tk.DoubleVar(value=1.0)
        self.speed_var = tk.DoubleVar(value=4.0)
        self.direction_var = tk.DoubleVar(value=35.0)
        self.force_x_var = tk.DoubleVar(value=0.0)
        self.force_y_var = tk.DoubleVar(value=0.0)
        self.restitution_var = tk.DoubleVar(value=0.95)
        self.dt_var = tk.DoubleVar(value=self.default_dt)

        self.ball = Ball(x=3.0, y=3.0, vx=0.0, vy=0.0, radius=0.25)
        self.engine = SquareBounceEngine(square_size=self.world_size, restitution=0.95)
        self._last_velocity_settings: Optional[Tuple[float, float]] = None
        self._apply_controlled_velocity_if_changed()
        self._build_ui()
        self._draw_ball()
        self._tick()

    def _build_ui(self) -> None:
        container = ttk.Frame(self.root, padding=10)
        container.grid(row=0, column=0, sticky="nsew")

        self.canvas = tk.Canvas(
            container,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="#111111",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, padx=(0, 12))

        controls_frame = ttk.LabelFrame(container, text="Menu", padding=10)
        controls_frame.grid(row=0, column=1, sticky="ns")

        self._add_scale(controls_frame, "Ball diameter", self.diameter_var, 0.2, 2.0, 0)
        self._add_scale(controls_frame, "Weight", self.weight_var, 0.1, 10.0, 1)
        self._add_scale(controls_frame, "Velocity speed", self.speed_var, 0.0, 20.0, 2)
        self._add_scale(
            controls_frame, "Velocity direction (°)", self.direction_var, 0.0, 360.0, 3
        )
        self._add_scale(controls_frame, "Force X", self.force_x_var, -30.0, 30.0, 4)
        self._add_scale(controls_frame, "Force Y", self.force_y_var, -30.0, 30.0, 5)
        self._add_scale(controls_frame, "Restitution", self.restitution_var, 0.1, 1.0, 6)
        self._add_scale(controls_frame, "Time step", self.dt_var, 0.005, 0.08, 7)

        ttk.Button(controls_frame, text="Reset ball", command=self._reset_ball).grid(
            row=8, column=0, sticky="ew", pady=(10, 0)
        )
        ttk.Button(controls_frame, text="Quit", command=self.root.destroy).grid(
            row=9, column=0, sticky="ew", pady=(6, 0)
        )

    def _add_scale(
        self,
        parent: ttk.LabelFrame,
        label: str,
        variable: tk.DoubleVar,
        low: float,
        high: float,
        row: int,
    ) -> None:
        group = ttk.Frame(parent)
        group.grid(row=row, column=0, sticky="ew", pady=3)
        group.columnconfigure(0, weight=1)
        ttk.Label(group, text=label).grid(row=0, column=0, sticky="w")
        ttk.Scale(group, variable=variable, from_=low, to=high).grid(
            row=1, column=0, sticky="ew", padx=(0, 6)
        )
        ttk.Label(group, textvariable=variable, width=self.VALUE_LABEL_WIDTH).grid(
            row=1, column=1, sticky="e"
        )

    def _world_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        scale = (self.canvas_size - 2 * self.margin) / self.world_size
        cx = self.margin + x * scale
        cy = self.canvas_size - (self.margin + y * scale)
        return cx, cy

    def _draw_ball(self) -> None:
        self.canvas.delete("ball")
        cx, cy = self._world_to_canvas(self.ball.x, self.ball.y)
        scale = (self.canvas_size - 2 * self.margin) / self.world_size
        r = self.ball.radius * scale
        self.canvas.create_rectangle(
            self.margin,
            self.margin,
            self.canvas_size - self.margin,
            self.canvas_size - self.margin,
            outline="#888888",
            width=2,
        )
        self.canvas.create_oval(
            cx - r,
            cy - r,
            cx + r,
            cy + r,
            fill="orange",
            outline="#FFCC80",
            width=2,
            tags="ball",
        )

    def _reset_ball(self) -> None:
        self.ball = Ball(
            x=3.0,
            y=3.0,
            vx=0.0,
            vy=0.0,
            radius=max(0.1, self.diameter_var.get() / 2),
        )
        self._last_velocity_settings = None
        self._apply_controlled_velocity_if_changed()

    def _apply_controlled_velocity_if_changed(self) -> None:
        speed = max(0.0, self.speed_var.get())
        direction = self.direction_var.get() % 360
        settings = (speed, direction)
        if settings == self._last_velocity_settings:
            return

        radians = math.radians(direction)
        self.ball.vx = speed * math.cos(radians)
        self.ball.vy = speed * math.sin(radians)
        self._last_velocity_settings = settings

    def _tick(self) -> None:
        dt = max(0.001, self.dt_var.get())
        self.ball.radius = max(0.1, self.diameter_var.get() / 2)
        self.engine.restitution = min(1.0, max(0.0, self.restitution_var.get()))

        self._apply_controlled_velocity_if_changed()

        weight = max(0.01, self.weight_var.get())
        force_x = self.force_x_var.get()
        force_y = self.force_y_var.get()
        self.ball.vx += (force_x / weight) * dt
        self.ball.vy += (force_y / weight) * dt

        self.ball = self.engine.step(self.ball, dt)
        self._draw_ball()
        self.root.after(self.ANIMATION_FRAME_MS, self._tick)

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    try:
        SimulationApp().run()
    except (RuntimeError, TK_TCL_ERROR):
        # Fallback for environments without Tk support or no display.
        engine = SquareBounceEngine(square_size=10.0, restitution=1.0)
        ball = Ball(x=2.0, y=2.0, vx=4.0, vy=3.0, radius=0.2)
        trajectory = engine.simulate(ball=ball, dt=0.5, steps=12)
        for i, state in enumerate(trajectory):
            print(
                f"step={i:2d} pos=({state.x:.2f},{state.y:.2f}) "
                f"vel=({state.vx:.2f},{state.vy:.2f})"
            )
