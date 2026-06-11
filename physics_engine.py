from __future__ import annotations

"""Simple 2D physics engine for a ball bouncing inside a rectangular field."""

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


class RectangularBounceEngine:
    def __init__(
        self,
        world_width: float,
        world_height: float,
        restitution: float = 0.82,
        wall_thickness: float = 0.04,
    ) -> None:
        """Create a rectangular bounce simulator with physical wall thickness."""
        if world_width <= 0 or world_height <= 0:
            raise ValueError("world dimensions must be > 0")
        if not 0 <= restitution <= 1:
            raise ValueError("restitution must be between 0 and 1")
        if wall_thickness < 0:
            raise ValueError("wall_thickness must be >= 0")
        if world_width <= 2 * wall_thickness or world_height <= 2 * wall_thickness:
            raise ValueError("wall_thickness leaves no inner play area")
        self.world_width = world_width
        self.world_height = world_height
        self.restitution = restitution
        self.wall_thickness = wall_thickness

    def step(self, ball: Ball, dt: float) -> Ball:
        stepped_ball, _ = self.step_with_contacts(ball, dt)
        return stepped_ball

    def step_with_contacts(self, ball: Ball, dt: float) -> Tuple[Ball, int]:
        """Advance the simulation by one fixed time step.

        Returns a new `Ball` state after movement and wall collision resolution.
        Raises `ValueError` for non-positive `dt` or invalid radius/bounds setup.
        """
        if dt <= 0:
            raise ValueError("dt must be > 0")

        min_x = self.wall_thickness + ball.radius
        max_x = self.world_width - self.wall_thickness - ball.radius
        min_y = self.wall_thickness + ball.radius
        max_y = self.world_height - self.wall_thickness - ball.radius
        if min_x > max_x or min_y > max_y:
            raise ValueError("ball radius is too large for the field")

        x = ball.x + ball.vx * dt
        y = ball.y + ball.vy * dt
        vx = ball.vx
        vy = ball.vy

        x, vx, x_contacts = self._bounce_1d(x, vx, min_x, max_x)
        y, vy, y_contacts = self._bounce_1d(y, vy, min_y, max_y)
        return Ball(x=x, y=y, vx=vx, vy=vy, radius=ball.radius), x_contacts + y_contacts

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
    ) -> Tuple[float, float, int]:
        """Reflect one axis into inclusive bounds and count wall contacts."""
        if low <= position <= high:
            return position, velocity, 0

        span = high - low
        shifted = position - low
        contacts = abs(math.floor(shifted / span))
        period = 2 * span
        wrapped = shifted % period
        if wrapped <= span:
            reflected_position = low + wrapped
        else:
            reflected_position = high - (wrapped - span)
        reflection_derivative = -1.0 if contacts % 2 else 1.0
        reflected_velocity = velocity * reflection_derivative * (self.restitution ** contacts)
        return reflected_position, reflected_velocity, contacts


class SimulationApp:
    """Tkinter app that renders an animated ball with live controls."""

    VALUE_LABEL_WIDTH = 9
    ANIMATION_FRAME_MS = 16
    GRAVITY = 0.0
    AIR_DRAG_COEFF = 0.08
    HIT_RADIUS_MULTIPLIER = 1.35
    MIN_HIT_RADIUS_PX = 10.0
    METERS_PER_PIXEL = 0.012
    MIN_WINDOW_WIDTH = 900
    MIN_WINDOW_HEIGHT = 640
    WINDOW_WIDTH_MARGIN = 80
    WINDOW_HEIGHT_MARGIN = 100
    CANVAS_BG_COLOR = "#0A6629"
    FIELD_COLOR = "#108A3A"
    BALL_FILL_COLOR = "#F57C00"
    BALL_OUTLINE_COLOR = "#FFE0B2"

    def __init__(self) -> None:
        if tk is None or ttk is None:
            raise RuntimeError("tkinter is not available in this environment")

        self.world_width = 12.0
        self.world_height = 7.0
        self.wall_thickness = 0.04
        self.margin = 24
        self.default_dt = 0.01

        self.root = tk.Tk()
        self.root.title("2D Ball Bounce Simulation")
        self.root.resizable(True, True)
        self._maximize_window()

        self.diameter_var = tk.DoubleVar(value=0.22)
        self.weight_var = tk.DoubleVar(value=1.5)
        self.speed_var = tk.DoubleVar(value=4.0)
        self.direction_var = tk.DoubleVar(value=35.0)
        self.force_x_var = tk.DoubleVar(value=0.0)
        self.force_y_var = tk.DoubleVar(value=0.0)
        self.restitution_var = tk.DoubleVar(value=0.82)
        self.dt_var = tk.DoubleVar(value=self.default_dt)
        self.max_bounces_var = tk.IntVar(value=25)
        self.wall_contacts_var = tk.StringVar(value="Wall contacts: 0 / 25")
        self.wall_contacts = 0
        self._last_valid_control_values: dict[str, float] = {}

        start_x = self.world_width / 2
        start_y = self.world_height / 2
        self.ball = Ball(x=start_x, y=start_y, vx=0.0, vy=0.0, radius=0.11)
        self.reset_point = (start_x, start_y)
        self.engine = RectangularBounceEngine(
            world_width=self.world_width,
            world_height=self.world_height,
            restitution=self.restitution_var.get(),
            wall_thickness=self.wall_thickness,
        )
        self._dragging_ball = False
        self._drag_start_offset = (0.0, 0.0)
        self._last_velocity_settings: Optional[Tuple[float, float]] = None
        self._is_running = False
        self._apply_controlled_velocity_if_changed()
        self._build_ui()
        self._resize_world_to_canvas()
        self._draw_ball()
        self._tick()

    def _maximize_window(self) -> None:
        try:
            self.root.state("zoomed")
            return
        except TK_TCL_ERROR:
            pass
        try:
            self.root.attributes("-zoomed", True)
            return
        except TK_TCL_ERROR:
            pass
        screen_w = max(self.MIN_WINDOW_WIDTH, self.root.winfo_screenwidth() - self.WINDOW_WIDTH_MARGIN)
        screen_h = max(self.MIN_WINDOW_HEIGHT, self.root.winfo_screenheight() - self.WINDOW_HEIGHT_MARGIN)
        self.root.geometry(f"{screen_w}x{screen_h}")

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        container = ttk.Frame(self.root, padding=10)
        container.grid(row=0, column=0, sticky="nsew")

        self.canvas = tk.Canvas(
            container,
            width=960,
            height=640,
            bg=self.CANVAS_BG_COLOR,
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, padx=(0, 12), sticky="nsew")
        container.columnconfigure(0, weight=4)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(0, weight=1)
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self.canvas.bind("<ButtonPress-1>", self._on_press_ball)
        self.canvas.bind("<B1-Motion>", self._on_drag_ball)
        self.canvas.bind("<ButtonRelease-1>", self._on_release_ball)

        controls_frame = ttk.LabelFrame(container, text="Menu", padding=10)
        controls_frame.grid(row=0, column=1, sticky="nsew")
        controls_frame.columnconfigure(0, weight=1)

        self._add_scale(controls_frame, "Ball diameter (m)", self.diameter_var, 0.1, 0.6, 0)
        self._add_scale(controls_frame, "Weight (kg)", self.weight_var, 0.2, 10.0, 1)
        self._add_scale(controls_frame, "Velocity speed (m/s)", self.speed_var, 0.0, 20.0, 2)
        self._add_scale(
            controls_frame, "Velocity direction (°)", self.direction_var, 0.0, 360.0, 3
        )
        self._add_scale(controls_frame, "Force X (N)", self.force_x_var, -80.0, 80.0, 4)
        self._add_scale(controls_frame, "Force Y (N)", self.force_y_var, -80.0, 80.0, 5)
        self._add_scale(controls_frame, "Restitution", self.restitution_var, 0.1, 1.0, 6)
        self._add_scale(controls_frame, "Time step (s)", self.dt_var, 0.003, 0.04, 7)
        self._add_scale(
            controls_frame,
            "Max wall contacts",
            self.max_bounces_var,
            1,
            300,
            8,
            integer_only=True,
        )

        ttk.Button(controls_frame, text="Start", command=self._start_simulation).grid(
            row=9, column=0, sticky="ew", pady=(10, 0)
        )
        ttk.Button(controls_frame, text="Pause", command=self._pause_simulation).grid(
            row=10, column=0, sticky="ew", pady=(6, 0)
        )
        ttk.Button(controls_frame, text="Reset ball", command=self._reset_ball).grid(
            row=11, column=0, sticky="ew", pady=(6, 0)
        )
        ttk.Label(
            controls_frame,
            textvariable=self.wall_contacts_var,
            anchor="center",
        ).grid(row=12, column=0, sticky="ew", pady=(10, 0))
        ttk.Button(controls_frame, text="Quit", command=self.root.destroy).grid(
            row=13, column=0, sticky="ew", pady=(6, 0)
        )

    def _add_scale(
        self,
        parent: ttk.LabelFrame,
        label: str,
        variable: tk.Variable,
        low: float,
        high: float,
        row: int,
        integer_only: bool = False,
    ) -> None:
        var_name = str(variable)
        self._last_valid_control_values[var_name] = float(variable.get())
        group = ttk.Frame(parent)
        group.grid(row=row, column=0, sticky="ew", pady=3)
        group.columnconfigure(0, weight=1)
        ttk.Label(group, text=label).grid(row=0, column=0, sticky="w")
        ttk.Scale(
            group,
            variable=variable,
            from_=low,
            to=high,
            command=lambda value: self._on_scale_change(value, variable, integer_only),
        ).grid(
            row=1, column=0, sticky="ew", padx=(0, 6)
        )
        validate_command = (
            self.root.register(
                lambda proposed, int_only=integer_only: self._validate_numeric_input(
                    proposed, int_only
                )
            ),
            "%P",
        )
        value_entry = ttk.Entry(
            group,
            textvariable=variable,
            width=self.VALUE_LABEL_WIDTH,
            validate="key",
            validatecommand=validate_command,
        )
        value_entry.grid(
            row=1, column=1, sticky="e"
        )
        value_entry.bind(
            "<FocusOut>",
            lambda _event: self._commit_control_value(variable, low, high, integer_only),
        )
        value_entry.bind(
            "<Return>",
            lambda _event: self._commit_control_value(variable, low, high, integer_only),
        )

    def _on_scale_change(self, value: str, variable: tk.Variable, integer_only: bool) -> None:
        if not integer_only:
            return
        variable.set(int(round(float(value))))

    def _validate_numeric_input(self, proposed: str, integer_only: bool) -> bool:
        if proposed in {"", "-", ".", "-."}:
            return True
        if integer_only:
            return proposed.lstrip("-").isdigit()
        try:
            float(proposed)
            return True
        except ValueError:
            return False

    def _commit_control_value(
        self,
        variable: tk.Variable,
        low: float,
        high: float,
        integer_only: bool,
    ) -> None:
        var_name = str(variable)
        try:
            value = float(variable.get())
        except (ValueError, TK_TCL_ERROR):
            value = self._last_valid_control_values.get(var_name, low)
        value = min(high, max(low, value))
        if integer_only:
            variable.set(int(round(value)))
        else:
            variable.set(value)
        self._last_valid_control_values[var_name] = float(variable.get())

    def _compute_viewport(self) -> Tuple[float, float, float, float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        usable_w = max(1.0, width - 2 * self.margin)
        usable_h = max(1.0, height - 2 * self.margin)
        scale = min(usable_w / self.world_width, usable_h / self.world_height)
        field_w_px = self.world_width * scale
        field_h_px = self.world_height * scale
        left = (width - field_w_px) / 2
        top = (height - field_h_px) / 2
        return scale, left, top, left + field_w_px, top + field_h_px

    def _world_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        scale, left, _, _, bottom = self._compute_viewport()
        cx = left + x * scale
        cy = bottom - y * scale
        return cx, cy

    def _draw_ball(self) -> None:
        self.canvas.delete("all")
        scale, left, top, right, bottom = self._compute_viewport()
        wall_px = self.wall_thickness * scale
        self.canvas.create_rectangle(left, top, right, bottom, fill="black", outline="black", width=1)
        self.canvas.create_rectangle(
            left + wall_px,
            top + wall_px,
            right - wall_px,
            bottom - wall_px,
            fill=self.FIELD_COLOR,
            outline="",
        )
        cx, cy = self._world_to_canvas(self.ball.x, self.ball.y)
        r = self.ball.radius * scale
        self.canvas.create_oval(
            cx - r,
            cy - r,
            cx + r,
            cy + r,
            fill=self.BALL_FILL_COLOR,
            outline=self.BALL_OUTLINE_COLOR,
            width=2,
            tags="ball",
        )

    def _refresh_wall_contacts_label(self) -> None:
        max_contacts = max(1, int(self.max_bounces_var.get()))
        self.wall_contacts_var.set(
            f"Wall contacts: {self.wall_contacts} / {max_contacts}"
        )

    def _reset_ball(self) -> None:
        new_radius = max(0.05, self.diameter_var.get() / 2)
        reset_x, reset_y = self._clamp_to_play_area(*self.reset_point, new_radius)
        self.ball = Ball(
            x=reset_x,
            y=reset_y,
            vx=0.0,
            vy=0.0,
            radius=new_radius,
        )
        self._last_velocity_settings = None
        self._is_running = False
        self.wall_contacts = 0
        self._refresh_wall_contacts_label()
        self._apply_controlled_velocity_if_changed()

    def _start_simulation(self) -> None:
        self._apply_controlled_velocity_if_changed()
        if self.wall_contacts >= max(1, int(self.max_bounces_var.get())):
            self.wall_contacts = 0
        self._is_running = True
        self._refresh_wall_contacts_label()

    def _pause_simulation(self) -> None:
        self._is_running = False

    def _resize_world_to_canvas(self) -> None:
        usable_width_px = max(1.0, self.canvas.winfo_width() - 2 * self.margin)
        usable_height_px = max(1.0, self.canvas.winfo_height() - 2 * self.margin)
        new_world_width = max(0.6, usable_width_px * self.METERS_PER_PIXEL)
        new_world_height = max(0.6, usable_height_px * self.METERS_PER_PIXEL)

        old_world_width = self.world_width
        old_world_height = self.world_height
        if (
            abs(new_world_width - old_world_width) < 1e-9
            and abs(new_world_height - old_world_height) < 1e-9
        ):
            return

        ball_ratio_x = self.ball.x / old_world_width
        ball_ratio_y = self.ball.y / old_world_height
        reset_ratio_x = self.reset_point[0] / old_world_width
        reset_ratio_y = self.reset_point[1] / old_world_height

        self.world_width = new_world_width
        self.world_height = new_world_height
        self.engine.world_width = new_world_width
        self.engine.world_height = new_world_height
        self.ball.x = ball_ratio_x * self.world_width
        self.ball.y = ball_ratio_y * self.world_height
        self.reset_point = (reset_ratio_x * self.world_width, reset_ratio_y * self.world_height)
        self.ball.x, self.ball.y = self._clamp_to_play_area(self.ball.x, self.ball.y, self.ball.radius)
        self.reset_point = self._clamp_to_play_area(
            self.reset_point[0], self.reset_point[1], self.ball.radius
        )

    def _clamp_to_play_area(self, x: float, y: float, radius: float) -> Tuple[float, float]:
        min_x = self.wall_thickness + radius
        max_x = self.world_width - self.wall_thickness - radius
        min_y = self.wall_thickness + radius
        max_y = self.world_height - self.wall_thickness - radius
        return min(max(x, min_x), max_x), min(max(y, min_y), max_y)

    def _canvas_to_world(self, cx: float, cy: float) -> Tuple[float, float]:
        scale, left, _, _, bottom = self._compute_viewport()
        x = (cx - left) / scale
        y = (bottom - cy) / scale
        return x, y

    def _on_canvas_resize(self, _event: tk.Event) -> None:
        self._resize_world_to_canvas()
        self._draw_ball()

    def _on_press_ball(self, event: tk.Event) -> None:
        ball_cx, ball_cy = self._world_to_canvas(self.ball.x, self.ball.y)
        scale, _, _, _, _ = self._compute_viewport()
        hit_radius = max(self.MIN_HIT_RADIUS_PX, self.ball.radius * scale * self.HIT_RADIUS_MULTIPLIER)
        if (event.x - ball_cx) ** 2 + (event.y - ball_cy) ** 2 <= hit_radius**2:
            self._dragging_ball = True
            world_x, world_y = self._canvas_to_world(event.x, event.y)
            self._drag_start_offset = (self.ball.x - world_x, self.ball.y - world_y)

    def _on_drag_ball(self, event: tk.Event) -> None:
        if not self._dragging_ball:
            return
        cursor_x, cursor_y = self._canvas_to_world(event.x, event.y)
        x = cursor_x + self._drag_start_offset[0]
        y = cursor_y + self._drag_start_offset[1]
        x, y = self._clamp_to_play_area(x, y, self.ball.radius)
        self.ball.x = x
        self.ball.y = y
        self.ball.vx = 0.0
        self.ball.vy = 0.0
        self.reset_point = (x, y)
        self._draw_ball()

    def _on_release_ball(self, _event: tk.Event) -> None:
        self._dragging_ball = False

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
        self.ball.radius = max(0.05, self.diameter_var.get() / 2)
        self.engine.restitution = min(1.0, max(0.0, self.restitution_var.get()))
        self.ball.x, self.ball.y = self._clamp_to_play_area(self.ball.x, self.ball.y, self.ball.radius)
        self.reset_point = self._clamp_to_play_area(
            self.reset_point[0], self.reset_point[1], self.ball.radius
        )

        if self._dragging_ball:
            self.ball.vx = 0.0
            self.ball.vy = 0.0
            self._draw_ball()
            self.root.after(self.ANIMATION_FRAME_MS, self._tick)
            return

        self._refresh_wall_contacts_label()
        if not self._is_running:
            self._draw_ball()
            self.root.after(self.ANIMATION_FRAME_MS, self._tick)
            return

        self._apply_controlled_velocity_if_changed()

        weight = max(0.01, self.weight_var.get())
        force_x = self.force_x_var.get()
        force_y = self.force_y_var.get()
        accel_x = force_x / weight
        accel_y = (force_y / weight) + self.GRAVITY
        self.ball.vx += accel_x * dt
        self.ball.vy += accel_y * dt
        self.ball.vx *= max(0.0, 1.0 - self.AIR_DRAG_COEFF * dt)
        self.ball.vy *= max(0.0, 1.0 - self.AIR_DRAG_COEFF * dt)

        self.ball, wall_contacts = self.engine.step_with_contacts(self.ball, dt)
        self.wall_contacts += wall_contacts
        if self.wall_contacts >= max(1, int(self.max_bounces_var.get())):
            self._is_running = False

        self._draw_ball()
        self.root.after(self.ANIMATION_FRAME_MS, self._tick)

    def run(self) -> None:
        self.root.mainloop()


if __name__ == "__main__":
    try:
        SimulationApp().run()
    except (RuntimeError, TK_TCL_ERROR):
        # Fallback for environments without Tk support or no display.
        engine = RectangularBounceEngine(world_width=12.0, world_height=7.0, restitution=0.82)
        ball = Ball(x=2.0, y=2.0, vx=4.0, vy=3.0, radius=0.11)
        trajectory = engine.simulate(ball=ball, dt=0.5, steps=12)
        for i, state in enumerate(trajectory):
            print(
                f"step={i:2d} pos=({state.x:.2f},{state.y:.2f}) "
                f"vel=({state.vx:.2f},{state.vy:.2f})"
            )
