"""Simple 2D physics engine for a ball bouncing inside a square."""

from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 0.0


class SquareBounceEngine:
    def __init__(self, square_size: float, restitution: float = 1.0) -> None:
        if square_size <= 0:
            raise ValueError("square_size must be > 0")
        if not 0 <= restitution <= 1:
            raise ValueError("restitution must be between 0 and 1")
        self.square_size = square_size
        self.restitution = restitution

    def step(self, ball: Ball, dt: float) -> Ball:
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


if __name__ == "__main__":
    engine = SquareBounceEngine(square_size=10.0, restitution=1.0)
    ball = Ball(x=2.0, y=2.0, vx=4.0, vy=3.0, radius=0.2)
    trajectory = engine.simulate(ball=ball, dt=0.5, steps=12)

    for i, state in enumerate(trajectory):
        print(
            f"step={i:2d} pos=({state.x:.2f},{state.y:.2f}) "
            f"vel=({state.vx:.2f},{state.vy:.2f})"
        )
