from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class DemoConfig:
    dt: float = 0.2
    horizon_steps: int = 28
    max_simulation_steps: int = 90
    goal_tolerance: float = 0.25

    max_speed: float = 1.25
    min_speed: float = 0.0
    max_omega: float = 1.6

    robot_radius: float = 0.22
    safety_margin: float = 0.18

    start: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))
    goal: np.ndarray = field(default_factory=lambda: np.array([8.0, 4.0, 0.0], dtype=float))

    obstacles: tuple[Obstacle, ...] = (
        Obstacle(2.4, 0.95, 0.42),
        Obstacle(4.15, 2.55, 0.52),
        Obstacle(6.1, 2.55, 0.48),
    )

    state_weights: tuple[float, float, float] = (2.5, 2.5, 0.12)
    terminal_weights: tuple[float, float, float] = (18.0, 18.0, 0.5)
    control_weights: tuple[float, float] = (0.08, 0.04)
    control_rate_weights: tuple[float, float] = (0.22, 0.08)

    @property
    def obstacle_clearance(self) -> float:
        return self.robot_radius + self.safety_margin
