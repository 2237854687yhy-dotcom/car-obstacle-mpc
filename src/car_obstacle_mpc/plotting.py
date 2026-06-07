from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from car_obstacle_mpc.config import DemoConfig
from car_obstacle_mpc.simulate import SimulationResult


def plot_simulation(result: SimulationResult, config: DemoConfig, output_path: Path, show: bool = False) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig, (ax_path, ax_control) = plt.subplots(
        1,
        2,
        figsize=(13, 5.5),
        gridspec_kw={"width_ratios": [1.45, 1.0]},
        constrained_layout=True,
    )

    _plot_path(ax_path, result, config)
    _plot_controls(ax_control, result, config)

    fig.savefig(output_path, dpi=180)
    if show:
        plt.show()
    plt.close(fig)


def _plot_path(ax: plt.Axes, result: SimulationResult, config: DemoConfig) -> None:
    states = result.states
    ax.plot(states[:, 0], states[:, 1], color="#1f77b4", linewidth=2.4, label="closed-loop path")
    ax.scatter(config.start[0], config.start[1], s=70, color="#2ca02c", zorder=5, label="start")
    ax.scatter(config.goal[0], config.goal[1], s=90, color="#d62728", marker="*", zorder=5, label="goal")

    for idx, prediction in enumerate(result.predicted_paths[::4]):
        label = "MPC predictions" if idx == 0 else None
        ax.plot(prediction[:, 0], prediction[:, 1], color="#7f7f7f", alpha=0.22, linewidth=0.9, label=label)

    for obstacle in config.obstacles:
        hard = plt.Circle((obstacle.x, obstacle.y), obstacle.radius, color="#444444", alpha=0.88)
        clearance = plt.Circle(
            (obstacle.x, obstacle.y),
            obstacle.radius + config.obstacle_clearance,
            color="#ffbf00",
            alpha=0.18,
        )
        ax.add_patch(clearance)
        ax.add_patch(hard)

    ax.set_title("CasADi nonlinear MPC obstacle avoidance")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#e5e5e5")
    ax.legend(loc="upper left")
    ax.set_xlim(-0.6, 8.7)
    ax.set_ylim(-1.0, 5.3)


def _plot_controls(ax: plt.Axes, result: SimulationResult, config: DemoConfig) -> None:
    if len(result.controls) == 0:
        ax.set_title("No controls")
        return

    time = np.arange(len(result.controls)) * config.dt
    ax.plot(time, result.controls[:, 0], label="v [m/s]", color="#1f77b4", linewidth=2.0)
    ax.plot(time, result.controls[:, 1], label="omega [rad/s]", color="#ff7f0e", linewidth=2.0)
    ax.axhline(config.max_speed, color="#1f77b4", linewidth=0.9, alpha=0.35)
    ax.axhline(-config.max_omega, color="#ff7f0e", linewidth=0.9, alpha=0.35)
    ax.axhline(config.max_omega, color="#ff7f0e", linewidth=0.9, alpha=0.35)
    ax.set_title("Executed controls")
    ax.set_xlabel("time [s]")
    ax.grid(True, color="#e5e5e5")
    ax.legend(loc="best")
