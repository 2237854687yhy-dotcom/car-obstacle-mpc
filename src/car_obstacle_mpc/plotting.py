from __future__ import annotations

from pathlib import Path

from matplotlib import animation
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


def save_prediction_animation(
    result: SimulationResult,
    config: DemoConfig,
    output_path: Path,
    fps: int = 6,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(8.8, 6.0), constrained_layout=True)
    _setup_path_axis(ax, config, "MPC prediction horizon changes")
    _draw_obstacles(ax, config)

    ax.scatter(config.start[0], config.start[1], s=70, color="#2ca02c", zorder=5, label="start")
    ax.scatter(config.goal[0], config.goal[1], s=90, color="#d62728", marker="*", zorder=5, label="goal")

    history_line, = ax.plot([], [], color="#1f77b4", linewidth=2.6, label="executed path")
    prediction_line, = ax.plot([], [], color="#ff7f0e", linewidth=2.0, alpha=0.88, label="current prediction")
    prediction_points = ax.scatter(
        [],
        [],
        c=[],
        cmap="viridis",
        vmin=0,
        vmax=config.horizon_steps,
        s=34,
        edgecolors="#222222",
        linewidths=0.35,
        zorder=6,
        label="prediction steps",
    )
    current_marker, = ax.plot([], [], marker="o", markersize=9, color="#111111", linestyle="", label="car")
    info_text = ax.text(
        0.02,
        0.03,
        "",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#d0d0d0", "alpha": 0.88},
    )

    cbar = fig.colorbar(prediction_points, ax=ax, fraction=0.035, pad=0.02)
    cbar.set_label("prediction step")
    ax.legend(loc="upper left")

    n_frames = len(result.predicted_paths)

    def update(frame: int):
        states_until = result.states[: frame + 1]
        prediction = result.predicted_paths[frame]
        current = states_until[-1]
        control = result.controls[frame] if frame < len(result.controls) else np.zeros(2, dtype=float)

        history_line.set_data(states_until[:, 0], states_until[:, 1])
        prediction_line.set_data(prediction[:, 0], prediction[:, 1])
        prediction_points.set_offsets(prediction[:, :2])
        prediction_points.set_array(np.arange(len(prediction)))
        current_marker.set_data([current[0]], [current[1]])
        info_text.set_text(
            f"closed-loop step: {frame + 1}/{n_frames}\n"
            f"prediction horizon: {len(prediction) - 1} steps\n"
            f"v={control[0]:.2f} m/s, omega={control[1]:.2f} rad/s"
        )

        return history_line, prediction_line, prediction_points, current_marker, info_text

    anim = animation.FuncAnimation(fig, update, frames=n_frames, interval=1000 / max(fps, 1), blit=False)
    writer = _animation_writer(output_path, fps)
    anim.save(output_path, writer=writer, dpi=140)
    plt.close(fig)


def _plot_path(ax: plt.Axes, result: SimulationResult, config: DemoConfig) -> None:
    states = result.states
    ax.plot(states[:, 0], states[:, 1], color="#1f77b4", linewidth=2.4, label="closed-loop path")
    ax.scatter(config.start[0], config.start[1], s=70, color="#2ca02c", zorder=5, label="start")
    ax.scatter(config.goal[0], config.goal[1], s=90, color="#d62728", marker="*", zorder=5, label="goal")

    for idx, prediction in enumerate(result.predicted_paths[::4]):
        label = "MPC predictions" if idx == 0 else None
        ax.plot(prediction[:, 0], prediction[:, 1], color="#7f7f7f", alpha=0.22, linewidth=0.9, label=label)

    _draw_obstacles(ax, config)
    _setup_path_axis(ax, config, "CasADi nonlinear MPC obstacle avoidance")
    ax.legend(loc="upper left")


def _draw_obstacles(ax: plt.Axes, config: DemoConfig) -> None:
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


def _setup_path_axis(ax: plt.Axes, config: DemoConfig, title: str) -> None:
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#e5e5e5")
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


def _animation_writer(output_path: Path, fps: int):
    if output_path.suffix.lower() == ".mp4":
        return animation.FFMpegWriter(fps=fps)
    return animation.PillowWriter(fps=fps)
