from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

CACHE_ROOT = PROJECT_ROOT / ".cache"
(CACHE_ROOT / "matplotlib").mkdir(parents=True, exist_ok=True)
os.environ.setdefault("XDG_CACHE_HOME", str(CACHE_ROOT))
os.environ.setdefault("MPLCONFIGDIR", str(CACHE_ROOT / "matplotlib"))

from car_obstacle_mpc.config import DemoConfig
from car_obstacle_mpc.simulate import run_closed_loop


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the CasADi MPC obstacle avoidance demo.")
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "car_obstacle_mpc_trajectory.png",
        help="Path where the trajectory plot will be saved.",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Save an animation showing how the MPC prediction horizon changes.",
    )
    parser.add_argument(
        "--animation-output",
        type=Path,
        default=PROJECT_ROOT / "mpc_prediction.gif",
        help="Path where the prediction animation will be saved. Use .gif by default; .mp4 needs ffmpeg.",
    )
    parser.add_argument(
        "--animation-fps",
        type=int,
        default=6,
        help="Frames per second for the saved animation.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save the plot without opening an interactive Matplotlib window.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.no_show or args.animate:
        os.environ.setdefault("MPLBACKEND", "Agg")

    from car_obstacle_mpc.plotting import plot_simulation, save_prediction_animation

    config = DemoConfig()

    result = run_closed_loop(config)
    plot_simulation(result, config, args.output, show=not args.no_show)
    if args.animate:
        save_prediction_animation(result, config, args.animation_output, fps=args.animation_fps)

    final_xy = result.states[-1, :2]
    goal_xy = config.goal[:2]
    distance_to_goal = ((final_xy - goal_xy) ** 2).sum() ** 0.5

    print(f"Saved plot: {args.output}")
    if args.animate:
        print(f"Saved animation: {args.animation_output}")
    print(f"Steps: {len(result.controls)}")
    print(f"Final state: {result.states[-1]}")
    print(f"Distance to goal: {distance_to_goal:.3f} m")
    print(f"Solver failures: {result.solver_failures}")


if __name__ == "__main__":
    main()
