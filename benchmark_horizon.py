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
os.environ.setdefault("MPLBACKEND", "Agg")

from car_obstacle_mpc.benchmark import choose_best_horizon, run_horizon_benchmark, save_benchmark_outputs
from car_obstacle_mpc.config import DemoConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark MPC prediction horizon length.")
    parser.add_argument(
        "--horizons",
        default="4,6,8,10,12,16,20,24,28,32,36,40",
        help="Comma-separated horizon steps to test.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "benchmark_results",
        help="Directory where CSV, JSON, plot, and markdown report will be saved.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    horizons = _parse_horizons(args.horizons)
    rows = run_horizon_benchmark(DemoConfig(), horizons)
    best = choose_best_horizon(rows)
    outputs = save_benchmark_outputs(rows, best, args.output_dir)

    print(f"Recommended horizon_steps: {best.horizon_steps}")
    print(f"Mean solve time: {best.mean_solve_ms:.2f} ms")
    print(f"Final distance: {best.final_distance_m:.3f} m")
    print(f"Min safety margin: {best.min_clearance_margin_m:.6f} m")
    for name, path in outputs.items():
        print(f"Saved {name}: {path}")


def _parse_horizons(raw: str) -> list[int]:
    horizons = [int(item.strip()) for item in raw.split(",") if item.strip()]
    if not horizons:
        raise ValueError("At least one horizon value is required.")
    if any(horizon < 2 for horizon in horizons):
        raise ValueError("All horizon values must be >= 2.")
    return sorted(set(horizons))


if __name__ == "__main__":
    main()
