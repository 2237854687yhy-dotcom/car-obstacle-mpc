from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np

from car_obstacle_mpc.config import DemoConfig
from car_obstacle_mpc.simulate import run_closed_loop


@dataclass(frozen=True)
class HorizonBenchmarkRow:
    horizon_steps: int
    steps: int
    total_runtime_s: float
    mean_solve_ms: float
    max_solve_ms: float
    final_distance_m: float
    min_clearance_margin_m: float
    solver_failures: int
    reached_goal: bool


def run_horizon_benchmark(
    base_config: DemoConfig,
    horizons: list[int],
) -> list[HorizonBenchmarkRow]:
    rows = []
    for horizon in horizons:
        config = replace(base_config, horizon_steps=horizon)
        started = perf_counter()
        result = run_closed_loop(config)
        total_runtime = perf_counter() - started

        final_distance = float(np.linalg.norm(result.states[-1, :2] - config.goal[:2]))
        min_margin = _minimum_clearance_margin(result.states, config)
        solve_times_ms = result.solve_times * 1000.0
        rows.append(
            HorizonBenchmarkRow(
                horizon_steps=horizon,
                steps=len(result.controls),
                total_runtime_s=total_runtime,
                mean_solve_ms=float(np.mean(solve_times_ms)) if len(solve_times_ms) else 0.0,
                max_solve_ms=float(np.max(solve_times_ms)) if len(solve_times_ms) else 0.0,
                final_distance_m=final_distance,
                min_clearance_margin_m=min_margin,
                solver_failures=result.solver_failures,
                reached_goal=final_distance <= config.goal_tolerance,
            )
        )
    return rows


def choose_best_horizon(rows: list[HorizonBenchmarkRow], accuracy_band_m: float = 0.03) -> HorizonBenchmarkRow:
    feasible = [
        row
        for row in rows
        if row.reached_goal and row.solver_failures == 0 and row.min_clearance_margin_m >= -1e-5
    ]
    candidates = feasible if feasible else rows
    best_distance = min(row.final_distance_m for row in candidates)
    accurate_candidates = [
        row for row in candidates if row.final_distance_m <= best_distance + accuracy_band_m
    ]
    return min(accurate_candidates, key=lambda row: (row.mean_solve_ms, row.total_runtime_s, row.horizon_steps))


def save_benchmark_outputs(
    rows: list[HorizonBenchmarkRow],
    best: HorizonBenchmarkRow,
    output_dir: Path,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "horizon_benchmark.csv"
    json_path = output_dir / "horizon_benchmark.json"
    plot_path = output_dir / "horizon_benchmark.png"
    report_path = output_dir / "horizon_benchmark_report.md"

    _write_csv(rows, csv_path)
    _write_json(rows, best, json_path)
    _write_report(rows, best, report_path)
    _plot_benchmark(rows, best, plot_path)

    return {
        "csv": csv_path,
        "json": json_path,
        "plot": plot_path,
        "report": report_path,
    }


def _minimum_clearance_margin(states: np.ndarray, config: DemoConfig) -> float:
    if not config.obstacles:
        return float("inf")

    margins = []
    for obstacle in config.obstacles:
        minimum_distance = obstacle.radius + config.obstacle_clearance
        distances = np.sqrt((states[:, 0] - obstacle.x) ** 2 + (states[:, 1] - obstacle.y) ** 2)
        margins.append(float(np.min(distances - minimum_distance)))
    return min(margins)


def _write_csv(rows: list[HorizonBenchmarkRow], csv_path: Path) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def _write_json(rows: list[HorizonBenchmarkRow], best: HorizonBenchmarkRow, json_path: Path) -> None:
    payload = {
        "best_horizon_steps": best.horizon_steps,
        "selection_rule": (
            "Among safe runs that reached the goal with no solver failures, choose the fastest mean solve "
            "time within 0.03 m of the best final distance."
        ),
        "rows": [asdict(row) for row in rows],
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _write_report(rows: list[HorizonBenchmarkRow], best: HorizonBenchmarkRow, report_path: Path) -> None:
    lines = [
        "# MPC Prediction Horizon Benchmark",
        "",
        f"Recommended `horizon_steps`: **{best.horizon_steps}**",
        "",
        "Selection rule: among safe runs that reached the goal with no solver failures, choose the fastest",
        "average MPC solve time within `0.03 m` of the best final distance.",
        "",
        "| horizon | mean solve ms | total runtime s | final distance m | min safety margin m | steps | failures | reached |",
        "|---:|---:|---:|---:|---:|---:|---:|:---:|",
    ]
    for row in rows:
        marker = " **best**" if row.horizon_steps == best.horizon_steps else ""
        lines.append(
            f"| {row.horizon_steps}{marker} | {row.mean_solve_ms:.2f} | {row.total_runtime_s:.3f} | "
            f"{row.final_distance_m:.3f} | {row.min_clearance_margin_m:.6f} | {row.steps} | "
            f"{row.solver_failures} | {'yes' if row.reached_goal else 'no'} |"
        )
    lines.append("")
    lines.append("Interpretation:")
    lines.append("- Smaller `mean solve ms` means faster online MPC replanning.")
    lines.append("- Smaller `final distance m` means better terminal accuracy.")
    lines.append("- Non-negative `min safety margin m` means the closed-loop path stayed outside safety circles.")
    report_path.write_text("\n".join(lines), encoding="utf-8")


def _plot_benchmark(rows: list[HorizonBenchmarkRow], best: HorizonBenchmarkRow, plot_path: Path) -> None:
    horizons = [row.horizon_steps for row in rows]
    mean_solve_ms = [row.mean_solve_ms for row in rows]
    final_distance_m = [row.final_distance_m for row in rows]

    fig, ax_time = plt.subplots(figsize=(9.5, 5.4), constrained_layout=True)
    ax_distance = ax_time.twinx()

    ax_time.plot(horizons, mean_solve_ms, marker="o", color="#1f77b4", label="mean solve time")
    ax_distance.plot(horizons, final_distance_m, marker="s", color="#d62728", label="final distance")
    ax_time.axvline(best.horizon_steps, color="#2ca02c", linestyle="--", linewidth=1.5, label="recommended")

    ax_time.set_title("Prediction horizon trade-off")
    ax_time.set_xlabel("horizon steps")
    ax_time.set_ylabel("mean solve time [ms]", color="#1f77b4")
    ax_distance.set_ylabel("final distance to goal [m]", color="#d62728")
    ax_time.grid(True, color="#e5e5e5")

    lines = ax_time.get_lines() + ax_distance.get_lines()
    labels = [line.get_label() for line in lines]
    ax_time.legend(lines, labels, loc="best")

    fig.savefig(plot_path, dpi=180)
    plt.close(fig)
