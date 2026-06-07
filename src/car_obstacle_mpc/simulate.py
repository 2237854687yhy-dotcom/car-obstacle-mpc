from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np

from car_obstacle_mpc.config import DemoConfig
from car_obstacle_mpc.mpc import MPCController, integrate_unicycle


@dataclass
class SimulationResult:
    states: np.ndarray
    controls: np.ndarray
    predicted_paths: list[np.ndarray]
    objective_values: np.ndarray
    solver_success: np.ndarray
    solve_times: np.ndarray
    solver_failures: int


def run_closed_loop(config: DemoConfig) -> SimulationResult:
    controller = MPCController(config)

    state = config.start.copy()
    states = [state.copy()]
    controls = []
    predicted_paths = []
    objective_values = []
    solver_success = []
    solve_times = []
    warm_start = None

    for _ in range(config.max_simulation_steps):
        solve_started = perf_counter()
        solution = controller.solve(state, config.goal, warm_start)
        solve_times.append(perf_counter() - solve_started)
        predicted_paths.append(solution.states.copy())
        objective_values.append(solution.objective)
        solver_success.append(solution.success)

        control = _select_control(solution.controls)
        controls.append(control.copy())

        state = integrate_unicycle(state, control, config.dt)
        states.append(state.copy())

        warm_start = controller.shift_solution(solution)

        if np.linalg.norm(state[:2] - config.goal[:2]) <= config.goal_tolerance:
            break

    return SimulationResult(
        states=np.array(states, dtype=float),
        controls=np.array(controls, dtype=float),
        predicted_paths=predicted_paths,
        objective_values=np.array(objective_values, dtype=float),
        solver_success=np.array(solver_success, dtype=bool),
        solve_times=np.array(solve_times, dtype=float),
        solver_failures=int(np.count_nonzero(~np.array(solver_success, dtype=bool))),
    )


def _select_control(control_plan: np.ndarray) -> np.ndarray:
    if control_plan.size == 0:
        return np.zeros(2, dtype=float)
    return control_plan[0]
