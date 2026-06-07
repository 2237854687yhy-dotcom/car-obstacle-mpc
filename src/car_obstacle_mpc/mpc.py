from __future__ import annotations

from dataclasses import dataclass

import casadi as ca
import numpy as np

from car_obstacle_mpc.config import DemoConfig


@dataclass
class MPCSolution:
    states: np.ndarray
    controls: np.ndarray
    objective: float
    success: bool


class MPCController:
    def __init__(self, config: DemoConfig):
        self.config = config
        self._build_solver()

    def _build_solver(self) -> None:
        cfg = self.config
        opti = ca.Opti()

        n_states = 3
        n_controls = 2
        n_steps = cfg.horizon_steps

        x = opti.variable(n_states, n_steps + 1)
        u = opti.variable(n_controls, n_steps)
        x0 = opti.parameter(n_states)
        x_goal = opti.parameter(n_states)

        q = ca.diag(ca.DM(cfg.state_weights))
        q_terminal = ca.diag(ca.DM(cfg.terminal_weights))
        r = ca.diag(ca.DM(cfg.control_weights))
        r_delta = ca.diag(ca.DM(cfg.control_rate_weights))

        objective = 0
        opti.subject_to(x[:, 0] == x0)

        for k in range(n_steps):
            x_k = x[:, k]
            u_k = u[:, k]
            next_state = self._dynamics(x_k, u_k)

            opti.subject_to(x[:, k + 1] == next_state)
            opti.subject_to(opti.bounded(cfg.min_speed, u_k[0], cfg.max_speed))
            opti.subject_to(opti.bounded(-cfg.max_omega, u_k[1], cfg.max_omega))

            state_error = x_k - x_goal
            objective += ca.mtimes([state_error.T, q, state_error])
            objective += ca.mtimes([u_k.T, r, u_k])

            if k > 0:
                delta_u = u[:, k] - u[:, k - 1]
                objective += ca.mtimes([delta_u.T, r_delta, delta_u])

            self._add_obstacle_constraints(opti, x_k)

        terminal_error = x[:, n_steps] - x_goal
        objective += ca.mtimes([terminal_error.T, q_terminal, terminal_error])
        self._add_obstacle_constraints(opti, x[:, n_steps])

        opti.minimize(objective)
        opti.solver(
            "ipopt",
            {"expand": True, "print_time": False},
            {
                "max_iter": 180,
                "print_level": 0,
                "sb": "yes",
                "tol": 1e-4,
                "acceptable_tol": 5e-4,
            },
        )

        self.opti = opti
        self.x = x
        self.u = u
        self.x0 = x0
        self.x_goal = x_goal
        self.objective = objective

    def _dynamics(self, state: ca.MX, control: ca.MX) -> ca.MX:
        dt = self.config.dt
        theta = state[2]
        v = control[0]
        omega = control[1]
        return state + dt * ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)

    def _add_obstacle_constraints(self, opti: ca.Opti, state: ca.MX) -> None:
        cfg = self.config
        for obstacle in cfg.obstacles:
            minimum_distance = obstacle.radius + cfg.obstacle_clearance
            squared_distance = (state[0] - obstacle.x) ** 2 + (state[1] - obstacle.y) ** 2
            opti.subject_to(squared_distance >= minimum_distance**2)

    def solve(
        self,
        current_state: np.ndarray,
        goal_state: np.ndarray,
        warm_start: tuple[np.ndarray, np.ndarray] | None = None,
    ) -> MPCSolution:
        self.opti.set_value(self.x0, current_state)
        self.opti.set_value(self.x_goal, goal_state)

        if warm_start is None:
            state_guess, control_guess = self._make_initial_guess(current_state, goal_state)
        else:
            state_guess, control_guess = warm_start

        self.opti.set_initial(self.x, state_guess)
        self.opti.set_initial(self.u, control_guess)

        try:
            solution = self.opti.solve()
            states = np.array(solution.value(self.x), dtype=float).T
            controls = np.array(solution.value(self.u), dtype=float).T
            objective = float(solution.value(self.objective))
            return MPCSolution(states=states, controls=controls, objective=objective, success=True)
        except RuntimeError:
            states = np.array(self.opti.debug.value(self.x), dtype=float).T
            controls = np.array(self.opti.debug.value(self.u), dtype=float).T
            objective = float(self.opti.debug.value(self.objective))
            return MPCSolution(states=states, controls=controls, objective=objective, success=False)

    def shift_solution(self, solution: MPCSolution) -> tuple[np.ndarray, np.ndarray]:
        states = np.vstack([solution.states[1:], solution.states[-1:]]).T
        controls = np.vstack([solution.controls[1:], solution.controls[-1:]]).T
        return states, controls

    def _make_initial_guess(self, current_state: np.ndarray, goal_state: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.config
        n_steps = cfg.horizon_steps
        points = np.zeros((n_steps + 1, 3), dtype=float)

        start_xy = current_state[:2]
        goal_xy = goal_state[:2]
        path_vector = goal_xy - start_xy
        path_length = np.linalg.norm(path_vector)

        if path_length < 1e-6:
            points[:] = goal_state
            controls = np.zeros((n_steps, 2), dtype=float)
            return points.T, controls.T

        direction = path_vector / path_length
        normal = np.array([-direction[1], direction[0]])
        detour_amplitude = 0.9

        for k in range(n_steps + 1):
            alpha = k / n_steps
            xy = start_xy + alpha * path_vector
            xy += detour_amplitude * np.sin(np.pi * alpha) * normal
            points[k, :2] = xy

        headings = np.zeros(n_steps + 1, dtype=float)
        for k in range(n_steps):
            delta = points[k + 1, :2] - points[k, :2]
            headings[k] = np.arctan2(delta[1], delta[0])
        headings[-1] = goal_state[2]
        points[:, 2] = headings
        points[0] = current_state

        controls = np.zeros((n_steps, 2), dtype=float)
        for k in range(n_steps):
            delta = points[k + 1, :2] - points[k, :2]
            controls[k, 0] = np.clip(np.linalg.norm(delta) / cfg.dt, cfg.min_speed, cfg.max_speed)
            controls[k, 1] = np.clip(wrap_angle(points[k + 1, 2] - points[k, 2]) / cfg.dt, -cfg.max_omega, cfg.max_omega)

        return points.T, controls.T


def integrate_unicycle(state: np.ndarray, control: np.ndarray, dt: float) -> np.ndarray:
    next_state = np.array(
        [
            state[0] + dt * control[0] * np.cos(state[2]),
            state[1] + dt * control[0] * np.sin(state[2]),
            state[2] + dt * control[1],
        ],
        dtype=float,
    )
    next_state[2] = wrap_angle(next_state[2])
    return next_state


def wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi
