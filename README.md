# CasADi MPC 小车避障 Demo

这是一个小型非线性 MPC 项目：用 CasADi + IPOPT 控制一个二维小车从起点到目标点，同时避开圆形障碍物。

模型采用 unicycle/差速小车：

```text
x[k+1]     = x[k] + dt * v[k] * cos(theta[k])
y[k+1]     = y[k] + dt * v[k] * sin(theta[k])
theta[k+1] = theta[k] + dt * omega[k]
```

MPC 每一步求解一个有限时域最优控制问题：

- 追踪目标点 `(x_goal, y_goal, theta_goal)`
- 限制速度 `v` 和角速度 `omega`
- 对每个圆形障碍物加入距离约束
- 滚动执行第一个控制量，然后重新规划

## 安装

推荐创建项目内虚拟环境：

```bash
cd /Users/a1/Documents/Codex/2026-06-07/casadi-mpc/work/car_obstacle_mpc
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

如果你使用 `uv`：

```bash
cd /Users/a1/Documents/Codex/2026-06-07/casadi-mpc/work/car_obstacle_mpc
uv venv
uv pip install -r requirements.txt
```

## 运行

```bash
python run_demo.py
```

默认会把结果图保存到：

```text
/Users/a1/Documents/Codex/2026-06-07/casadi-mpc/outputs/car_obstacle_mpc_trajectory.png
```

也可以自定义输出路径：

```bash
python run_demo.py --output /tmp/mpc_result.png
```

## 可以改的参数

主要参数在 `src/car_obstacle_mpc/config.py`：

- `dt`：离散时间步长
- `horizon_steps`：MPC 预测步数
- `max_speed` / `max_omega`：控制约束
- `robot_radius` / `safety_margin`：避障安全距离
- `start` / `goal`：起点和目标点
- `obstacles`：圆形障碍物位置和半径

## 项目结构

```text
.
├── README.md
├── pyproject.toml
├── requirements.txt
├── run_demo.py
└── src/car_obstacle_mpc
    ├── __init__.py
    ├── config.py
    ├── mpc.py
    ├── plotting.py
    └── simulate.py
```
