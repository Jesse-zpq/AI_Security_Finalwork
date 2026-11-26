"""成员 1 的参考场景与二维运动更新。仅需 Python 标准库。

默认值对应 11 月 25 日新版方案及共用参数表 v1.0 中的
「公用参数」和「车辆与工况」。距离用 m，速度用 m/s，时间用 s。
坐标取车身中心，x 沿行驶方向为正，y 向左为正。

成员 5 的最小调用示例：
    from member1_scenario import BRAKE_ACCELERATION, DT, initial_states, step_reference_scene
    states = initial_states()
    states = step_reference_scene(states, 0.0, DT, ego_acceleration=BRAKE_ACCELERATION)

每次运行先调用 initial_states()。状态是场景真值，用于推进和评分。
策略输入由观测模块提供。步进函数接收策略已经选好的加速度和横移段。
直接运行 python member1_scenario.py 可以复现制动一步的试算。
正常观测 N1、N2 由同目录的 member1_n1_n2.py 使用本模块执行。
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from math import isclose, isfinite


LANE_WIDTH = 3.6
SHOULDER_WIDTH = 3.0
VEHICLE_LENGTH = 5.0
VEHICLE_WIDTH = 1.8
LEFT_LANE_Y = LANE_WIDTH
MIDDLE_LANE_Y = 0.0
RIGHT_LANE_Y = -LANE_WIDTH
SHOULDER_Y = -1.5 * LANE_WIDTH - 0.5 * SHOULDER_WIDTH
ROAD_MIN_Y = -1.5 * LANE_WIDTH - SHOULDER_WIDTH
ROAD_MAX_Y = 1.5 * LANE_WIDTH

FAULT_TIME = 0.0
DT = 0.1
LANE_CHANGE_DURATION = 3.5
MAX_RUN_TIME = 25.0
POST_STOP_OBSERVATION_TIME = 3.0
TRANSITION_ACCELERATION = -2.0
BRAKE_ACCELERATION = -4.0
REAR_BRAKE_TIME = 2.0
REAR_BRAKE_ACCELERATION = -5.0
LONGITUDINAL_MARGIN_M = 2.0
LATERAL_MARGIN_M = 0.2
FOLLOW_CHECK_S = 1.0


@dataclass(frozen=True)
class VehicleState:
    """同一时刻的车身中心坐标和向前行驶的速度。"""

    x: float
    y: float
    speed: float

    def __post_init__(self) -> None:
        if self.speed < 0 or not all(isfinite(value) for value in (self.x, self.y, self.speed)):
            raise ValueError("车辆坐标和速度必须是有限数值，速度必须大于等于零。")


@dataclass(frozen=True)
class LaneChange:
    """一段横移，起点和终点是横向坐标，开始时间是仿真时钟。"""

    start_time: float
    start_y: float
    target_y: float
    duration: float = LANE_CHANGE_DURATION

    def __post_init__(self) -> None:
        values = (self.start_time, self.start_y, self.target_y, self.duration)
        if not all(isfinite(value) for value in values) or self.duration <= 0:
            raise ValueError("横移参数必须是有限数值，时长必须大于零。")

    def y_at(self, time: float) -> float:
        """用五次曲线计算指定时刻的横向位置。"""
        u = min(1.0, max(0.0, (time - self.start_time) / self.duration))
        fraction = u**3 * (10.0 - 15.0 * u + 6.0 * u**2)
        return self.start_y + (self.target_y - self.start_y) * fraction


def initial_states() -> dict[str, VehicleState]:
    """返回 t = 0 时五辆车的初值。每次调用返回一份新字典。"""
    return {
        "E": VehicleState(x=0.0, y=MIDDLE_LANE_Y, speed=25.0),
        "F": VehicleState(x=50.0, y=MIDDLE_LANE_Y, speed=20.0),
        "L": VehicleState(x=-20.0, y=LEFT_LANE_Y, speed=33.0),
        "R": VehicleState(x=-50.0, y=RIGHT_LANE_Y, speed=27.0),
        "Bk": VehicleState(x=-60.0, y=MIDDLE_LANE_Y, speed=27.0),
    }


def step_vehicle(
    state: VehicleState,
    time: float,
    dt: float = DT,
    *,
    acceleration: float = 0.0,
    lane_change: LaneChange | None = None,
) -> VehicleState:
    """从 time 推进到 time + dt。加速度在这一步内恒定。

    横移段从实际起始 y 指向目标 y，按步末时刻采样。
    提前停稳时，纵向位置保持在停稳点。
    """
    if not all(isfinite(value) for value in (time, dt, acceleration)) or dt < 0:
        raise ValueError("时刻、步长和加速度必须是有限数值，步长必须大于等于零。")
    if dt == 0:
        return state

    motion_time = dt
    if acceleration < 0:
        motion_time = min(dt, state.speed / -acceleration)
    x = state.x + state.speed * motion_time + 0.5 * acceleration * motion_time**2
    speed = max(0.0, state.speed + acceleration * motion_time)

    y = state.y
    if lane_change is not None and time + dt >= lane_change.start_time:
        y = lane_change.y_at(time + dt)
    return VehicleState(x=x, y=y, speed=speed)


def step_reference_scene(
    states: Mapping[str, VehicleState],
    time: float,
    dt: float = DT,
    *,
    ego_acceleration: float = 0.0,
    ego_lane_change: LaneChange | None = None,
) -> dict[str, VehicleState]:
    """用同一个时间步推进本车和参考车流，返回新状态字典。

    F、L、R 保持当前速度与车道。Bk 在仿真 2.0 s 时开始以
    −5 m/s² 制动，停下后保持静止。跨过制动切换点的步会分段推进。
    中道到右道、右道到路肩各用一段 LaneChange，由调用方切换当前段。
    """
    next_states = {
        "E": step_vehicle(
            states["E"], time, dt, acceleration=ego_acceleration, lane_change=ego_lane_change
        )
    }
    for vehicle_id, state in states.items():
        if vehicle_id == "E":
            continue
        if vehicle_id == "Bk":
            coast_time = min(dt, max(0.0, REAR_BRAKE_TIME - time))
            state = step_vehicle(state, time, coast_time)
            next_states[vehicle_id] = step_vehicle(
                state,
                time + coast_time,
                dt - coast_time,
                acceleration=REAR_BRAKE_ACCELERATION,
            )
        else:
            next_states[vehicle_id] = step_vehicle(state, time, dt)
    return next_states


if __name__ == "__main__":
    before = initial_states()
    after = step_reference_scene(before, FAULT_TIME, DT, ego_acceleration=BRAKE_ACCELERATION)
    ego = after["E"]
    expected = VehicleState(x=2.48, y=0.0, speed=24.6)
    measured = (ego.x, ego.y, ego.speed)
    hand_calculated = (expected.x, expected.y, expected.speed)
    if not all(isclose(value, target, abs_tol=1e-10) for value, target in zip(measured, hand_calculated)):
        raise SystemExit(f"制动一步校验失败：实际 {measured}，手算 {hand_calculated}")
    print(f"制动一步：t = {DT:.2f} s，x = {ego.x:.2f} m，y = {ego.y:.2f} m，v = {ego.speed:.2f} m/s")
    print("与 Excel「成员1交接」页的手算结果一致。")
