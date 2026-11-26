"""运行 N1、N2 的正常观测参照，输出两次决策及选中动作的轨迹。

与 member1_scenario.py 放在同一目录。
运行 python member1_n1_n2.py 可在终端查看 JSON。
加 --output 路径可保存相同内容。正常观测是二维模型中的理想目标列表。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from member1_scenario import (
    DT,
    FOLLOW_CHECK_S,
    LANE_CHANGE_DURATION,
    LATERAL_MARGIN_M,
    LEFT_LANE_Y,
    LONGITUDINAL_MARGIN_M,
    VEHICLE_LENGTH,
    VEHICLE_WIDTH,
    LaneChange,
    VehicleState,
    initial_states,
    step_reference_scene,
)


LEFT_CHANGE = "left_change"
FOLLOW_LANE = "follow_lane"


def normal_observation(states: dict[str, VehicleState]) -> dict[str, VehicleState]:
    """无故障时返回场景中所有其他车辆的理想目标观测。"""
    return {vehicle: state for vehicle, state in states.items() if vehicle != "E"}


def rollout(
    states: dict[str, VehicleState], horizon: float, lane_change: LaneChange | None = None
) -> list[tuple[float, dict[str, VehicleState]]]:
    """用共用运动模型滚动预测或执行恒速候选，包含起点和终点。"""
    ticks = round(horizon / DT)
    frames = [(0.0, states)]
    for tick in range(ticks):
        time = round(tick * DT, 10)
        states = step_reference_scene(states, time, dt=DT, ego_lane_change=lane_change)
        frames.append((round((tick + 1) * DT, 10), states))
    return frames


def first_contacts(frames: list[tuple[float, dict[str, VehicleState]]]) -> dict:
    """按矩形车身计算最先出现的裕度不足及几何重叠。"""
    margin_conflict = None
    body_overlap = None
    for time, states in frames:
        ego = states["E"]
        for vehicle, target in normal_observation(states).items():
            gap_x = abs(ego.x - target.x) - VEHICLE_LENGTH
            gap_y = abs(ego.y - target.y) - VEHICLE_WIDTH
            detail = {
                "vehicle": vehicle,
                "time_s": time,
                "longitudinal_edge_gap_m": round(gap_x, 6),
                "lateral_edge_gap_m": round(gap_y, 6),
            }
            if margin_conflict is None and gap_x < LONGITUDINAL_MARGIN_M and gap_y < LATERAL_MARGIN_M:
                margin_conflict = detail
            if body_overlap is None and gap_x < 0 and gap_y < 0:
                body_overlap = detail
    return {"margin_conflict": margin_conflict, "body_overlap": body_overlap}


def assess_candidate(
    ego: VehicleState, observations: dict[str, VehicleState], action: str
) -> dict:
    """常规规划读取本车状态和已观测目标，检查行动候选。"""
    lane_change = LaneChange(0.0, ego.y, LEFT_LANE_Y) if action == LEFT_CHANGE else None
    horizon = LANE_CHANGE_DURATION if lane_change is not None else FOLLOW_CHECK_S
    predicted = rollout({"E": ego, **observations}, horizon, lane_change)
    contacts = first_contacts(predicted)
    return {
        "action": action,
        "check_horizon_s": horizon,
        "passes_check": contacts["margin_conflict"] is None,
        **contacts,
    }


def run_case(case_id: str, requested: str) -> dict:
    states = initial_states()
    observations = normal_observation(states)
    checks = [assess_candidate(states["E"], observations, requested)]
    selected = requested
    if not checks[0]["passes_check"] and requested == LEFT_CHANGE:
        selected = FOLLOW_LANE
        checks.append(assess_candidate(states["E"], observations, selected))
    if not checks[-1]["passes_check"]:
        raise RuntimeError(f"{case_id} 的参考条件下没有通过检查的正常候选动作。")

    lane_change = LaneChange(0.0, states["E"].y, LEFT_LANE_Y) if selected == LEFT_CHANGE else None
    executed = rollout(states, FOLLOW_CHECK_S, lane_change)
    _, last_states = executed[-1]
    return {
        "case_id": case_id,
        "requested_intent": requested,
        "fault_injected": False,
        "initial_observed_vehicles": {vehicle: asdict(state) for vehicle, state in observations.items()},
        "left_rear_L_observed": "L" in observations,
        "intent_adjusted": requested != selected,
        "selected_action": selected,
        "candidate_checks": checks,
        "executed_horizon_s": FOLLOW_CHECK_S,
        "end_front_edge_gap_m": round(last_states["F"].x - last_states["E"].x - VEHICLE_LENGTH, 6),
        "executed_contacts": first_contacts(executed),
        "execution_trace": [
            {
                "time_s": time,
                "ego": asdict(frame["E"]),
                "left_rear_L": asdict(frame["L"]),
                "front_F": asdict(frame["F"]),
            }
            for time, frame in executed
        ],
    }


def run_reference_cases() -> dict:
    return {
        "model": "二维运动学，正常观测为理想目标列表",
        "strategy": "A",
        "takeover": "none",
        "dt_s": DT,
        "longitudinal_margin_m": LONGITUDINAL_MARGIN_M,
        "lateral_margin_m": LATERAL_MARGIN_M,
        "initial_states": {vehicle: asdict(state) for vehicle, state in initial_states().items()},
        "cases": [run_case("N1", LEFT_CHANGE), run_case("N2", FOLLOW_LANE)],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="将 JSON 实跑记录保存到指定路径")
    args = parser.parse_args()
    body = json.dumps(run_reference_cases(), ensure_ascii=False, indent=2) + "\n"
    if args.output is None:
        print(body, end="")
    else:
        args.output.write_text(body, encoding="utf-8")
        print(f"已保存 N1、N2 实跑记录到 {args.output}")


if __name__ == "__main__":
    main()
