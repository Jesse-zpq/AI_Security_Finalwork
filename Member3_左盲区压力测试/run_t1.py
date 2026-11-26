"""Replay the formal P2/T1 pairs, rerun the 2D model and audit C's deadlines."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys


def save_csv(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def check_pair(reference, stress):
    p, t = reference["case"], stress["case"]
    varied = {"run_id", "pair_with", "actor_overrides"}
    assert {k: v for k, v in p.items() if k not in varied} == {
        k: v for k, v in t.items() if k not in varied
    }
    assert t["pair_with"] == p["run_id"] and p["actor_overrides"] == {} and t["actor_overrides"] == {"L": None}
    assert p["intent"] == "left" and p["left_observation_lost"] and p["readiness_event_s"] is None
    assert not p["expand_left_prior_to_middle"]
    for record, has_left in ((reference, True), (stress, False)):
        for frame in record["frames"]:
            assert ("L" in frame["states"]) == has_left and "L" not in frame["observed"]
    assert reference["frames"][0]["observed"] == stress["frames"][0]["observed"] == ["F", "R", "Bk"]


def summary(record):
    score = record["score"]
    first = record["frames"][0]
    events = {e["event"]: e for e in record["events"]}
    hit = score["first_collision"] or {}
    stop = events.get("fallback_started", {})
    row = {
        "run_id": record["run_id"], "backend": record["backend"],
        "matched_run": ("T1" if record["run_id"].startswith("P2") else "P2") + "-" + record["case"]["strategy"],
        "L_in_world": "L" in first["states"], "observed_at_0": "/".join(first["observed"]),
        "action_at_0": first["action"], "left_decision": next((s for s in ("left_accepted", "left_rejected") if s in events), "not_evaluated"),
        "left_reject_reason": events.get("left_rejected", {}).get("trigger"),
        "takeover_request_s": events.get("takeover_requested", {}).get("time_s"),
        "handover_s": events.get("scripted_handover", {}).get("time_s"),
        "fallback": stop.get("action"), "fallback_s": stop.get("time_s"),
        "left_lane_follow_s": next((f["time_s"] for f in record["frames"] if f["action"] == "left_lane_follow"), None),
        "first_below_stop_speed_s": events.get("observed_stop", {}).get("first_below_threshold_s"),
        "collision_actor": hit.get("other"), "collision_s": hit.get("time_s"),
        "outcome": score["outcome"], "end_s": score["terminal_time_s"],
    }
    for window, prefix in (("shared_window_s", "first2s"), ("until_endpoint", "full")):
        for metric in ("front_gap_m", "rear_gap_m", "left_gap_m", "right_gap_m", "longitudinal_ttc_s", "ego_acceleration_mps2"):
            v = score[window][metric]
            row[f"{prefix}_{metric}"] = round(v["value"], 6) if v else None
    return row


def audit_deadlines(record):
    from replay import frame_input
    from carla_pilot import DECISION_DT, PARAMS, Plan, Policy, check_plan, follow_acceleration, planned_state

    policy = Policy(record["case"])
    launch = next(e for e in record["events"] if e["event"] == "fallback_started")
    rows = []
    previous = math.inf
    stride = round(DECISION_DT / record["physics_step_s"])
    for frame in record["frames"][::stride]:
        now = frame["time_s"]
        if now > launch["time_s"]:
            break
        ego, observed, rear_a = frame_input(record, now)
        options, _ = policy.stop_options(ego, now, now, observed, rear_a)
        assert options, f"No current fallback at {now}"
        wait = Plan("wait", now, ego, min(0.0, follow_acceleration(ego, observed)))
        safe_until = now
        for tick in range(1, 1 + round((PARAMS["max_wait_s"] - now) / DECISION_DT)):
            future = now + tick * DECISION_DT
            _, conflict = check_plan(wait, now, future - now, observed, rear_a, policy.blind_prior)
            future_options, _ = policy.stop_options(planned_state(wait, future), future, now, observed, rear_a)
            if conflict or not future_options:
                break
            safe_until = future
        deadline = min(previous, max(now, safe_until - PARAMS["mrm_launch_margin_s"]))
        assert math.isclose(deadline, frame["deadline_s"], abs_tol=1e-7)
        rows.append({"run_id": record["run_id"], "decision_s": now,
                     "available_now": "/".join(plan.name for _, plan in options),
                     "wait_feasible_until_s": round(safe_until, 6),
                     "recalculated_deadline_s": round(deadline, 6), "logged_deadline_s": frame["deadline_s"],
                     "action": frame["action"]})
        previous = deadline
    assert rows[-1]["decision_s"] == launch["time_s"] == launch["deadline_s"]
    return rows


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=here.parent)
    parser.add_argument("--output", type=Path, default=here / "results")
    args = parser.parse_args()
    handoff = args.repo / "CARLA_Member3-4-5_Handoff" / "CARLA_repro_handoff"
    sys.path.insert(0, str(handoff))
    from replay import load_case, reproduce
    from member3_action_filter import explain_frame
    from carla_pilot import CORRIDOR, PARAMS, ModelWorld, run_case
    from carla_metrics import score_run
    from carla_preflight import make_runs

    manifest = {c["run_id"]: c for c in make_runs()}
    formal, models, deadlines = [], [], []
    n1 = load_case("N1-A")
    reproduce(n1)
    evidence = {"N1-A": [explain_frame(n1, 0.0)]}
    for strategy in "ABC":
        pair = [load_case(f"{group}-{strategy}") for group in ("P2", "T1")]
        check_pair(*pair)
        for record in pair:
            key = record["run_id"]
            assert record["case"] == manifest[key]
            reproduce(record)
            formal.append(summary(record))
            case = manifest[key]
            simulated = run_case(case, ModelWorld(case), PARAMS["horizon_s"])
            simulated.update(run_id=key, case=case, backend="model")
            simulated["score"] = score_run(simulated, PARAMS["longitudinal_clearance_margin_m"],
                                          PARAMS["lateral_clearance_margin_m"], CORRIDOR)
            models.append(summary(simulated))
            if strategy in ("A", "C"):
                evidence[key] = [explain_frame(record, 0.0)]
            if strategy == "C":
                audit = audit_deadlines(record)
                deadlines.extend(audit)
                evidence[key].append(explain_frame(record, audit[-1]["decision_s"]))
    args.output.mkdir(parents=True, exist_ok=True)
    save_csv(args.output / "formal_pairs.csv", formal)
    save_csv(args.output / "model_pairs.csv", models)
    save_csv(args.output / "deadline_audit.csv", deadlines)
    (args.output / "decision_evidence.json").write_text(
        json.dumps(evidence, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"{len(formal)} formal runs exactly replayed, N1-A checked, {len(models)} model runs, "
          f"{len(deadlines)} deadline checks passed.")


if __name__ == "__main__":
    main()
