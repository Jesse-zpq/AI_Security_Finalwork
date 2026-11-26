"""Build the course experiment manifest and verify a dedicated CARLA server."""

import argparse
import json
import math
import socket
from pathlib import Path
from xml.etree import ElementTree as ET


ROAD_LENGTH = 1200.0
EGO_X_ORIGIN = 100.0
LANE_WIDTH = 3.6
SHOULDER_WIDTH = 3.0
LANES = {
    "left": (-1, 3.6, LANE_WIDTH, "Driving"),
    "middle": (-2, 0.0, LANE_WIDTH, "Driving"),
    "right": (-3, -3.6, LANE_WIDTH, "Driving"),
    "shoulder": (-4, -6.9, SHOULDER_WIDTH, "Shoulder"),
}
REFERENCE = {
    "E": {"x_m": 0.0, "lane": "middle", "speed_mps": 25.0},
    "F": {"x_m": 50.0, "lane": "middle", "speed_mps": 20.0},
    "L": {"x_m": -20.0, "lane": "left", "speed_mps": 33.0},
    "R": {"x_m": -50.0, "lane": "right", "speed_mps": 27.0},
    "Bk": {"x_m": -60.0, "lane": "middle", "speed_mps": 27.0},
}


def make_runs():
    conditions = [
        ("N1", "left", None, False, "P2", {}, "A"),
        ("N2", "follow", None, False, "P4", {}, "A"),
        ("P1", "left", 2.0, True, "P2", {}, "ABC"),
        ("P2", "left", None, True, None, {}, "ABC"),
        ("P3", "follow", 2.0, True, "P4", {}, "ABC"),
        ("P4", "follow", None, True, None, {}, "ABC"),
        ("T1", "left", None, True, "P2", {"L": None}, "ABC"),
        ("T2", "follow", None, True, "P4", {"R": {"x_m": -10.0}}, "ABC"),
        ("T3", "follow", None, True, "P4", {"Bk": {"x_m": -20.0}}, "ABC"),
        ("X1", "follow", None, True, "P4", {
            "F": {"x_m": 10.0},
            "R": {"x_m": -10.0},
            "L": {"cut_in_at_s": 1.0, "cut_in_duration_s": 3.5},
        }, "C"),
    ]
    runs = []
    for name, intent, takeover, fault, pair, changes, strategies in conditions:
        for strategy in strategies:
            pair_id = f"{pair}-{strategy}" if pair else None
            runs.append({
                "run_id": f"{name}-{strategy}",
                "role": "reference" if not fault else "boundary" if name == "X1" else "fault",
                "strategy": strategy,
                "pair_with": pair_id,
                "intent": intent,
                "readiness_event_s": takeover,
                "left_observation_lost": fault,
                "expand_left_prior_to_middle": name == "X1",
                "actor_overrides": changes,
            })
    return runs


def build_manifest():
    return {
        "status": "draft_inputs_for_calibration",
        "coordinates": "vehicle centers, +x forward, +y left, course x=0 at road s=100 m",
        "road_length_m": ROAD_LENGTH,
        "lanes": {name: {"xodr_lane_id": lane_id, "y_m": y, "width_m": width}
                  for name, (lane_id, y, width, _) in LANES.items()},
        "reference_actors": REFERENCE,
        "reference_actor_motion": {
            "F_L_R": "hold initial speed and lane unless overridden",
            "Bk": {"brake_at_s": 2.0, "acceleration_mps2": -5.0, "speed_floor_mps": 0.0},
        },
        "parameters": {
            "physics_step_s": 0.05,
            "decision_step_s": 0.1,
            "fault_at_s": 0.0,
            "horizon_s": 25.0,
            "post_stop_observation_s": 3.0,
            "max_wait_s": 5.0,
            "mrm_launch_margin_s": 0.3,
            "handover_buffer_s": 0.5,
            "lateral_segment_s": 3.5,
            "braking_target_mps2": -4.0,
            "mild_braking_target_mps2": -2.0,
            "nominal_follow_desired_speed_mps": 25.0,
            "nominal_follow_standoff_m": 5.0,
            "nominal_follow_headway_s": 1.4,
            "nominal_follow_speed_gain_per_s": 0.6,
            "nominal_follow_gap_gain_per_s2": 0.3,
            "predicted_rear_response_delay_s": [1.5, 2.0],
            "predicted_rear_braking_magnitude_mps2": [5.0, 5.5],
            "longitudinal_clearance_margin_m": 2.0,
            "lateral_clearance_margin_m": 0.2,
            "fallback_score_tie_margin_m": 0.2,
            "left_prior_x_m": [-30.0, -15.0],
            "left_prior_v_mps": [32.0, 34.0],
            "left_prior_lane_center_offset_m": 0.3,
        },
        "runs": make_runs(),
    }


def build_road():
    root = ET.Element("OpenDRIVE")
    ET.SubElement(root, "header", revMajor="1", revMinor="4", name="CourseHighway",
                  version="1.00", north="0", south="-14", east="1200", west="0")
    road = ET.SubElement(root, "road", name="CourseHighway", id="1",
                         length=str(ROAD_LENGTH), junction="-1")
    road_type = ET.SubElement(road, "type", s="0", type="motorway")
    ET.SubElement(road_type, "speed", max="36", unit="m/s")
    plan = ET.SubElement(road, "planView")
    geometry = ET.SubElement(plan, "geometry", s="0", x="0", y="0", hdg="0",
                             length=str(ROAD_LENGTH))
    ET.SubElement(geometry, "line")
    elevation = ET.SubElement(road, "elevationProfile")
    ET.SubElement(elevation, "elevation", s="0", a="0", b="0", c="0", d="0")
    section = ET.SubElement(ET.SubElement(road, "lanes"), "laneSection", s="0")
    center = ET.SubElement(ET.SubElement(section, "center"), "lane", id="0",
                           type="none", level="false")
    ET.SubElement(center, "roadMark", sOffset="0", type="solid", color="white",
                  weight="standard", width="0.15", laneChange="none")
    right = ET.SubElement(section, "right")
    for lane_id, _, width, kind in LANES.values():
        lane = ET.SubElement(right, "lane", id=str(lane_id), type=kind.lower(), level="false")
        ET.SubElement(lane, "width", sOffset="0", a=str(width), b="0", c="0", d="0")
        is_inner = lane_id in (-1, -2)
        ET.SubElement(lane, "roadMark", sOffset="0", type="broken" if is_inner else "solid",
                      color="white", weight="standard", width="0.15",
                      laneChange="both" if is_inner else "none")
    ET.indent(root)
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def verify_offline(manifest, road_text):
    runs = manifest["runs"]
    ids = {run["run_id"] for run in runs}
    assert len(ids) == len(runs) == 24
    assert sum(run["role"] == "fault" for run in runs) == 21
    assert all(run["pair_with"] is None or run["pair_with"] in ids for run in runs)
    assert len(ET.fromstring(road_text).findall(".//laneSection/right/lane")) == 4
    x = 25.0 * 0.1 - 0.5 * 4.0 * 0.1**2
    assert math.isclose(x, 2.48)
    assert math.isclose(25.0 / 4.0, 6.25)
    assert math.isclose(20.0 / (33.0 - 25.0), 2.5)
    assert math.isclose(20.0 + (25.0 * 2.0 - 0.5 * 4.0 * 2.0**2) - 27.0 * 2.0 - 5.0, 3.0)
    print("Offline checks passed: 21 fault runs, 2 normal references, 1 boundary run.")


def probe_server(host, port, road_text, load_geometry):
    try:
        with socket.create_connection((host, port), timeout=4):
            pass
    except OSError as exc:
        raise SystemExit(f"CARLA TCP {host}:{port} unavailable: {exc}") from exc
    try:
        import carla
    except ImportError as exc:
        raise SystemExit("Install the CARLA 1.0 wheel matching this Python interpreter.") from exc
    client = carla.Client(host, port)
    cache = Path(__file__).resolve().parents[1] / 'work' / 'preflight_carla_cache'
    cache.mkdir(parents=True, exist_ok=True)
    client.set_files_base_folder(str(cache.resolve()))
    client.set_timeout(60.0 if load_geometry else 10.0)
    client_version = client.get_client_version()
    server_version = client.get_server_version()
    if client_version != server_version:
        raise SystemExit(f"CARLA version mismatch: {client_version=} {server_version=}")
    print(f"CARLA {server_version}; current map = {client.get_world().get_map().name}")
    if not load_geometry:
        return
    params = carla.OpendriveGenerationParameters(wall_height=0.0, additional_width=0.0)
    world = client.generate_opendrive_world(road_text, params)
    carla_map = world.get_map()
    middle = carla_map.get_waypoint_xodr(1, -2, EGO_X_ORIGIN)
    if middle is None:
        raise SystemExit("OpenDRIVE load did not expose the middle lane.")
    for name, (lane_id, y, width, kind) in LANES.items():
        waypoint = carla_map.get_waypoint_xodr(1, lane_id, EGO_X_ORIGIN)
        if waypoint is None:
            raise SystemExit(f"OpenDRIVE load did not expose {name}.")
        measured_y = middle.transform.location.y - waypoint.transform.location.y
        if (waypoint.lane_type != getattr(carla.LaneType, kind)
                or not math.isclose(measured_y, y, abs_tol=0.05)
                or not math.isclose(waypoint.lane_width, width, abs_tol=0.05)
                or not math.isclose(waypoint.transform.rotation.yaw, 0, abs_tol=0.05)):
            raise SystemExit(f"Unexpected geometry for {name}: {waypoint.transform}")
        print(f"{name}: course y={measured_y:.2f} m, width={waypoint.lane_width:.2f} m")
    check_shoulder_support(carla, world, carla_map)


def check_shoulder_support(carla, world, carla_map):
    saved_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    actor = None
    try:
        waypoint = carla_map.get_waypoint_xodr(1, -4, EGO_X_ORIGIN)
        location = waypoint.transform.location
        location.z += 0.45
        vehicles = world.get_blueprint_library().filter("vehicle.*")
        cars = [bp for bp in vehicles if bp.has_attribute("number_of_wheels")
                and bp.get_attribute("number_of_wheels").as_int() == 4]
        if not cars:
            raise SystemExit("No four-wheel vehicle blueprint available.")
        blueprint = sorted(cars, key=lambda bp: bp.id)[0]
        actor = world.try_spawn_actor(blueprint, carla.Transform(location, waypoint.transform.rotation))
        if actor is None:
            raise SystemExit("Could not spawn a witness car above the shoulder.")
        actor.apply_control(carla.VehicleControl(hand_brake=True))
        for _ in range(30):
            world.tick()
        settled = actor.get_location()
        for _ in range(10):
            world.tick()
        after = actor.get_location()
        if abs(after.z - (location.z - 0.45)) > 0.75 or after.distance(settled) > 0.08:
            raise SystemExit(f"Shoulder support check failed: {settled=} {after=}")
        print(f"Shoulder support witness settled; blueprint={blueprint.id}.")
    finally:
        if actor is not None:
            actor.destroy()
        world.apply_settings(saved_settings)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-dir", type=Path, help="write experiment_manifest.json and course_highway.xodr")
    parser.add_argument("--probe", action="store_true", help="test a running CARLA server")
    parser.add_argument("--geometry", action="store_true", help="load and check map on a dedicated server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2200)
    args = parser.parse_args()
    manifest = build_manifest()
    road = build_road()
    verify_offline(manifest, road)
    if args.write_dir:
        args.write_dir.mkdir(parents=True, exist_ok=True)
        (args.write_dir / "experiment_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                                                   encoding="utf-8")
        (args.write_dir / "course_highway.xodr").write_text(road + "\n", encoding="utf-8")
        print(f"Wrote the manifest and road to {args.write_dir.resolve()}")
    if args.probe or args.geometry:
        probe_server(args.host, args.port, road, args.geometry)


if __name__ == "__main__":
    main()
