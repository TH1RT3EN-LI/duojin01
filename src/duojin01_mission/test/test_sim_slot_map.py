from pathlib import Path
from unittest.mock import patch

from duojin01_mission.sim_slot_map import (
    SLOT_NAV_X_OFFSET_M,
    build_slot_waypoints,
    default_slot_world_path,
    load_cube_positions_from_world,
    normalize_slot_id,
    resolve_slot_waypoint,
)


WORLD_PATH = (
    Path(__file__).resolve().parents[2]
    / "duojin01_bringup"
    / "worlds"
    / "race_track.sdf"
)


def test_load_cube_positions_from_world_reads_all_12_cubes():
    positions = load_cube_positions_from_world(WORLD_PATH)

    assert len(positions) == 12
    assert positions["tag_cube_13"] == (3.035893, -2.657427)
    assert positions["tag_cube_24"] == (1.438865, -1.256578)


def test_build_slot_waypoints_assigns_b_and_c_rows_in_y_order():
    positions = load_cube_positions_from_world(WORLD_PATH)

    waypoints = build_slot_waypoints(positions)

    assert list(waypoints.keys()) == [
        "B1",
        "B2",
        "B3",
        "B4",
        "B5",
        "B6",
        "C1",
        "C2",
        "C3",
        "C4",
        "C5",
        "C6",
    ]
    assert waypoints["B1"].cube_name == "tag_cube_13"
    assert waypoints["B6"].cube_name == "tag_cube_18"
    assert waypoints["C1"].cube_name == "tag_cube_19"
    assert waypoints["C6"].cube_name == "tag_cube_24"


def test_build_slot_waypoints_uses_cube_x_minus_70cm_for_nav_x():
    positions = load_cube_positions_from_world(WORLD_PATH)

    waypoints = build_slot_waypoints(positions)

    assert waypoints["B1"].cube_x == 3.035893
    assert waypoints["B1"].nav_x == 3.035893 - SLOT_NAV_X_OFFSET_M
    assert waypoints["B1"].nav_y == -2.657427
    assert waypoints["C6"].cube_x == 1.438865
    assert waypoints["C6"].nav_x == 1.438865 - SLOT_NAV_X_OFFSET_M
    assert waypoints["C6"].nav_y == -1.256578


def test_resolve_slot_waypoint_accepts_case_insensitive_slot_id():
    positions = load_cube_positions_from_world(WORLD_PATH)
    waypoints = build_slot_waypoints(positions)

    assert normalize_slot_id(" b1 ") == "B1"
    assert resolve_slot_waypoint(waypoints, " b1 ").slot_id == "B1"
    assert resolve_slot_waypoint(waypoints, "c6").slot_id == "C6"
    assert resolve_slot_waypoint(waypoints, "D1") is None


def test_default_slot_world_path_comes_from_duojin01_bringup_share():
    with patch(
        "duojin01_mission.sim_slot_map.get_package_share_directory",
        return_value="/tmp/fake_share/duojin01_bringup",
    ):
        world_path = default_slot_world_path()

    assert world_path == Path("/tmp/fake_share/duojin01_bringup/worlds/race_track.sdf")
