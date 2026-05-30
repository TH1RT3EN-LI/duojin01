from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


SLOT_NAV_X_OFFSET_M = 0.70
RIGHT_ROW_CUBES = tuple(f"tag_cube_{index}" for index in range(13, 19))
LEFT_ROW_CUBES = tuple(f"tag_cube_{index}" for index in range(19, 25))
EXPECTED_CUBE_NAMES = RIGHT_ROW_CUBES + LEFT_ROW_CUBES


@dataclass(frozen=True)
class SlotWaypoint:
    slot_id: str
    cube_name: str
    cube_x: float
    cube_y: float
    nav_x: float
    nav_y: float


def load_cube_positions_from_world(world_path: str | Path) -> Dict[str, tuple[float, float]]:
    world_path = Path(world_path)
    root = ET.parse(world_path).getroot()

    positions: Dict[str, tuple[float, float]] = {}
    for include in root.findall(".//include"):
        name_text = include.findtext("name", default="").strip()
        if name_text not in EXPECTED_CUBE_NAMES:
            continue

        pose_text = include.findtext("pose", default="").strip()
        pose_values = pose_text.split()
        if len(pose_values) < 2:
            raise ValueError(f"Invalid pose for {name_text!r} in {world_path}: {pose_text!r}")

        positions[name_text] = (float(pose_values[0]), float(pose_values[1]))

    missing = [cube_name for cube_name in EXPECTED_CUBE_NAMES if cube_name not in positions]
    if missing:
        raise ValueError(f"Missing cube definitions in {world_path}: {', '.join(missing)}")

    return positions


def default_slot_world_path() -> Path:
    share_dir = Path(get_package_share_directory("duojin01_bringup"))
    return share_dir / "worlds" / "race_track.sdf"


def build_slot_waypoints(
    cube_positions: Mapping[str, tuple[float, float]],
    nav_x_offset_m: float = SLOT_NAV_X_OFFSET_M,
) -> Dict[str, SlotWaypoint]:
    waypoints: Dict[str, SlotWaypoint] = {}

    for prefix, cube_names in (("B", RIGHT_ROW_CUBES), ("C", LEFT_ROW_CUBES)):
        ordered = sorted(
            (
                (cube_name, cube_positions[cube_name][0], cube_positions[cube_name][1])
                for cube_name in cube_names
            ),
            key=lambda item: item[2],
        )

        for index, (cube_name, cube_x, cube_y) in enumerate(ordered, start=1):
            slot_id = f"{prefix}{index}"
            waypoints[slot_id] = SlotWaypoint(
                slot_id=slot_id,
                cube_name=cube_name,
                cube_x=cube_x,
                cube_y=cube_y,
                nav_x=cube_x - nav_x_offset_m,
                nav_y=cube_y,
            )

    return waypoints


def normalize_slot_id(slot_id: str) -> str:
    return slot_id.strip().upper()


def resolve_slot_waypoint(
    slot_waypoints: Mapping[str, SlotWaypoint],
    slot_id: str,
) -> SlotWaypoint | None:
    return slot_waypoints.get(normalize_slot_id(slot_id))
