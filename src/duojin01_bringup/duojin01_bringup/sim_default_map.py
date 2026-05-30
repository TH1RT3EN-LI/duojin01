from __future__ import annotations

import hashlib
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from .map_paths import _PACKAGE_NAME, get_workspace_maps_dir, pick_latest_map_yaml, resolve_map_yaml


_FREE_VALUE = 254
_OCCUPIED_VALUE = 0
_MAP_PADDING_M = 0.30
_MAP_RESOLUTION_M = 0.02
_LASER_PLANE_Z_M = 0.11


def resolve_sim_navigation_map(map_value: str, world_value: str, package_name: str = _PACKAGE_NAME) -> str:
    raw_value = map_value.strip()
    if raw_value:
        return resolve_map_yaml(raw_value, package_name)

    try:
        return pick_latest_map_yaml(package_name)
    except FileNotFoundError:
        return ensure_default_sim_map_for_world(world_value)


def ensure_default_sim_map_for_world(world_value: str) -> str:
    world_path = Path(world_value).expanduser().resolve(strict=False)
    if not world_path.is_file():
        raise FileNotFoundError(f"World file not found: {world_path}")

    if world_path.stem != "race_track":
        maps_dir = get_workspace_maps_dir(_PACKAGE_NAME)
        raise FileNotFoundError(
            f"No map yaml found under workspace root maps directory: {maps_dir}. "
            f"Built-in sim default map is only available for world 'race_track.sdf'; "
            f"received '{world_path.name}'. "
            f"Place maps in {maps_dir} or pass an explicit map:=... path."
        )

    cache_dir = Path(tempfile.gettempdir()) / "duojin01_sim_maps"
    cache_dir.mkdir(parents=True, exist_ok=True)

    world_key = hashlib.sha1(str(world_path).encode("utf-8")).hexdigest()[:10]
    map_stem = f"sim_default_{world_path.stem}_{world_key}"
    yaml_path = cache_dir / f"{map_stem}.yaml"
    pgm_path = cache_dir / f"{map_stem}.pgm"

    if (
        yaml_path.is_file()
        and pgm_path.is_file()
        and yaml_path.stat().st_mtime >= world_path.stat().st_mtime
        and pgm_path.stat().st_mtime >= world_path.stat().st_mtime
    ):
        return str(yaml_path)

    _generate_race_track_map(world_path=world_path, yaml_path=yaml_path, pgm_path=pgm_path)
    return str(yaml_path)


def _generate_race_track_map(*, world_path: Path, yaml_path: Path, pgm_path: Path) -> None:
    polygons = _extract_race_track_obstacle_polygons(world_path)
    if not polygons:
        raise RuntimeError(f"No obstacle geometry intersected the lidar plane in world: {world_path}")

    min_x = min(point[0] for polygon in polygons for point in polygon) - _MAP_PADDING_M
    max_x = max(point[0] for polygon in polygons for point in polygon) + _MAP_PADDING_M
    min_y = min(point[1] for polygon in polygons for point in polygon) - _MAP_PADDING_M
    max_y = max(point[1] for polygon in polygons for point in polygon) + _MAP_PADDING_M

    width = max(1, int(math.ceil((max_x - min_x) / _MAP_RESOLUTION_M)))
    height = max(1, int(math.ceil((max_y - min_y) / _MAP_RESOLUTION_M)))
    pixels = bytearray([_FREE_VALUE]) * (width * height)

    for polygon in polygons:
        poly_min_x = min(point[0] for point in polygon)
        poly_max_x = max(point[0] for point in polygon)
        poly_min_y = min(point[1] for point in polygon)
        poly_max_y = max(point[1] for point in polygon)

        min_px = max(0, int(math.floor((poly_min_x - min_x) / _MAP_RESOLUTION_M)))
        max_px = min(width - 1, int(math.ceil((poly_max_x - min_x) / _MAP_RESOLUTION_M)))
        min_py = max(0, int(math.floor((poly_min_y - min_y) / _MAP_RESOLUTION_M)))
        max_py = min(height - 1, int(math.ceil((poly_max_y - min_y) / _MAP_RESOLUTION_M)))

        for py in range(min_py, max_py + 1):
            sample_y = min_y + (py + 0.5) * _MAP_RESOLUTION_M
            image_row = height - 1 - py
            for px in range(min_px, max_px + 1):
                sample_x = min_x + (px + 0.5) * _MAP_RESOLUTION_M
                if _point_in_polygon((sample_x, sample_y), polygon):
                    pixels[image_row * width + px] = _OCCUPIED_VALUE

    with pgm_path.open("wb") as pgm_file:
        pgm_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        pgm_file.write(pixels)

    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                f"resolution: {_MAP_RESOLUTION_M:.3f}",
                f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def _extract_race_track_obstacle_polygons(world_path: Path) -> list[list[tuple[float, float]]]:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"Missing <world> in: {world_path}")

    race_track_model = world.find("model[@name='race_track']")
    if race_track_model is None:
        raise RuntimeError(f"Missing race_track model in: {world_path}")

    model_rotation, model_translation = _pose_to_transform(race_track_model.findtext("pose", default="0 0 0 0 0 0"))
    polygons: list[list[tuple[float, float]]] = []

    for collision in race_track_model.findall(".//collision"):
        box = collision.find("geometry/box/size")
        if box is None:
            continue

        size = tuple(float(value) for value in box.text.split())
        local_rotation, local_translation = _pose_to_transform(collision.findtext("pose", default="0 0 0 0 0 0"))
        world_rotation, world_translation = _combine_transforms(
            model_rotation,
            model_translation,
            local_rotation,
            local_translation,
        )

        corners = _box_world_corners(world_rotation, world_translation, size)
        z_values = [corner[2] for corner in corners]
        if min(z_values) > _LASER_PLANE_Z_M or max(z_values) < _LASER_PLANE_Z_M:
            continue

        polygon = _convex_hull([(corner[0], corner[1]) for corner in corners])
        if len(polygon) >= 3:
            polygons.append(polygon)

    return polygons


def _pose_to_transform(pose_text: str) -> tuple[list[list[float]], tuple[float, float, float]]:
    x, y, z, roll, pitch, yaw = (float(value) for value in pose_text.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return rotation, (x, y, z)


def _combine_transforms(
    parent_rotation: list[list[float]],
    parent_translation: tuple[float, float, float],
    child_rotation: list[list[float]],
    child_translation: tuple[float, float, float],
) -> tuple[list[list[float]], tuple[float, float, float]]:
    rotation = _matrix_multiply(parent_rotation, child_rotation)
    translation = _transform_point(parent_rotation, parent_translation, child_translation)
    return rotation, translation


def _matrix_multiply(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][idx] * right[idx][col] for idx in range(3)) for col in range(3)]
        for row in range(3)
    ]


def _transform_point(
    rotation: list[list[float]],
    translation: tuple[float, float, float],
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        translation[0] + sum(rotation[0][idx] * point[idx] for idx in range(3)),
        translation[1] + sum(rotation[1][idx] * point[idx] for idx in range(3)),
        translation[2] + sum(rotation[2][idx] * point[idx] for idx in range(3)),
    )


def _box_world_corners(
    rotation: list[list[float]],
    translation: tuple[float, float, float],
    size: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
    half_x, half_y, half_z = (axis_size / 2.0 for axis_size in size)
    corners: list[tuple[float, float, float]] = []

    for local_x in (-half_x, half_x):
        for local_y in (-half_y, half_y):
            for local_z in (-half_z, half_z):
                corners.append(
                    _transform_point(rotation, translation, (local_x, local_y, local_z))
                )

    return corners


def _convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    unique_points = sorted(set((round(x, 9), round(y, 9)) for x, y in points))
    if len(unique_points) <= 1:
        return unique_points

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique_points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)

    upper: list[tuple[float, float]] = []
    for point in reversed(unique_points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)

    return lower[:-1] + upper[:-1]


def _point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    for idx, current in enumerate(polygon):
        previous = polygon[idx - 1]
        x1, y1 = previous
        x2, y2 = current

        intersects = ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1
        )
        if intersects:
            inside = not inside

    return inside
