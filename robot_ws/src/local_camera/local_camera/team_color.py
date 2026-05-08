import os

import yaml
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory


def normalize_team_color(color):
    color = str(color).strip().lower()
    if color in ("blue", "bleu"):
        return "blue"
    if color in ("yellow", "jaune"):
        return "yellow"
    return "blue"


def read_default_team_color(default="blue"):
    try:
        camera_share = get_package_share_directory("camera_localization")
        camera_map = os.path.join(camera_share, "config", "camera_global_map.yaml")
        with open(camera_map, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        params = data.get("global_localization_node", {}).get("ros__parameters", {})
        return normalize_team_color(params.get("team_color", default))
    except (PackageNotFoundError, OSError, yaml.YAMLError):
        return normalize_team_color(default)
