"""Project RGB pixel + depth to camera-frame 3D (for world dedup)."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


def depth_at_pixel(depth_image: np.ndarray, u: int, v: int, window: int = 3) -> Optional[float]:
  if depth_image is None:
    return None
  h, w = depth_image.shape[:2]
  u = max(0, min(w - 1, int(u)))
  v = max(0, min(h - 1, int(v)))
  half = max(1, window // 2)
  patch = depth_image[
      max(0, v - half):min(h, v + half + 1),
      max(0, u - half):min(w, u + half + 1),
  ]
  valid = patch[(patch > 0) & np.isfinite(patch)]
  if valid.size == 0:
    return None
  return float(np.median(valid))


def pixel_to_camera(
    u: int,
    v: int,
    depth_mm: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float = 0.001,
) -> Tuple[float, float, float]:
  z = depth_mm * depth_scale
  x = (u - cx) * z / fx
  y = (v - cy) * z / fy
  return x, y, z


def pixel_to_world(
    u: int,
    v: int,
    depth_image,
    camera_info,
    robot_pose: Tuple[float, float, float],
) -> Optional[Tuple[float, float, float]]:
  """Map image pixel to approximate world XY (robot odom frame)."""
  if depth_image is None or camera_info is None:
    return None

  depth_mm = depth_at_pixel(depth_image, u, v)
  if depth_mm is None or depth_mm <= 0:
    return None

  k = camera_info.K
  fx, fy, cx, cy = k[0], k[4], k[2], k[5]
  cam_x, cam_y, cam_z = pixel_to_camera(u, v, depth_mm, fx, fy, cx, cy)

  # Camera mounted forward on robot: optical Z forward, X right, Y down.
  robot_x, robot_y, yaw = robot_pose
  world_x = robot_x + cam_z * math.cos(yaw) - cam_x * math.sin(yaw)
  world_y = robot_y + cam_z * math.sin(yaw) + cam_x * math.cos(yaw)
  world_z = cam_y
  return world_x, world_y, world_z
