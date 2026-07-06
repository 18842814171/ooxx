"""LiDAR helpers — extend demo.py index pattern to multi-sector reads."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from sensor_msgs.msg import LaserScan


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def range_at_angle(msg: LaserScan, angle: float, window: int = 2) -> float:
    """Return minimum valid range near *angle* (rad) in scan frame."""
    if not msg.ranges:
        return float('inf')

    target = _normalize_angle(angle)
    best = float('inf')
    for idx, rng in enumerate(msg.ranges):
        beam = msg.angle_min + idx * msg.angle_increment
        beam = _normalize_angle(beam)
        if abs(beam - target) > msg.angle_increment * max(window, 1):
            continue
        if msg.range_min <= rng <= msg.range_max:
            best = min(best, rng)
    return best


def front_distance(
    msg: LaserScan,
    front_angle: float = math.pi,
    half_width: float = 0.35,
) -> float:
    """Forward clearance; uses sector minimum, not a single beam."""
    return sector_min(msg, front_angle, half_width)


def sector_min(msg: LaserScan, center_angle: float, half_width: float = 0.35) -> float:
    """Minimum range within [center - half_width, center + half_width]."""
    if not msg.ranges:
        return float('inf')

    best = float('inf')
    lo = _normalize_angle(center_angle - half_width)
    hi = _normalize_angle(center_angle + half_width)

    for idx, rng in enumerate(msg.ranges):
        if not (msg.range_min <= rng <= msg.range_max):
            continue
        beam = _normalize_angle(msg.angle_min + idx * msg.angle_increment)
        if lo <= hi:
            in_sector = lo <= beam <= hi
        else:
            in_sector = beam >= lo or beam <= hi
        if in_sector:
            best = min(best, rng)
    return best


def sector_mean(msg: LaserScan, center_angle: float, half_width: float = 0.35) -> float:
    """Mean valid range within a sector; returns inf when no valid samples."""
    if not msg.ranges:
        return float('inf')

    lo = _normalize_angle(center_angle - half_width)
    hi = _normalize_angle(center_angle + half_width)
    total = 0.0
    count = 0

    for idx, rng in enumerate(msg.ranges):
        if not (msg.range_min <= rng <= msg.range_max):
            continue
        beam = _normalize_angle(msg.angle_min + idx * msg.angle_increment)
        if lo <= hi:
            in_sector = lo <= beam <= hi
        else:
            in_sector = beam >= lo or beam <= hi
        if in_sector:
            total += rng
            count += 1

    if count == 0:
        return float('inf')
    return total / count


def left_distance(msg: LaserScan, front_angle: float = math.pi) -> float:
    return sector_min(msg, _normalize_angle(front_angle + math.pi / 2))


def right_distance(msg: LaserScan, front_angle: float = math.pi) -> float:
    return sector_min(msg, _normalize_angle(front_angle - math.pi / 2))


def is_boundary(
    msg: LaserScan,
    boundary_dist: float,
    front_angle: float = math.pi,
    half_width: float = 0.35,
) -> bool:
    """True when forward-sector clearance is below threshold."""
    return sector_min(msg, front_angle, half_width) < boundary_dist


def open_side(
    msg: LaserScan,
    front_angle: float = math.pi,
    tie_flip: Optional[str] = None,
) -> str:
    """Return 'left' or 'right' for the side with more clearance."""
    left = left_distance(msg, front_angle)
    right = right_distance(msg, front_angle)
    if abs(left - right) < 0.05:
        if tie_flip == 'right':
            return 'right'
        if tie_flip == 'left':
            return 'left'
        front_left = sector_min(
            msg, _normalize_angle(front_angle + math.pi / 4), 0.22,
        )
        front_right = sector_min(
            msg, _normalize_angle(front_angle - math.pi / 4), 0.22,
        )
        return 'left' if front_left >= front_right else 'right'
    return 'left' if left >= right else 'right'


def passage_score(
    msg: LaserScan,
    front_angle: float,
    nav_half_width: float,
    wide_half_width: float,
    boundary_dist: float,
) -> float:
    """Continuous 0..1 score — higher means likely passable gap (wide tight, center open)."""
    center = sector_min(msg, front_angle, nav_half_width)
    wide = sector_min(msg, front_angle, wide_half_width)
    left = left_distance(msg, front_angle)
    right = right_distance(msg, front_angle)

    if center == float('inf') and wide == float('inf'):
        return 0.0

    score = 0.0
    if wide < boundary_dist and center > boundary_dist * 0.85:
        center_margin = min(1.0, (center - boundary_dist * 0.85) / 0.15)
        wide_gap = min(1.0, max(0.0, (center - wide) / 0.12))
        score = 0.45 + 0.35 * center_margin + 0.20 * wide_gap

    if left < 0.55 and right < 0.55:
        symmetry = 1.0 - min(1.0, abs(left - right) / 0.25)
        side_score = 0.35 * symmetry
        if center > wide:
            side_score += 0.15
        score = max(score, side_score)

    if left < 0.35 and right < 0.35 and center < boundary_dist:
        score = max(score, 0.85)

    # 块+墙单通道：一侧近、一侧远，前方可蠕行（Issue-1 第二个缝）
    near_left = left < 0.55
    near_right = right < 0.55
    if (near_left ^ near_right) and center >= boundary_dist * 1.10:
        close_side = min(left, right)
        open_side = max(left, right)
        if close_side < 0.55 and open_side >= 0.75:
            side_fit = 1.0 - min(1.0, close_side / 0.55)
            forward = min(1.0, (center - boundary_dist * 1.05) / 0.18)
            channel_score = 0.44 + 0.28 * side_fit + 0.28 * forward
            score = max(score, channel_score)

    return min(1.0, max(0.0, score))


def interpret_obstacle(
    msg: LaserScan,
    front_angle: float,
    nav_half_width: float,
    wide_half_width: float,
    boundary_dist: float,
) -> str:
    """Classify why forward motion is constrained — no FSM state added."""
    center = sector_min(msg, front_angle, nav_half_width)
    wide = sector_min(msg, front_angle, wide_half_width)
    left = left_distance(msg, front_angle)
    right = right_distance(msg, front_angle)

    if center >= boundary_dist:
        return 'clear'

    if (
        center < boundary_dist
        and wide < boundary_dist
        and left < 0.40
        and right < 0.40
    ):
        return 'dead_end'

    if wide < boundary_dist and center >= boundary_dist * 0.88:
        return 'flanked'

    near_left = left < 0.38
    near_right = right < 0.38
    if center < boundary_dist and (near_left ^ near_right):
        return 'single_wall'

    if center < boundary_dist:
        return 'front_blocked'

    return 'clear'


def classify_boundary_type(
    msg: LaserScan,
    front_angle: float,
    nav_half_width: float,
    wide_half_width: float,
    boundary_dist: float,
) -> str:
    """Classify boundary encounter for recovery action selection."""
    obs = interpret_obstacle(
        msg, front_angle, nav_half_width, wide_half_width, boundary_dist,
    )
    if obs == 'dead_end':
        return 'corner'
    if obs == 'flanked':
        return 'narrow_passage'
    if obs == 'single_wall':
        return 'side_wall'
    return 'front_wall'


def forward_path_clear(
    msg: LaserScan,
    front_angle: float,
    nav_half_width: float,
    wide_half_width: float,
    boundary_dist: float,
    margin: float = 0.06,
) -> bool:
    """True when both narrow and wide forward sectors have enough clearance."""
    center = sector_min(msg, front_angle, nav_half_width)
    wide = sector_min(msg, front_angle, wide_half_width)
    limit = boundary_dist + margin
    return center >= limit and wide >= limit


def front_sector_clear(
    msg: LaserScan,
    front_angle: float,
    nav_half_width: float,
    boundary_dist: float,
    margin: float = 0.04,
) -> bool:
    """True when the narrow forward sector has enough clearance to drive."""
    center = sector_min(msg, front_angle, nav_half_width)
    return center >= boundary_dist + margin


def clearance_profile(
    msg: LaserScan,
    front_angle: float,
    nav_half_width: float,
    wide_half_width: float,
    boundary_dist: float,
) -> Dict[str, Any]:
    """Snapshot for navigation logging and decisions."""
    center = sector_min(msg, front_angle, nav_half_width)
    wide = sector_min(msg, front_angle, wide_half_width)
    left = left_distance(msg, front_angle)
    right = right_distance(msg, front_angle)
    return {
        'center': center,
        'wide': wide,
        'left': left,
        'right': right,
        'passage_score': passage_score(
            msg, front_angle, nav_half_width, wide_half_width, boundary_dist,
        ),
        'obstacle': interpret_obstacle(
            msg, front_angle, nav_half_width, wide_half_width, boundary_dist,
        ),
    }
