"""Load mission.yaml and expose typed config objects."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml


@dataclass
class SearchConfig:
    mode: str = 'occupancy_grid'
    boundary_dist: float = 0.35
    cruise_speed: float = 0.18
    turn_speed: float = 0.45
    turn_angle_deg: float = 90.0
    wall_follow_duration: float = 2.0
    wall_follow_speed: float = 0.12
    max_boundary_events: int = 80
    front_angle: float = 3.14159
    front_sector_half_width: float = 0.35
    boundary_confirm_frames: int = 3
    align_tolerance_deg: float = 18.0
    nav_heading_kp: float = 2.0
    wall_target_dist: float = 0.25
    explore_stall_sec: float = 30.0
    explore_no_candidate_frames: int = 15
    nav_block_half_width: float = 0.14
    creep_speed: float = 0.06
    waypoint_tolerance: float = 0.10
    align_timeout_sec: float = 1.5
    align_arc_speed: float = 0.04
    nav_forward_stop_margin: float = 0.04
    nav_cruise_clear_margin: float = 0.08
    nav_wide_hold_margin: float = 0.08
    nav_lateral_block_dist: float = 0.25
    robot_radius: float = 0.15
    robot_half_width: float = 0.18
    robot_front_overhang: float = 0.08
    nav_passage_score_threshold: float = 0.40
    nav_passage_max_ticks: int = 80
    nav_passage_creep_factor: float = 0.90
    nav_passage_auto_arm: bool = True
    nav_passage_channel_auto_arm: bool = False
    nav_passage_wide_max: float = 0.48
    nav_passage_side_max: float = 0.65
    nav_passage_center_min_ratio: float = 0.80
    nav_passage_channel_close_max: float = 0.55
    nav_passage_channel_open_min: float = 0.75
    nav_passage_channel_center_min_ratio: float = 1.15
    nav_passage_channel_heading_max_deg: float = 35.0
    nav_passage_channel_min_hold_ticks: int = 15
    nav_passage_channel_lost_streak: int = 6
    nav_passage_channel_exit_yaw_hold_sec: float = 2.0
    nav_corridor_commit_travel_m: float = 0.80
    nav_corridor_commit_cooldown_sec: float = 2.0
    nav_corridor_commit_arm_confirm_frames: int = 10
    nav_corridor_commit_min_hold_ticks: int = 18
    nav_corridor_commit_min_travel_m: float = 0.30
    nav_corridor_commit_abort_yaw_deg: float = 45.0
    nav_corridor_commit_hard_abort_ratio: float = 0.55
    nav_open_align_center_min: float = 0.65
    nav_open_align_arc_speed: float = 0.10
    nav_wide_hold_override_center_min: float = 0.70
    nav_drive_heading_far_dist: float = 1.5
    nav_drive_heading_far_deg: float = 32.0
    nav_drive_heading_mid_dist: float = 0.9
    nav_drive_heading_mid_deg: float = 28.0
    recovery_passage_replan_angle_deg: float = 70.0
    recovery_passage_replan_rounds: int = 2
    boundary_pause_sec: float = 0.25
    boundary_turn_deg: float = 15.0
    boundary_corner_turn_deg: float = 25.0
    boundary_repeat_turn_deg: float = 35.0
    boundary_backoff_m: float = 0.10
    boundary_spatial_radius: float = 0.30
    boundary_spatial_count: int = 3
    recovery_backoff_levels: Tuple[float, float, float] = (0.10, 0.18, 0.28)
    recovery_turn_levels: Tuple[float, float, float] = (25.0, 35.0, 55.0)
    recent_collision_penalty: float = 0.25
    recent_collision_penalty_deg: float = 20.0
    recent_collision_penalty_sec: float = 3.0
    recovery_success_center_gain: float = 0.05
    recovery_success_disp_m: float = 0.06
    recovery_max_escalation: int = 3
    recovery_max_level_retries: int = 2
    recovery_replan_max_angle_deg: float = 35.0
    recovery_replan_max_dist_m: float = 3.0
    recovery_replan_rounds: int = 3
    recovery_escape_angle_deg: float = 50.0
    recovery_escape_center_max: float = 0.35
    local_replan_interval_sec: float = 0.35
    collision_cost_increment: float = 1.0
    collision_cost_max: float = 8.0
    vision_at_boundary: bool = False
    bootstrap_local_plan: bool = True
    bootstrap_max_sec: float = 90.0
    bootstrap_min_coverage: float = 0.10
    bootstrap_max_boundary: int = 3
    bootstrap_max_consecutive_recovery: int = 3
    bootstrap_stall_coverage_delta: float = 0.03
    bootstrap_detour_dist: float = 0.90
    bootstrap_detour_deg: float = 35.0
    local_plan_dist_min: float = 0.4
    local_plan_dist_max: float = 3.0
    local_plan_progress_dist_cap: float = 2.0
    local_plan_clearance_tie_band: float = 0.06
    local_plan_angles: Tuple[float, ...] = (0.0, 20.0, 35.0, 50.0, 70.0)
    stress_decay_tau: float = 0.8
    local_plan_score_clearance: float = 0.65
    local_plan_score_alignment: float = 0.07
    local_plan_score_explore: float = 0.22
    local_plan_score_progress: float = 0.06
    local_plan_score_curvature: float = 0.14
    local_plan_score_collision: float = 0.12
    local_plan_score_stress: float = 0.08
    local_plan_score_execution: float = 0.10
    failure_memory_penalty: float = 0.35
    failure_memory_angle_deg: float = 25.0
    failure_memory_radius_m: float = 0.45
    failure_memory_sec: float = 45.0
    failure_memory_max_entries: int = 8
    failure_memory_cluster_count: int = 3
    failure_memory_cluster_radius_m: float = 0.60
    failure_memory_cluster_penalty: float = 0.50
    wall_hug_zero_penalty: float = 0.25
    wall_hug_clearance_margin: float = 0.12
    wall_hug_angle_deg: float = 25.0
    planner_debug: bool = True
    stats_interval_sec: float = 60.0


@dataclass
class MapConfig:
    resolution: float = 0.08
    size_m: float = 12.0
    goal_tolerance: float = 0.03
    coverage_complete_threshold: float = 0.92
    use_odom: bool = True
    odom_topic: str = '/odom'
    return_home: bool = True
    align_yaw_on_return: bool = True
    bootstrap_visited: int = 40
    astar_unknown_cost: float = 2.0
    astar_inflation_cells: int = 1
    astar_waypoint_spacing: float = 0.20
    astar_max_candidates: int = 25


@dataclass
class BoustrophedonSearchConfig:
    strip_width: float = 0.30
    max_lanes: int = 30


@dataclass
class TargetSpec:
    name: str
    color: str
    shape: str
    count: int


@dataclass
class ColorSpec:
    lab_min: Tuple[int, int, int]
    lab_max: Tuple[int, int, int]


@dataclass
class ShapeSpec:
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class VisionConfig:
    proc_size: Tuple[int, int] = (320, 240)
    min_contour_area: float = 50.0
    confirm_area: float = 200.0
    debounce_frames: int = 3
    dedup_pixel_dist: float = 40.0
    dedup_world_dist: float = 0.25
    backend: str = 'classical'


@dataclass
class TrackingConfig:
    enabled: bool = False
    backend: str = 'stub'


@dataclass
class GlobalPlannerConfig:
    enabled: bool = True
    initial_mode: str = 'frontier'
    perimeter_enabled: bool = False
    perimeter_min_visited: int = 120
    perimeter_yaw_deg: float = 340.0
    perimeter_max_sec: float = 180.0
    perimeter_min_coverage: float = 0.12
    perimeter_wall_side: str = 'right'
    perimeter_lost_wall_dist: float = 0.80
    perimeter_found_wall_dist: float = 0.65
    perimeter_corner_stuck_ticks: int = 25


@dataclass
class MissionConfig:
    search: SearchConfig
    map: MapConfig
    global_planner: GlobalPlannerConfig
    boustrophedon: BoustrophedonSearchConfig
    targets: List[TargetSpec]
    colors: Dict[str, ColorSpec]
    shapes: Dict[str, ShapeSpec]
    vision: VisionConfig
    tracking: TrackingConfig

    @property
    def active_colors(self) -> List[str]:
        return sorted({t.color for t in self.targets})

    def required_counts(self) -> Dict[str, int]:
        return {'{}/{}'.format(t.color, t.shape): t.count for t in self.targets}


def _tuple3(values: List[int]) -> Tuple[int, int, int]:
    return int(values[0]), int(values[1]), int(values[2])


def load_mission_config(path: str) -> MissionConfig:
    with open(path, 'r', encoding='utf-8') as handle:
        raw = yaml.safe_load(handle)

    search_raw = raw.get('search', {})
    search = SearchConfig(
        mode=search_raw.get('mode', 'occupancy_grid'),
        boundary_dist=float(search_raw.get('boundary_dist', 0.2)),
        cruise_speed=float(search_raw.get('cruise_speed', 0.18)),
        turn_speed=float(search_raw.get('turn_speed', 0.45)),
        turn_angle_deg=float(search_raw.get('turn_angle_deg', 90)),
        wall_follow_duration=float(search_raw.get('wall_follow_duration', 2.0)),
        wall_follow_speed=float(search_raw.get('wall_follow_speed', 0.12)),
        max_boundary_events=int(search_raw.get('max_boundary_events', 80)),
        front_angle=float(search_raw.get('front_angle', 3.14159)),
        front_sector_half_width=float(search_raw.get('front_sector_half_width', 0.35)),
        boundary_confirm_frames=int(search_raw.get('boundary_confirm_frames', 3)),
        align_tolerance_deg=float(search_raw.get('align_tolerance_deg', 18.0)),
        nav_heading_kp=float(search_raw.get('nav_heading_kp', 2.0)),
        wall_target_dist=float(search_raw.get('wall_target_dist', 0.25)),
        explore_stall_sec=float(search_raw.get('explore_stall_sec', 30.0)),
        explore_no_candidate_frames=int(search_raw.get('explore_no_candidate_frames', 15)),
        nav_block_half_width=float(search_raw.get('nav_block_half_width', 0.14)),
        creep_speed=float(search_raw.get('creep_speed', 0.06)),
        waypoint_tolerance=float(search_raw.get('waypoint_tolerance', 0.10)),
        align_timeout_sec=float(search_raw.get('align_timeout_sec', 1.5)),
        align_arc_speed=float(search_raw.get('align_arc_speed', 0.04)),
        nav_forward_stop_margin=float(search_raw.get('nav_forward_stop_margin', 0.04)),
        nav_cruise_clear_margin=float(search_raw.get('nav_cruise_clear_margin', 0.08)),
        nav_wide_hold_margin=float(search_raw.get('nav_wide_hold_margin', 0.08)),
        nav_lateral_block_dist=float(search_raw.get('nav_lateral_block_dist', 0.25)),
        robot_radius=float(search_raw.get('robot_radius', 0.15)),
        robot_half_width=float(search_raw.get('robot_half_width', 0.18)),
        robot_front_overhang=float(search_raw.get('robot_front_overhang', 0.08)),
        nav_passage_score_threshold=float(
            search_raw.get('nav_passage_score_threshold', 0.45),
        ),
        nav_passage_max_ticks=int(search_raw.get('nav_passage_max_ticks', 80)),
        nav_passage_creep_factor=float(
            search_raw.get('nav_passage_creep_factor', 0.90),
        ),
        nav_passage_auto_arm=bool(search_raw.get('nav_passage_auto_arm', True)),
        nav_passage_channel_auto_arm=bool(
            search_raw.get('nav_passage_channel_auto_arm', False),
        ),
        nav_passage_wide_max=float(search_raw.get('nav_passage_wide_max', 0.48)),
        nav_passage_side_max=float(search_raw.get('nav_passage_side_max', 0.65)),
        nav_passage_center_min_ratio=float(
            search_raw.get('nav_passage_center_min_ratio', 0.80),
        ),
        nav_passage_channel_close_max=float(
            search_raw.get('nav_passage_channel_close_max', 0.55),
        ),
        nav_passage_channel_open_min=float(
            search_raw.get('nav_passage_channel_open_min', 0.75),
        ),
        nav_passage_channel_center_min_ratio=float(
            search_raw.get('nav_passage_channel_center_min_ratio', 1.15),
        ),
        nav_passage_channel_heading_max_deg=float(
            search_raw.get('nav_passage_channel_heading_max_deg', 35.0),
        ),
        nav_passage_channel_min_hold_ticks=int(
            search_raw.get('nav_passage_channel_min_hold_ticks', 15),
        ),
        nav_passage_channel_lost_streak=int(
            search_raw.get('nav_passage_channel_lost_streak', 6),
        ),
        nav_passage_channel_exit_yaw_hold_sec=float(
            search_raw.get('nav_passage_channel_exit_yaw_hold_sec', 2.0),
        ),
        nav_corridor_commit_travel_m=float(
            search_raw.get('nav_corridor_commit_travel_m', 0.80),
        ),
        nav_corridor_commit_cooldown_sec=float(
            search_raw.get('nav_corridor_commit_cooldown_sec', 2.0),
        ),
        nav_corridor_commit_arm_confirm_frames=int(
            search_raw.get('nav_corridor_commit_arm_confirm_frames', 10),
        ),
        nav_corridor_commit_min_hold_ticks=int(
            search_raw.get('nav_corridor_commit_min_hold_ticks', 18),
        ),
        nav_corridor_commit_min_travel_m=float(
            search_raw.get('nav_corridor_commit_min_travel_m', 0.30),
        ),
        nav_corridor_commit_abort_yaw_deg=float(
            search_raw.get('nav_corridor_commit_abort_yaw_deg', 45.0),
        ),
        nav_corridor_commit_hard_abort_ratio=float(
            search_raw.get('nav_corridor_commit_hard_abort_ratio', 0.55),
        ),
        nav_open_align_center_min=float(
            search_raw.get('nav_open_align_center_min', 0.65),
        ),
        nav_open_align_arc_speed=float(
            search_raw.get('nav_open_align_arc_speed', 0.10),
        ),
        nav_wide_hold_override_center_min=float(
            search_raw.get('nav_wide_hold_override_center_min', 0.70),
        ),
        nav_drive_heading_far_dist=float(
            search_raw.get('nav_drive_heading_far_dist', 1.5),
        ),
        nav_drive_heading_far_deg=float(
            search_raw.get('nav_drive_heading_far_deg', 32.0),
        ),
        nav_drive_heading_mid_dist=float(
            search_raw.get('nav_drive_heading_mid_dist', 0.9),
        ),
        nav_drive_heading_mid_deg=float(
            search_raw.get('nav_drive_heading_mid_deg', 28.0),
        ),
        recovery_passage_replan_angle_deg=float(
            search_raw.get('recovery_passage_replan_angle_deg', 70.0),
        ),
        recovery_passage_replan_rounds=int(
            search_raw.get('recovery_passage_replan_rounds', 2),
        ),
        boundary_pause_sec=float(search_raw.get('boundary_pause_sec', 0.25)),
        boundary_turn_deg=float(search_raw.get('boundary_turn_deg', 15.0)),
        boundary_corner_turn_deg=float(search_raw.get('boundary_corner_turn_deg', 25.0)),
        boundary_repeat_turn_deg=float(search_raw.get('boundary_repeat_turn_deg', 35.0)),
        boundary_backoff_m=float(search_raw.get('boundary_backoff_m', 0.10)),
        boundary_spatial_radius=float(search_raw.get('boundary_spatial_radius', 0.30)),
        boundary_spatial_count=int(search_raw.get('boundary_spatial_count', 3)),
        recovery_backoff_levels=tuple(
            float(v) for v in search_raw.get('recovery_backoff_levels', [0.10, 0.18, 0.28])
        ),
        recovery_turn_levels=tuple(
            float(v) for v in search_raw.get('recovery_turn_levels', [25, 35, 55])
        ),
        recent_collision_penalty=float(search_raw.get('recent_collision_penalty', 0.25)),
        recent_collision_penalty_deg=float(
            search_raw.get('recent_collision_penalty_deg', 20.0),
        ),
        recent_collision_penalty_sec=float(
            search_raw.get('recent_collision_penalty_sec', 3.0),
        ),
        recovery_success_center_gain=float(search_raw.get('recovery_success_center_gain', 0.05)),
        recovery_success_disp_m=float(search_raw.get('recovery_success_disp_m', 0.06)),
        recovery_max_escalation=int(search_raw.get('recovery_max_escalation', 3)),
        recovery_max_level_retries=int(search_raw.get('recovery_max_level_retries', 2)),
        recovery_replan_max_angle_deg=float(
            search_raw.get('recovery_replan_max_angle_deg', 35.0),
        ),
        recovery_replan_max_dist_m=float(
            search_raw.get('recovery_replan_max_dist_m', 2.0),
        ),
        recovery_replan_rounds=int(search_raw.get('recovery_replan_rounds', 3)),
        recovery_escape_angle_deg=float(search_raw.get('recovery_escape_angle_deg', 50.0)),
        recovery_escape_center_max=float(search_raw.get('recovery_escape_center_max', 0.35)),
        local_replan_interval_sec=float(search_raw.get('local_replan_interval_sec', 0.35)),
        collision_cost_increment=float(search_raw.get('collision_cost_increment', 1.0)),
        collision_cost_max=float(search_raw.get('collision_cost_max', 8.0)),
        vision_at_boundary=bool(search_raw.get('vision_at_boundary', False)),
        bootstrap_local_plan=bool(search_raw.get('bootstrap_local_plan', True)),
        bootstrap_max_sec=float(search_raw.get('bootstrap_max_sec', 60.0)),
        bootstrap_min_coverage=float(search_raw.get('bootstrap_min_coverage', 0.10)),
        bootstrap_max_boundary=int(search_raw.get('bootstrap_max_boundary', 3)),
        bootstrap_max_consecutive_recovery=int(
            search_raw.get('bootstrap_max_consecutive_recovery', 3),
        ),
        bootstrap_stall_coverage_delta=float(
            search_raw.get('bootstrap_stall_coverage_delta', 0.03),
        ),
        bootstrap_detour_dist=float(search_raw.get('bootstrap_detour_dist', 0.90)),
        bootstrap_detour_deg=float(search_raw.get('bootstrap_detour_deg', 35.0)),
        local_plan_dist_min=float(search_raw.get('local_plan_dist_min', 0.4)),
        local_plan_dist_max=float(
            search_raw.get(
                'local_plan_dist_max',
                search_raw.get('local_plan_dist', 3.0),
            )
        ),
        local_plan_progress_dist_cap=float(
            search_raw.get('local_plan_progress_dist_cap', 2.0),
        ),
        local_plan_clearance_tie_band=float(
            search_raw.get('local_plan_clearance_tie_band', 0.06),
        ),
        local_plan_angles=tuple(
            float(v) for v in search_raw.get('local_plan_angles', [0, 20, 35, 50, 70])
        ),
        stress_decay_tau=float(search_raw.get('stress_decay_tau', 0.8)),
        local_plan_score_clearance=float(
            search_raw.get('local_plan_score_clearance', 0.58),
        ),
        local_plan_score_alignment=float(
            search_raw.get('local_plan_score_alignment', 0.10),
        ),
        local_plan_score_explore=float(search_raw.get('local_plan_score_explore', 0.22)),
        local_plan_score_progress=float(search_raw.get('local_plan_score_progress', 0.06)),
        local_plan_score_curvature=float(
            search_raw.get('local_plan_score_curvature', 0.14),
        ),
        local_plan_score_collision=float(
            search_raw.get('local_plan_score_collision', 0.12),
        ),
        local_plan_score_stress=float(search_raw.get('local_plan_score_stress', 0.08)),
        local_plan_score_execution=float(
            search_raw.get('local_plan_score_execution', 0.10),
        ),
        failure_memory_penalty=float(search_raw.get('failure_memory_penalty', 0.35)),
        failure_memory_angle_deg=float(search_raw.get('failure_memory_angle_deg', 25.0)),
        failure_memory_radius_m=float(search_raw.get('failure_memory_radius_m', 0.45)),
        failure_memory_sec=float(search_raw.get('failure_memory_sec', 45.0)),
        failure_memory_max_entries=int(search_raw.get('failure_memory_max_entries', 8)),
        failure_memory_cluster_count=int(
            search_raw.get('failure_memory_cluster_count', 3),
        ),
        failure_memory_cluster_radius_m=float(
            search_raw.get('failure_memory_cluster_radius_m', 0.60),
        ),
        failure_memory_cluster_penalty=float(
            search_raw.get('failure_memory_cluster_penalty', 0.50),
        ),
        wall_hug_zero_penalty=float(search_raw.get('wall_hug_zero_penalty', 0.25)),
        wall_hug_clearance_margin=float(
            search_raw.get('wall_hug_clearance_margin', 0.12),
        ),
        wall_hug_angle_deg=float(search_raw.get('wall_hug_angle_deg', 25.0)),
        planner_debug=bool(search_raw.get('planner_debug', True)),
        stats_interval_sec=float(search_raw.get('stats_interval_sec', 60.0)),
    )

    map_raw = raw.get('map', {})
    map_cfg = MapConfig(
        resolution=float(map_raw.get('resolution', 0.08)),
        size_m=float(map_raw.get('size_m', 8.0)),
        goal_tolerance=float(map_raw.get('goal_tolerance', 0.15)),
        coverage_complete_threshold=float(map_raw.get('coverage_complete_threshold', 0.92)),
        use_odom=bool(map_raw.get('use_odom', True)),
        odom_topic=map_raw.get('odom_topic', '/odom'),
        return_home=bool(map_raw.get('return_home', True)),
        align_yaw_on_return=bool(map_raw.get('align_yaw_on_return', True)),
        bootstrap_visited=int(map_raw.get('bootstrap_visited', 40)),
        astar_unknown_cost=float(map_raw.get('astar_unknown_cost', 2.0)),
        astar_inflation_cells=int(map_raw.get('astar_inflation_cells', 1)),
        astar_waypoint_spacing=float(map_raw.get('astar_waypoint_spacing', 0.20)),
        astar_max_candidates=int(map_raw.get('astar_max_candidates', 25)),
    )

    bous_raw = raw.get('boustrophedon', {})
    boustrophedon = BoustrophedonSearchConfig(
        strip_width=float(bous_raw.get('strip_width', 0.30)),
        max_lanes=int(bous_raw.get('max_lanes', 30)),
    )

    gp_raw = raw.get('global_planner', {})
    global_planner = GlobalPlannerConfig(
        enabled=bool(gp_raw.get('enabled', True)),
        initial_mode=str(gp_raw.get('initial_mode', 'frontier')),
        perimeter_enabled=bool(gp_raw.get('perimeter_enabled', False)),
        perimeter_min_visited=int(gp_raw.get('perimeter_min_visited', 120)),
        perimeter_yaw_deg=float(gp_raw.get('perimeter_yaw_deg', 340.0)),
        perimeter_max_sec=float(gp_raw.get('perimeter_max_sec', 180.0)),
        perimeter_min_coverage=float(gp_raw.get('perimeter_min_coverage', 0.12)),
        perimeter_wall_side=str(gp_raw.get('perimeter_wall_side', 'right')),
        perimeter_lost_wall_dist=float(gp_raw.get('perimeter_lost_wall_dist', 0.80)),
        perimeter_found_wall_dist=float(gp_raw.get('perimeter_found_wall_dist', 0.65)),
        perimeter_corner_stuck_ticks=int(gp_raw.get('perimeter_corner_stuck_ticks', 25)),
    )

    targets = [
        TargetSpec(
            name=t.get('name', 'target_{}'.format(i)),
            color=t['color'],
            shape=t['shape'],
            count=int(t['count']),
        )
        for i, t in enumerate(raw.get('targets', []))
    ]

    colors: Dict[str, ColorSpec] = {}
    for name, spec in raw.get('colors', {}).items():
        colors[name] = ColorSpec(
            lab_min=_tuple3(spec['lab_min']),
            lab_max=_tuple3(spec['lab_max']),
        )

    shapes: Dict[str, ShapeSpec] = {}
    for name, spec in raw.get('shapes', {}).items():
        shapes[name] = ShapeSpec(params=dict(spec))

    vision_raw = raw.get('vision', {})
    proc = vision_raw.get('proc_size', [320, 240])
    vision = VisionConfig(
        proc_size=(int(proc[0]), int(proc[1])),
        min_contour_area=float(vision_raw.get('min_contour_area', 50)),
        confirm_area=float(vision_raw.get('confirm_area', 200)),
        debounce_frames=int(vision_raw.get('debounce_frames', 3)),
        dedup_pixel_dist=float(vision_raw.get('dedup_pixel_dist', 40)),
        dedup_world_dist=float(vision_raw.get('dedup_world_dist', 0.25)),
        backend=vision_raw.get('backend', 'classical'),
    )

    tracking_raw = raw.get('tracking', {})
    tracking = TrackingConfig(
        enabled=bool(tracking_raw.get('enabled', False)),
        backend=tracking_raw.get('backend', 'stub'),
    )

    return MissionConfig(
        search=search,
        map=map_cfg,
        global_planner=global_planner,
        boustrophedon=boustrophedon,
        targets=targets,
        colors=colors,
        shapes=shapes,
        vision=vision,
        tracking=tracking,
    )


def resolve_config_path(param_path: Optional[str] = None) -> str:
    if param_path and os.path.isfile(param_path):
        return param_path
    raise FileNotFoundError('Mission config not found: {}'.format(param_path))
