"""Systematic boustrophedon (lawn-mower) strip coverage."""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Optional

from move_controller import MoveController


class StripPhase(enum.Enum):
  DRIVE_STRIP = 'DRIVE_STRIP'
  BOUNDARY_STOP = 'BOUNDARY_STOP'
  SHIFT_LANE = 'SHIFT_LANE'
  DONE = 'DONE'


@dataclass
class BoustrophedonConfig:
  strip_width: float = 0.30
  cruise_speed: float = 0.18
  turn_speed: float = 0.45
  max_lanes: int = 30


class BoustrophedonPlanner:
  """Drive alternating strips: >>>>>>> then <<<<<<<."""

  def __init__(self, move: MoveController, config: BoustrophedonConfig):
    self.move = move
    self.config = config
    self.phase = StripPhase.DRIVE_STRIP
    self.lane_index = 0
    self.forward = True  # True = +X body, False = -X body

  def on_boundary(self) -> None:
    self.phase = StripPhase.BOUNDARY_STOP

  def step_boundary_stop(self) -> StripPhase:
    self.phase = StripPhase.SHIFT_LANE
    return self.phase

  def step_shift_lane(self) -> StripPhase:
    if self.lane_index >= self.config.max_lanes:
      self.phase = StripPhase.DONE
      return self.phase

    # Turn 90°, lateral shift one strip width, turn 90° to face reverse lane.
    sign = 1.0 if self.lane_index % 2 == 0 else -1.0
    self.move.rotate_angle(90.0 * sign, self.config.turn_speed)
    self.move.move_distance_y(self.config.strip_width * sign, self.config.cruise_speed * 0.6)
    self.move.rotate_angle(90.0 * sign, self.config.turn_speed)

    self.forward = not self.forward
    self.lane_index += 1
    self.phase = StripPhase.DRIVE_STRIP
    return self.phase

  def cruise_speed_signed(self) -> float:
    return self.config.cruise_speed if self.forward else -self.config.cruise_speed

  def is_done(self) -> bool:
    return self.phase == StripPhase.DONE

  def status(self) -> str:
    return 'boustrophedon lane={} forward={} phase={}'.format(
        self.lane_index, self.forward, self.phase.value
    )
