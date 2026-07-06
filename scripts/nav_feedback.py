"""Navigator → Planner contract: continuous execution cost and goal status."""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Dict


class GoalStatus(enum.Enum):
  RUNNING = 'RUNNING'
  REACHED = 'REACHED'
  FAILED = 'FAILED'
  ABORTED = 'ABORTED'
  IDLE = 'IDLE'


@dataclass
class NavExecutionFeedback:
  goal_status: GoalStatus = GoalStatus.IDLE
  execution_cost: float = 0.0
  reason: str = 'none'
  metrics: Dict[str, float] = field(default_factory=dict)

  def to_log_line(self) -> str:
    return (
        'NavFeedback: status={} exec_cost={:.2f} reason={} '
        'center={:.2f} wide={:.2f} stress={:.0f}'.format(
            self.goal_status.value,
            self.execution_cost,
            self.reason,
            self.metrics.get('center', 0.0),
            self.metrics.get('wide', 0.0),
            self.metrics.get('stress_level', 0.0),
        )
    )
