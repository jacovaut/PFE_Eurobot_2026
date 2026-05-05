from dataclasses import dataclass

@dataclass
class XY:
    x: float
    y: float

@dataclass
class Block:
    name: str
    x: float
    y: float
    color: str = "unknown"
    yaw_deg: float = 0.0
    last_seen: float = 0.0
    raw_id: int = -1

@dataclass
class PickupCandidate:
    score: float
    picked_count: int
    avg_error: float
    yaw: float
    dx: float
    dy: float
    assignments: list
