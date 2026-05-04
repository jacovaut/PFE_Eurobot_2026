import math
from .data_structures import Block

class BlockTracker:
    def __init__(self, block_timeout_sec, match_distance_m):
        self.tracked_blocks = {}
        self.next_index_by_color = {"blue": 0, "yellow": 0, "unknown": 0}
        self.block_timeout_sec = block_timeout_sec
        self.match_distance_m = match_distance_m

    def make_name(self, color):
        idx = self.next_index_by_color[color]
        self.next_index_by_color[color] += 1
        return f"{color}_{idx}"

    def update_tracking(self, detections, now):
        updated = set()
        for d in detections:
            best_name = None
            best_dist = None
            for name, t in self.tracked_blocks.items():
                if t.color != d.color or name in updated:
                    continue
                dist = math.hypot(d.x - t.x, d.y - t.y)
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_name = name
            if best_name is not None and best_dist < self.match_distance_m:
                t = self.tracked_blocks[best_name]
                t.x = d.x
                t.y = d.y
                t.yaw_deg = d.yaw_deg
                t.raw_id = d.raw_id
                t.last_seen = now
                updated.add(best_name)
            else:
                name = self.make_name(d.color)
                self.tracked_blocks[name] = Block(
                    name=name,
                    x=d.x,
                    y=d.y,
                    color=d.color,
                    yaw_deg=d.yaw_deg,
                    last_seen=now,
                    raw_id=d.raw_id
                )
                updated.add(name)
        self.cleanup_stale_blocks(now)
        return self.tracked_blocks

    def cleanup_stale_blocks(self, now):
        to_del = []
        for name, b in self.tracked_blocks.items():
            if now - b.last_seen > self.block_timeout_sec:
                to_del.append(name)
        for name in to_del:
            del self.tracked_blocks[name]
