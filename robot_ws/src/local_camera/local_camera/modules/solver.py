import math
from itertools import combinations, permutations
from .data_structures import PickupCandidate
from .math_utils import rotate_xy, angle_between, wrap_angle, rectangular_yaw_diff_deg

def solve_pose(cup_subset, block_subset):
    n = len(cup_subset)
    if n == 1:
        yaw = 0.0
    else:
        (_, c1), (_, c2) = cup_subset[0], cup_subset[-1]
        (_, b1), (_, b2) = block_subset[0], block_subset[-1]
        yaw = wrap_angle(angle_between(b1, b2) - angle_between(c1, c2))
    dx = dy = 0.0
    for (_, cxy), (_, b) in zip(cup_subset, block_subset):
        rc = rotate_xy(cxy, yaw)
        dx += b.x - rc.x
        dy += b.y - rc.y
    dx /= n
    dy /= n
    return dx, dy, yaw

def compute_best_pickup(cups, blocks, team_color="blue"):
    MAX_ERROR = 0.040
    MAX_YAW = math.radians(105)
    MAX_BLOCK_YAW_DIFF_DEG = 20.0
    W_BLOCKS = 3000.0
    W_ERROR = 4000.0
    W_YAW = 10.0
    W_COLOR = 100.0
    W_BLOCK_PARALLEL = 25.0
    cup_items = list(cups.items())
    block_items = [(n, b) for n, b in blocks.items() if b.color in ("blue", "yellow")]
    best = None
    for n in range(1, min(len(cup_items), len(block_items)) + 1):
        for cup_subset in combinations(cup_items, n):
            if n == 1 and cup_subset[0][0] != "cup_3":
                continue
            for block_subset in combinations(block_items, n):
                yaw_spread = 0.0
                if n > 1:
                    ref = block_subset[0][1].yaw_deg
                    valid_parallel = True
                    for _, b in block_subset[1:]:
                        diff = rectangular_yaw_diff_deg(ref, b.yaw_deg)
                        yaw_spread = max(yaw_spread, diff)
                        if diff > MAX_BLOCK_YAW_DIFF_DEG:
                            valid_parallel = False
                            break
                    if not valid_parallel:
                        continue
                for perm in permutations(block_subset):
                    if n == 1:
                        yaw = 0.0
                    else:
                        (_, c1), (_, c2) = cup_subset[0], cup_subset[-1]
                        (_, b1), (_, b2) = perm[0], perm[-1]
                        yaw = wrap_angle(angle_between(b1, b2) - angle_between(c1, c2))
                    if abs(yaw) > MAX_YAW:
                        continue
                    dx = dy = 0.0
                    for (cn, cxy), (bn, b) in zip(cup_subset, perm):
                        rc = rotate_xy(cxy, yaw)
                        dx += b.x - rc.x
                        dy += b.y - rc.y
                    dx /= n
                    dy /= n
                    total_err = 0.0
                    color_score = 0.0
                    valid = True
                    assigns = []
                    for (cn, cxy), (bn, b) in zip(cup_subset, perm):
                        rc = rotate_xy(cxy, yaw)
                        px = rc.x + dx
                        py = rc.y + dy
                        err = math.hypot(px - b.x, py - b.y)
                        if err > MAX_ERROR:
                            valid = False
                            break
                        if b.color == team_color:
                            color_score += 1
                        elif b.color != "unknown":
                            color_score -= 0.5
                        total_err += err
                        assigns.append({
                            "cup": cn,
                            "block": b.name,
                            "color": b.color,
                            "err_m": err,
                            "err_mm": err * 1000,
                            "raw_id": b.raw_id,
                            "block_yaw_deg": b.yaw_deg
                        })
                    if not valid:
                        continue
                    avg_err = total_err / n
                    score = (
                        W_BLOCKS * n
                        - W_ERROR * avg_err
                        - W_YAW * abs(math.degrees(yaw))
                        + W_COLOR * color_score
                        - W_BLOCK_PARALLEL * yaw_spread
                    )
                    cand = PickupCandidate(
                        score, n, avg_err, yaw, dx, dy, assigns
                    )
                    if best is None or cand.score > best.score:
                        best = cand
    return best

def recompute_locked_pose(cups, blocks, locked_assignments):
    if not locked_assignments:
        return None
    cup_subset = []
    block_subset = []
    for a in locked_assignments:
        cup_name = a["cup"]
        block_name = a["block"]
        if cup_name not in cups or block_name not in blocks:
            return None
        cup_subset.append((cup_name, cups[cup_name]))
        block_subset.append((block_name, blocks[block_name]))
    dx, dy, yaw = solve_pose(cup_subset, block_subset)
    total_err = 0.0
    new_assignments = []
    for (cn, cxy), (_, b) in zip(cup_subset, block_subset):
        rc = rotate_xy(cxy, yaw)
        px = rc.x + dx
        py = rc.y + dy
        err = math.hypot(px - b.x, py - b.y)
        total_err += err
        new_assignments.append({
            "cup": cn,
            "block": b.name,
            "color": b.color,
            "err_m": err,
            "err_mm": err * 1000.0,
            "raw_id": b.raw_id,
            "block_yaw_deg": b.yaw_deg
        })
    avg_err = total_err / len(locked_assignments)
    return PickupCandidate(
        score=0.0,
        picked_count=len(locked_assignments),
        avg_error=avg_err,
        yaw=yaw,
        dx=dx,
        dy=dy,
        assignments=new_assignments
    )
