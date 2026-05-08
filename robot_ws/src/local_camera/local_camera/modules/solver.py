import math
from itertools import combinations, permutations

from .data_structures import PickupCandidate
from .math_utils import (
    rotate_xy,
    angle_between,
    wrap_angle,
    rectangular_yaw_diff_deg,
)


def solve_pose(cup_subset, block_subset):
    """
    Compute rigid transform (dx, dy, yaw)
    aligning cup geometry onto chosen blocks.
    """
    n = len(cup_subset)

    if n == 1:
        (_, cxy) = cup_subset[0]
        (_, b) = block_subset[0]

        # Use block yaw as best guess for single-point alignment
        yaw = math.radians(getattr(b, "yaw_deg", 0.0))

        rc = rotate_xy(cxy, yaw)

        dx = b.x - rc.x
        dy = b.y - rc.y

        return dx, dy, yaw

    # Use first and last point to compute orientation
    (_, c1), (_, c2) = cup_subset[0], cup_subset[-1]
    (_, b1), (_, b2) = block_subset[0], block_subset[-1]

    yaw = wrap_angle(
        angle_between(b1, b2) - angle_between(c1, c2)
    )

    # Average translation
    dx = 0.0
    dy = 0.0

    for (_, cxy), (_, b) in zip(cup_subset, block_subset):
        rc = rotate_xy(cxy, yaw)

        dx += (b.x - rc.x)
        dy += (b.y - rc.y)

    dx /= n
    dy /= n

    return dx, dy, yaw


def point_line_distance(px, py, ax, ay, bx, by):
    """
    Distance from point P to line AB.
    """
    abx = bx - ax
    aby = by - ay

    denom = math.hypot(abx, aby)

    if denom < 1e-9:
        return math.hypot(px - ax, py - ay)

    return abs(
        aby * px - abx * py + bx * ay - by * ax
    ) / denom


def sort_blocks_along_line(block_subset):
    """
    Sort blocks along their own line direction.
    This is orientation-independent.
    Works for vertical, horizontal, diagonal.
    """

    if len(block_subset) <= 1:
        return tuple(block_subset)

    best_pair = None
    best_dist = -1.0

    # Find the two furthest blocks
    for (_, b1), (_, b2) in combinations(block_subset, 2):
        d = math.hypot(
            b2.x - b1.x,
            b2.y - b1.y
        )

        if d > best_dist:
            best_dist = d
            best_pair = (b1, b2)

    b_first, b_last = best_pair

    line_dx = b_last.x - b_first.x
    line_dy = b_last.y - b_first.y

    line_len = math.hypot(line_dx, line_dy)

    if line_len < 1e-9:
        return tuple(block_subset)

    ux = line_dx / line_len
    uy = line_dy / line_len

    return tuple(
        sorted(
            block_subset,
            key=lambda x: (
                x[1].x * ux + x[1].y * uy
            )
        )
    )


def is_inline(block_subset, max_line_error):
    """
    Check whether all blocks lie approximately on one line.
    """

    n = len(block_subset)

    if n <= 2:
        return True, 0.0

    block_subset = sort_blocks_along_line(block_subset)

    (_, b1) = block_subset[0]
    (_, b2) = block_subset[-1]

    max_err = 0.0

    for _, b in block_subset[1:-1]:
        err = point_line_distance(
            b.x, b.y,
            b1.x, b1.y,
            b2.x, b2.y
        )

        max_err = max(max_err, err)

        if err > max_line_error:
            return False, max_err

    return True, max_err


def compute_best_pickup(cups, blocks, team_color="blue"):
    """
    Block-first solver.

    1. Find the best inline block cluster.
    2. Fit cup geometry onto that cluster.

    Robot orientation does NOT affect target choice.
    """
    team_color = str(team_color).strip().lower()
    if team_color == "bleu":
        team_color = "blue"
    elif team_color == "jaune":
        team_color = "yellow"

    MAX_ERROR = 0.040               # max cup-to-block alignment error (m)
    MAX_LINE_ERROR = 0.030          # max block inline deviation (m)
    MAX_BLOCK_YAW_DIFF_DEG = 20.0   # max block yaw mismatch

    W_BLOCKS = 3000.0
    W_ERROR = 4000.0
    W_COLOR = 100.0
    W_INLINE = 500.0
    W_BLOCK_PARALLEL = 25.0

    cup_items = list(cups.items())

    block_items = [
        (n, b)
        for n, b in blocks.items()
        if b.color in ("blue", "yellow", "unknown")
    ]

    if not cup_items or not block_items:
        return None

    best = None

    max_n = min(
        len(cup_items),
        len(block_items)
    )

    # Prefer bigger groups first
    for n in range(max_n, 0, -1):

        cup_subsets = list(
            combinations(cup_items, n)
        )

        # Case a: camera cannot see cup_0 when docked — never assign a single
        # block to cup_0.  For multi-block picks cup_0 is still allowed in the
        # assignment set (its pose contribution is handled separately in
        # ros_node.py).
        if n == 1:
            cup_subsets = [cs for cs in cup_subsets if cs[0][0] != "cup_0"]
        if not cup_subsets:
            continue

        block_subsets = list(
            combinations(block_items, n)
        )

        for block_subset in block_subsets:

            # Sort blocks along their own line
            block_subset = sort_blocks_along_line(
                block_subset
            )

            # Check inline quality
            inline_ok, inline_err = is_inline(
                block_subset,
                MAX_LINE_ERROR
            )

            if not inline_ok:
                continue

            # Check yaw consistency
            yaw_spread = 0.0

            if n > 1:
                ref_yaw = block_subset[0][1].yaw_deg
                valid_parallel = True

                for _, b in block_subset[1:]:
                    diff = rectangular_yaw_diff_deg(
                        ref_yaw,
                        b.yaw_deg
                    )

                    yaw_spread = max(
                        yaw_spread,
                        diff
                    )

                    if diff > MAX_BLOCK_YAW_DIFF_DEG:
                        valid_parallel = False
                        break

                if not valid_parallel:
                    continue

            # Try fitting cup geometry onto this block cluster
            for cup_subset in cup_subsets:
                for perm in permutations(block_subset):

                    dx, dy, yaw = solve_pose(
                        cup_subset,
                        perm
                    )

                    total_err = 0.0
                    color_score = 0.0
                    valid = True
                    assigns = []

                    for (cn, cxy), (bn, b) in zip(
                        cup_subset,
                        perm
                    ):
                        rc = rotate_xy(cxy, yaw)

                        px = rc.x + dx
                        py = rc.y + dy

                        err = math.hypot(
                            px - b.x,
                            py - b.y
                        )

                        if err > MAX_ERROR:
                            valid = False
                            break

                        if b.color == team_color:
                            color_score += 1.0
                        elif b.color != "unknown":
                            color_score -= 0.5

                        total_err += err

                        assigns.append({
                            "cup": cn,
                            "block": b.name,
                            "color": b.color,
                            "err_m": err,
                            "err_mm": err * 1000.0,
                            "raw_id": b.raw_id,
                            "block_yaw_deg": b.yaw_deg
                        })

                    if not valid:
                        continue

                    avg_err = total_err / n

                    score = (
                        W_BLOCKS * n
                        - W_ERROR * avg_err
                        + W_COLOR * color_score
                        + W_INLINE * (1.0 - inline_err)
                        - W_BLOCK_PARALLEL * yaw_spread
                    )

                    cand = PickupCandidate(
                        score=score,
                        picked_count=n,
                        avg_error=avg_err,
                        yaw=yaw,
                        dx=dx,
                        dy=dy,
                        assignments=assigns
                    )

                    if best is None or cand.score > best.score:
                        best = cand

        # Early stop:
        # first valid larger cluster wins over smaller clusters
        if best is not None and best.picked_count == n:
            break

    return best


def recompute_locked_pose(cups, blocks, locked_assignments):
    """
    Recompute pose for existing locked assignments.
    """

    if not locked_assignments:
        return None

    cup_subset = []
    block_subset = []

    for a in locked_assignments:
        cup_name = a["cup"]
        block_name = a["block"]

        if cup_name not in cups:
            return None

        if block_name not in blocks:
            return None

        cup_subset.append(
            (cup_name, cups[cup_name])
        )

        block_subset.append(
            (block_name, blocks[block_name])
        )

    dx, dy, yaw = solve_pose(
        cup_subset,
        block_subset
    )

    total_err = 0.0
    new_assignments = []

    for (cn, cxy), (_, b) in zip(
        cup_subset,
        block_subset
    ):
        rc = rotate_xy(cxy, yaw)

        px = rc.x + dx
        py = rc.y + dy

        err = math.hypot(
            px - b.x,
            py - b.y
        )

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

    avg_err = total_err / len(
        locked_assignments
    )

    return PickupCandidate(
        score=0.0,
        picked_count=len(
            locked_assignments
        ),
        avg_error=avg_err,
        yaw=yaw,
        dx=dx,
        dy=dy,
        assignments=new_assignments
    )
