"""
2D geometry utilities for path tracking metrics.
Includes point-to-segment projection, line intersection, and signed distance calculations.
"""

import math
from typing import Tuple, Optional, List
from dataclasses import dataclass


def seg_intersect(p1: Tuple[float, float], p2: Tuple[float, float],
                  q1: Tuple[float, float], q2: Tuple[float, float]) -> bool:
    """
    Check if two 2D line segments intersect.
    
    Args:
        p1, p2: Endpoints of first segment (vehicle trajectory)
        q1, q2: Endpoints of second segment (start line)
    
    Returns:
        True if segments intersect (including endpoints), False otherwise
    """
    def orient(a, b, c):
        """Orientation test: positive if counter-clockwise."""
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    
    o1 = orient(p1, p2, q1)
    o2 = orient(p1, p2, q2)
    o3 = orient(q1, q2, p1)
    o4 = orient(q1, q2, p2)
    
    # General case: segments intersect if orientations differ
    if (o1 * o2 < 0) and (o3 * o4 < 0):
        return True
    
    # Special cases: check if endpoints are on the other segment
    # This handles cases where vehicle passes exactly through start line endpoints
    def point_on_segment(p, a, b):
        """Check if point p is on segment ab."""
        if abs((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) > 1e-9:
            return False  # Not collinear
        # Check if p is between a and b
        return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and
                min(a[1], b[1]) <= p[1] <= max(a[1], b[1]))
    
    # Check if any endpoint of one segment lies on the other segment
    if (point_on_segment(p1, q1, q2) or point_on_segment(p2, q1, q2) or
        point_on_segment(q1, p1, p2) or point_on_segment(q2, p1, p2)):
        return True
    
    return False


def project_point_to_segment(p: Tuple[float, float],
                              a: Tuple[float, float],
                              b: Tuple[float, float]) -> Tuple[Tuple[float, float], float, float]:
    """
    Project a point onto a line segment.
    
    Args:
        p: Point to project
        a, b: Endpoints of segment
    
    Returns:
        (projected_point, t, squared_distance)
        - projected_point: Closest point on segment
        - t: Parameter in [0, 1] (0 at a, 1 at b)
        - squared_distance: Squared distance from p to segment
    """
    ax, ay = a
    bx, by = b
    px, py = p
    
    vx, vy = (bx - ax), (by - ay)
    wx, wy = (px - ax), (py - ay)
    vv = vx * vx + vy * vy
    
    if vv <= 1e-12:
        # Degenerate segment (a == b)
        dx, dy = px - ax, py - ay
        return a, 0.0, dx * dx + dy * dy
    
    t = (wx * vx + wy * vy) / vv
    t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
    
    proj = (ax + t * vx, ay + t * vy)
    dx, dy = px - proj[0], py - proj[1]
    return proj, t, dx * dx + dy * dy


def signed_lateral_error(p: Tuple[float, float],
                         a: Tuple[float, float],
                         b: Tuple[float, float],
                         proj: Tuple[float, float]) -> float:
    """
    Calculate signed lateral error using 2D cross product.
    
    Args:
        p: Vehicle position
        a, b: Segment endpoints
        proj: Projected point on segment
    
    Returns:
        Signed distance (positive = left of direction, negative = right)
    """
    # Tangent vector (normalized direction)
    tx, ty = (b[0] - a[0]), (b[1] - a[1])
    # Vector from projection to point
    rx, ry = (p[0] - proj[0]), (p[1] - proj[1])
    
    # 2D cross product (z-component)
    cross_z = tx * ry - ty * rx
    
    # Distance magnitude
    dist = math.hypot(rx, ry)
    
    # Sign: positive if cross_z > 0 (left side), negative otherwise
    return dist if cross_z > 0 else -dist


def point_to_path_distance(p: Tuple[float, float],
                           path: list) -> Tuple[float, float, int]:
    """
    Find minimum distance from point to polyline path.
    
    Args:
        p: Point (x, y)
        path: List of (x, y) points forming polyline
    
    Returns:
        (signed_distance, squared_distance, segment_index)
        - signed_distance: Signed lateral error
        - squared_distance: Squared distance
        - segment_index: Index of closest segment
    """
    if len(path) < 2:
        return float("nan"), float("inf"), -1
    
    best_d2 = float("inf")
    best_seg_idx = -1
    best_proj = None
    best_seg = None
    
    for k in range(len(path) - 1):
        a = path[k]
        b = path[k + 1]
        proj, t, d2 = project_point_to_segment(p, a, b)
        
        if d2 < best_d2:
            best_d2 = d2
            best_seg_idx = k
            best_proj = proj
            best_seg = (a, b)
    
    if best_seg is None:
        return float("nan"), float("inf"), -1
    
    signed_dist = signed_lateral_error(p, best_seg[0], best_seg[1], best_proj)
    return signed_dist, best_d2, best_seg_idx


def compute_path_length(path: list) -> float:
    """
    Compute total length of polyline path.
    
    Args:
        path: List of (x, y) points
    
    Returns:
        Total path length in meters
    """
    if len(path) < 2:
        return 0.0
    
    total = 0.0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        total += math.hypot(dx, dy)
    
    return total


def is_point_inside_track_boundary(point: Tuple[float, float],
                                   centerline: list,
                                   track_half_width: float) -> bool:
    """
    Check if a point is inside the track boundary.
    
    Track boundary is defined as centerline +/- track_half_width perpendicular to the path.
    
    Args:
        point: Point to check (x, y)
        centerline: List of (x, y) points forming the centerline
        track_half_width: Half-width of track [m]
    
    Returns:
        True if point is inside track boundary, False otherwise
    """
    if len(centerline) < 2:
        return False
    
    # Find nearest segment on centerline
    min_dist_sq = float("inf")
    nearest_seg_idx = -1
    nearest_proj = None
    
    for k in range(len(centerline) - 1):
        a = centerline[k]
        b = centerline[k + 1]
        proj, t, d2 = project_point_to_segment(point, a, b)
        
        if d2 < min_dist_sq:
            min_dist_sq = d2
            nearest_seg_idx = k
            nearest_proj = proj
    
    if nearest_seg_idx < 0:
        return False
    
    # Distance from point to centerline
    dist_to_centerline = math.sqrt(min_dist_sq)
    
    # Point is inside if distance is less than track half-width
    return dist_to_centerline <= track_half_width


def check_track_boundary_violation(point: Tuple[float, float],
                                   centerline: list,
                                   track_half_width: float) -> Tuple[bool, float]:
    """
    Check if point violates track boundary and return violation distance.
    
    Args:
        point: Point to check (x, y)
        centerline: List of (x, y) points forming the centerline
        track_half_width: Half-width of track [m]
    
    Returns:
        (is_violation, violation_distance)
        - is_violation: True if point is outside track boundary
        - violation_distance: Distance beyond track boundary (positive if outside, 0 if inside)
    """
    if len(centerline) < 2:
        return (True, float("inf"))
    
    # Find nearest segment
    min_dist_sq = float("inf")
    for k in range(len(centerline) - 1):
        a = centerline[k]
        b = centerline[k + 1]
        _, _, d2 = project_point_to_segment(point, a, b)
        if d2 < min_dist_sq:
            min_dist_sq = d2
    
    dist_to_centerline = math.sqrt(min_dist_sq)
    violation_dist = max(0.0, dist_to_centerline - track_half_width)
    
    return (violation_dist > 0.0, violation_dist)


# ========== Polyline Projection for Progress-based Lap Detection ==========

Point2 = Tuple[float, float]


@dataclass(frozen=True)
class PolylineProjection:
    """Result of projecting a point onto a polyline."""
    proj: Point2                 # Projected point coordinates
    seg_idx: int                 # Segment index (0..len(path)-2)
    t: float                     # Interpolation parameter [0,1] within segment
    dist: float                  # Minimum distance (unsigned)
    s: float                     # Cumulative arc length coordinate (0..total_length)
    signed_dist: Optional[float] = None  # Signed lateral error (optional)


def compute_polyline_prefix_lengths(path: List[Point2]) -> List[float]:
    """
    Compute cumulative arc lengths up to each point in path.
    
    Args:
        path: List of (x, y) points forming polyline
    
    Returns:
        prefix[i] = cumulative length up to path[i]
        prefix[0] = 0.0, prefix[-1] = total length
    """
    if len(path) < 2:
        raise ValueError("path must have at least 2 points")
    
    prefix = [0.0]
    total = 0.0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        total += math.hypot(dx, dy)
        prefix.append(total)
    return prefix


def _project_point_to_segment_raw(
    p: Point2, a: Point2, b: Point2
) -> Tuple[Point2, float, float]:
    """
    Internal: Project point p onto segment a-b.
    
    Returns:
        (proj, t, d2)
        - proj: Projected point
        - t: Interpolation parameter [0,1]
        - d2: Squared distance from p to proj
    """
    ax, ay = a
    bx, by = b
    px, py = p
    
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    
    ab2 = abx * abx + aby * aby
    if ab2 <= 1e-12:
        # Degenerate segment (a == b)
        proj = a
        dx = px - ax
        dy = py - ay
        return proj, 0.0, dx * dx + dy * dy
    
    t = (apx * abx + apy * aby) / ab2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    
    proj = (ax + t * abx, ay + t * aby)
    dx = px - proj[0]
    dy = py - proj[1]
    return proj, t, dx * dx + dy * dy


def project_point_to_polyline(
    pos: Point2,
    path: List[Point2],
    prefix: Optional[List[float]] = None,
    compute_signed: bool = False,
) -> PolylineProjection:
    """
    Project point onto polyline and return closest projection with arc length coordinate.
    
    Args:
        pos: Point (x, y) to project
        path: Polyline as list of (x, y) points (>=2)
        prefix: Precomputed prefix lengths (if None, computed internally)
        compute_signed: If True, compute signed lateral error
    
    Returns:
        PolylineProjection(proj, seg_idx, t, dist, s, signed_dist)
    """
    if len(path) < 2:
        raise ValueError("path must have at least 2 points")
    
    if prefix is None:
        prefix = compute_polyline_prefix_lengths(path)
    if len(prefix) != len(path):
        raise ValueError("prefix length must match path length")
    
    best_d2 = float("inf")
    best = None  # (proj, seg_idx, t, d2)
    
    for i in range(len(path) - 1):
        proj, t, d2 = _project_point_to_segment_raw(pos, path[i], path[i + 1])
        if d2 < best_d2:
            best_d2 = d2
            best = (proj, i, t, d2)
    
    assert best is not None
    proj, seg_idx, t, d2 = best
    dist = math.sqrt(d2)
    
    # Compute s: prefix[seg_idx] + t * segment_length
    ax, ay = path[seg_idx]
    bx, by = path[seg_idx + 1]
    seg_len = math.hypot(bx - ax, by - ay)
    s = prefix[seg_idx] + t * seg_len
    
    signed_dist = None
    if compute_signed:
        # Segment direction u (normalized)
        ux = bx - ax
        uy = by - ay
        ulen = math.hypot(ux, uy)
        if ulen > 1e-12:
            ux /= ulen
            uy /= ulen
            # Left normal n = (-uy, ux)
            nx, ny = -uy, ux
            vx = pos[0] - proj[0]
            vy = pos[1] - proj[1]
            signed_dist = vx * nx + vy * ny
        else:
            signed_dist = 0.0
    
    return PolylineProjection(
        proj=proj,
        seg_idx=seg_idx,
        t=t,
        dist=dist,
        s=s,
        signed_dist=signed_dist,
    )


def circular_progress(prev_s: float, curr_s: float, length: float) -> float:
    """
    Compute forward progress in circular track (0..length).
    
    Handles wrap-around: prev_s near end and curr_s near start.
    
    Args:
        prev_s: Previous arc length coordinate [0, length)
        curr_s: Current arc length coordinate [0, length)
        length: Total track length
    
    Returns:
        Forward progress in [0, length) range
        Example: prev=95, curr=2, length=100 => 7
    """
    if length <= 1e-12:
        return 0.0
    
    ds = curr_s - prev_s
    if ds < 0.0:
        ds += length
    # ds is now in [0, length)
    return ds
