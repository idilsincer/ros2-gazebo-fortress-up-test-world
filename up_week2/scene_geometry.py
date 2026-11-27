import math

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def camera_intrinsics(width, height, hfov):
    """Pinhole intrinsics derived from the horizontal field of view."""
    fx = (float(width) * 0.5) / math.tan(float(hfov) * 0.5)
    fy = fx  # square pixels
    return fx, fy, float(width) * 0.5, float(height) * 0.5


def project_point(point, camera_position, camera_rpy, width, height, hfov):
    """Project a world point into pixel coordinates.

    Returns ``(u, v, depth)`` or ``None`` when the point is behind the camera.
    """
    rot = rpy_matrix(*camera_rpy)
    delta = np.asarray(point, dtype=float) - np.asarray(camera_position,
                                                        dtype=float)
    p_cam = rot.T @ delta
    depth = p_cam[0]
    if depth <= 1e-6:
        return None

    fx, fy, cx, cy = camera_intrinsics(width, height, hfov)
    u = cx - fx * p_cam[1] / depth
    v = cy - fy * p_cam[2] / depth
    return float(u), float(v), float(depth)


def segment_hits_box(start, end, box_center, box_size, inflate=0.0):
    """True when the segment ``start -> end`` crosses an axis-aligned box.

    Slab method.  ``t`` is restricted to the open segment so that a target
    sitting exactly on the box surface is not treated as occluded.
    """
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    center = np.asarray(box_center, dtype=float)
    half = np.asarray(box_size, dtype=float) * 0.5 + float(inflate)

    direction = end - start
    t_min, t_max = 0.0, 1.0

    for axis in range(3):
        if abs(direction[axis]) < 1e-9:
            if abs(start[axis] - center[axis]) > half[axis]:
                return False
            continue
        t1 = (center[axis] - half[axis] - start[axis]) / direction[axis]
        t2 = (center[axis] + half[axis] - start[axis]) / direction[axis]
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return False

    return 0.0 < t_min < 1.0 and t_max > t_min


def target_visibility(target_center, camera_position, camera_rpy,
                      width, height, hfov, obstacle_center, obstacle_size,
                      target_radius=0.2, target_height=1.0, margin=8.0):
    """Classify how much of the target is visible from the camera.

    The target is approximated by a 3x3 grid of points on its silhouette
    (box of ``target_radius`` and ``target_height``).  A sample is
    visible when it projects inside the image and the camera ray is not
    blocked by the obstacle box.

    Returns one of ``"full"``, ``"partial"``, ``"hidden"`` or
    ``"out_of_view"``.
    """
    target_center = np.asarray(target_center, dtype=float)
    samples = []
    for dz in (-0.45 * target_height, 0.0, 0.45 * target_height):
        for dx in (-0.9 * target_radius, 0.0, 0.9 * target_radius):
            for dy in (-0.9 * target_radius, 0.0, 0.9 * target_radius):
                samples.append(target_center + np.array([dx, dy, dz]))

    inside = 0
    visible = 0
    for sample in samples:
        projected = project_point(sample, camera_position, camera_rpy,
                                  width, height, hfov)
        if projected is None:
            continue
        u, v, _ = projected
        if margin <= u < width - margin and margin <= v < height - margin:
            inside += 1
        if segment_hits_box(camera_position, sample,
                            obstacle_center, obstacle_size, inflate=0.01):
            continue
        if margin <= u < width - margin and margin <= v < height - margin:
            visible += 1

    if inside == 0:
        return 'out_of_view'
    if visible == 0:
        return 'hidden'
    if visible == len(samples):
        return 'full'
    return 'partial'
