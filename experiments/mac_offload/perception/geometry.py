"""RGB-D surface localization; optical axes right/down/forward, SI units."""
import numpy as np


def surface_point(depth, bbox, k, min_depth=0.15, max_depth=15.0):
    if depth.ndim != 2:
        raise ValueError('Depth must be a 2D aligned image')
    if not np.all(np.isfinite(bbox)) or not np.all(np.isfinite(k)):
        raise ValueError('Non-finite bbox or intrinsics')
    h, w = depth.shape
    x1, y1, x2, y2 = map(float, bbox)
    if x2 <= x1 or y2 <= y1:
        return None
    # Central 30% reduces background contamination, but does not segment objects.
    cx, cy = (x1+x2)/2, (y1+y2)/2
    rx, ry = max(1, (x2-x1)*0.15), max(1, (y2-y1)*0.15)
    left, right = max(0, int(cx-rx)), min(w, int(cx+rx)+1)
    top, bottom = max(0, int(cy-ry)), min(h, int(cy+ry)+1)
    if left >= right or top >= bottom:
        return None
    roi = depth[top:bottom, left:right]
    valid = np.isfinite(roi) & (roi >= min_depth) & (roi <= max_depth)
    if np.count_nonzero(valid) < 5:
        return None
    vv, uu = np.nonzero(valid)
    z = roi[valid].astype(float)
    median = np.median(z)
    # Use an actual pixel nearest the median depth, not a mismatched center ray.
    delta = np.abs(z-median)
    candidates = np.flatnonzero(delta == delta.min())
    distance = (uu[candidates]+left-cx)**2 + (vv[candidates]+top-cy)**2
    idx = int(candidates[np.argmin(distance)])
    u, v, z = uu[idx]+left, vv[idx]+top, float(z[idx])
    fx, fy, px, py = k[0], k[4], k[2], k[5]
    if fx <= 0 or fy <= 0:
        raise ValueError('Invalid camera intrinsics')
    return np.array([(u-px)*z/fx, (v-py)*z/fy, z])


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid odometry quaternion')
    x, y, z, w = q/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def to_odom(optical_point, position, quaternion):
    # Fixed sensor pose in camera-enabled SDF: forward 0.12m, height 0.28m.
    if not np.all(np.isfinite(optical_point)) or not np.all(np.isfinite(position)):
        raise ValueError('Non-finite point or odometry position')
    x, y, z = optical_point
    base_point = np.array([z+0.12, -x, -y+0.28])
    return rotation(quaternion) @ base_point + np.asarray(position)
