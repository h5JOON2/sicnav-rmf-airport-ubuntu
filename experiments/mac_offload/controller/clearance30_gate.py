"""Minimum body-surface clearance for swept linear prediction segments."""
import numpy as np

def min_body_clearance(states, robot_radius, human_radii):
    a = np.asarray(states, dtype=float)
    radii = np.asarray(human_radii, dtype=float)
    if a.ndim != 2 or len(a) < 2 or len(radii) < 1:
        raise ValueError('Expected at least two states and one pedestrian')
    hp = np.stack([a[:, 10+6*h:12+6*h] for h in range(len(radii))], axis=1)
    rel = hp - a[:, None, :2]
    if not np.isfinite(rel).all() or not np.isfinite(radii).all():
        raise ValueError('Nonfinite predicted position')
    start, delta = rel[:-1], np.diff(rel, axis=0)
    denominator = np.sum(delta*delta, axis=-1)
    fraction = np.clip(-np.sum(start*delta, axis=-1)/np.maximum(denominator, 1e-30), 0., 1.)
    closest = start + fraction[..., None]*delta
    return float(np.min(np.linalg.norm(closest, axis=-1)-float(robot_radius)-radii[None, :]))
