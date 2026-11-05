#!/usr/bin/env python3
"""
Optimal Reciprocal Collision Avoidance (ORCA) 2D Engine.
Implements the multi-agent reciprocal collision avoidance formulation
(Jur van den Berg, Stephen J. Guy, Ming Lin, Dinesh Manocha, 2011).
Pure Python / zero external dependencies.
"""

import math


class Line:
    __slots__ = ('point', 'direction')

    def __init__(self, point=(0.0, 0.0), direction=(0.0, 0.0)):
        self.point = point          # (px, py)
        self.direction = direction  # (dx, dy), unit vector


def det(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]


def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def normalize(v):
    l = math.hypot(v[0], v[1])
    if l < 1e-9:
        return (0.0, 0.0)
    return (v[0] / l, v[1] / l)


def linear_program1(lines, line_no, radius, opt_velocity, direction_opt):
    dot_prod = dot(lines[line_no].point, lines[line_no].direction)
    discriminant = dot_prod * dot_prod + radius * radius - dot(lines[line_no].point, lines[line_no].point)

    if discriminant < 0.0:
        return False, (0.0, 0.0)

    sqrt_disc = math.sqrt(discriminant)
    t_left = -dot_prod - sqrt_disc
    t_right = -dot_prod + sqrt_disc

    for i in range(line_no):
        denom = det(lines[line_no].direction, lines[i].direction)
        numer = det(lines[i].direction, (lines[line_no].point[0] - lines[i].point[0],
                                         lines[line_no].point[1] - lines[i].point[1]))

        if abs(denom) <= 1e-7:
            if numer < 0.0:
                return False, (0.0, 0.0)
            continue

        t = numer / denom
        if denom > 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)

        if t_left > t_right:
            return False, (0.0, 0.0)

    if direction_opt:
        if dot(opt_velocity, lines[line_no].direction) > 0.0:
            result = (lines[line_no].point[0] + t_right * lines[line_no].direction[0],
                      lines[line_no].point[1] + t_right * lines[line_no].direction[1])
        else:
            result = (lines[line_no].point[0] + t_left * lines[line_no].direction[0],
                      lines[line_no].point[1] + t_left * lines[line_no].direction[1])
    else:
        t = dot(lines[line_no].direction, (opt_velocity[0] - lines[line_no].point[0],
                                           opt_velocity[1] - lines[line_no].point[1]))
        if t < t_left:
            result = (lines[line_no].point[0] + t_left * lines[line_no].direction[0],
                      lines[line_no].point[1] + t_left * lines[line_no].direction[1])
        elif t > t_right:
            result = (lines[line_no].point[0] + t_right * lines[line_no].direction[0],
                      lines[line_no].point[1] + t_right * lines[line_no].direction[1])
        else:
            result = (lines[line_no].point[0] + t * lines[line_no].direction[0],
                      lines[line_no].point[1] + t * lines[line_no].direction[1])

    return True, result


def linear_program2(lines, radius, opt_velocity, direction_opt):
    if direction_opt:
        result = (opt_velocity[0] * radius, opt_velocity[1] * radius)
    elif dot(opt_velocity, opt_velocity) > radius * radius:
        result = normalize(opt_velocity)
        result = (result[0] * radius, result[1] * radius)
    else:
        result = opt_velocity

    for i in range(len(lines)):
        if det(lines[i].direction, (lines[i].point[0] - result[0], lines[i].point[1] - result[1])) > 0.0:
            success, result = linear_program1(lines, i, radius, opt_velocity, direction_opt)
            if not success:
                return False, result, i

    return True, result, len(lines)


def linear_program3(lines, num_obst_lines, begin_line, radius):
    distance = 0.0
    result = (0.0, 0.0)

    for i in range(begin_line, len(lines)):
        if det(lines[i].direction, (lines[i].point[0] - result[0], lines[i].point[1] - result[1])) > distance:
            proj_lines = []
            for j in range(num_obst_lines):
                proj_lines.append(lines[j])

            for j in range(num_obst_lines, i):
                determ = det(lines[i].direction, lines[j].direction)
                if abs(determ) <= 1e-7:
                    continue

                line = Line()
                line.point = (lines[i].point[0] + det(lines[j].direction,
                                                      (lines[i].point[0] - lines[j].point[0],
                                                       lines[i].point[1] - lines[j].point[1])) / determ * lines[i].direction[0],
                              lines[i].point[1] + det(lines[j].direction,
                                                      (lines[i].point[0] - lines[j].point[0],
                                                       lines[i].point[1] - lines[j].point[1])) / determ * lines[i].direction[1])
                diff_dir = (lines[j].direction[0] - lines[i].direction[0],
                            lines[j].direction[1] - lines[i].direction[1])
                line.direction = normalize(diff_dir)
                proj_lines.append(line)

            opt_vel = (-lines[i].direction[1], lines[i].direction[0])
            success, temp_result, _ = linear_program2(proj_lines, radius, opt_vel, True)
            if success:
                result = temp_result
                distance = det(lines[i].direction, (lines[i].point[0] - result[0], lines[i].point[1] - result[1]))

    return result


def compute_orca_line(pos_a, vel_a, radius_a, pos_b, vel_b, radius_b, time_horizon, inv_time_step, reciprocity=0.5):
    """Computes the 2D ORCA constraint line for agent A induced by neighbor B."""
    rel_pos = (pos_b[0] - pos_a[0], pos_b[1] - pos_a[1])
    rel_vel = (vel_a[0] - vel_b[0], vel_a[1] - vel_b[1])
    dist_sq = dot(rel_pos, rel_pos)
    combined_rad = radius_a + radius_b
    combined_rad_sq = combined_rad * combined_rad

    line = Line()

    if dist_sq > combined_rad_sq:
        inv_th = 1.0 / max(1e-4, time_horizon)
        w = (rel_vel[0] - inv_th * rel_pos[0], rel_vel[1] - inv_th * rel_pos[1])
        w_len_sq = dot(w, w)
        dot_prod1 = dot(w, rel_pos)

        if dot_prod1 < 0.0 and dot_prod1 * dot_prod1 > combined_rad_sq * w_len_sq:
            # Cut-off circle projection
            w_len = math.sqrt(w_len_sq)
            if w_len > 1e-9:
                unit_w = (w[0] / w_len, w[1] / w_len)
            else:
                unit_w = (0.0, 1.0)
            line.direction = (unit_w[1], -unit_w[0])
            u_mag = combined_rad * inv_th - w_len
            u = (u_mag * unit_w[0], u_mag * unit_w[1])
        else:
            # Cone leg projection
            leg = math.sqrt(max(0.0, dist_sq - combined_rad_sq))
            if det(rel_pos, w) > 0.0:
                line.direction = ((rel_pos[0] * leg - rel_pos[1] * combined_rad) / dist_sq,
                                  (rel_pos[0] * combined_rad + rel_pos[1] * leg) / dist_sq)
            else:
                line.direction = (- (rel_pos[0] * leg + rel_pos[1] * combined_rad) / dist_sq,
                                  - (-rel_pos[0] * combined_rad + rel_pos[1] * leg) / dist_sq)
            dot_prod2 = dot(w, line.direction)
            u = (dot_prod2 * line.direction[0] - w[0],
                 dot_prod2 * line.direction[1] - w[1])
    else:
        # Penetration / imminent contact
        w = (rel_vel[0] - inv_time_step * rel_pos[0], rel_vel[1] - inv_time_step * rel_pos[1])
        w_len = math.hypot(w[0], w[1])
        if w_len > 1e-9:
            unit_w = (w[0] / w_len, w[1] / w_len)
        else:
            unit_w = (0.0, 1.0)
        line.direction = (unit_w[1], -unit_w[0])
        u_mag = combined_rad * inv_time_step - w_len
        u = (u_mag * unit_w[0], u_mag * unit_w[1])

    line.point = (vel_a[0] + reciprocity * u[0], vel_a[1] + reciprocity * u[1])
    return line


def compute_orca_velocity(agent_pos, agent_vel, agent_pref_vel, agent_radius, agent_max_speed,
                          neighbors, time_horizon=2.5, dt=0.1):
    """
    Computes the optimal new collision-avoidance velocity for an agent using ORCA.
    :param neighbors: list of tuples: (pos, vel, radius, reciprocity)
    :return: (new_vx, new_vy)
    """
    inv_dt = 1.0 / max(1e-4, dt)
    orca_lines = []

    for n_pos, n_vel, n_rad, recip in neighbors:
        line = compute_orca_line(agent_pos, agent_vel, agent_radius,
                                 n_pos, n_vel, n_rad,
                                 time_horizon, inv_dt, reciprocity=recip)
        orca_lines.append(line)

    success, result, failed_idx = linear_program2(orca_lines, agent_max_speed, agent_pref_vel, False)
    if not success:
        result = linear_program3(orca_lines, 0, failed_idx, agent_max_speed)

    return result

