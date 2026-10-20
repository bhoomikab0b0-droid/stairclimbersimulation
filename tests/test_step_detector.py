"""
Unit tests for the perception pipeline using synthetic scans (pure numpy ray
casting against a 2D stair profile; no physics engine needed).

    python -m pytest tests/
"""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C  # noqa: E402
from perception.kalman import Kalman1D  # noqa: E402
from perception.step_detector import detect_step  # noqa: E402

SENSOR_H = 0.222


def stair_profile(riser_dist, h, d, n=4):
    """Line segments (x0, z0, x1, z1) of a staircase seen from a sensor at the origin."""
    g = -SENSOR_H
    segs = [(-0.1, g, riser_dist, g)]
    x, z = riser_dist, g
    for _ in range(n):
        segs.append((x, z, x, z + h))
        z += h
        segs.append((x, z, x + d, z))
        x += d
    return segs


def cast(segs, noise=0.0, rng=None):
    rng = rng or np.random.default_rng(0)
    angles = np.linspace(C.LIDAR_FOV_DOWN, C.LIDAR_FOV_UP, C.LIDAR_RAYS)
    pts = np.full((len(angles), 2), np.nan)
    for i, a in enumerate(angles):
        dx, dz = np.cos(a), np.sin(a)
        best = C.LIDAR_RANGE
        for x0, z0, x1, z1 in segs:
            ex, ez = x1 - x0, z1 - z0
            den = dx * ez - dz * ex
            if abs(den) < 1e-12:
                continue
            t = (x0 * ez - z0 * ex) / den
            u = (x0 * dz - z0 * dx) / den
            if 0 < t < best and 0 <= u <= 1:
                best = t
        if best < C.LIDAR_RANGE:
            r = best + rng.normal(0, noise)
            pts[i] = [r * dx, r * dz]
    return pts


@pytest.mark.parametrize("h,d,riser", [(0.15, 0.30, 0.35), (0.17, 0.34, 0.30), (0.19, 0.36, 0.45),
                                       (0.12, 0.33, 0.50), (0.20, 0.35, 0.03)])
def test_noise_free_geometry(h, d, riser):
    obs = detect_step(cast(stair_profile(riser, h, d)), ground_z_prior=-SENSOR_H)
    assert obs.step_ahead and not obs.blocked
    assert abs(obs.height - h) < 0.004
    assert abs(obs.riser_dist - riser) < 0.01
    assert obs.depth is not None and abs(obs.depth - d) < 0.01


def test_noisy_scans_with_kalman_filter():
    rng = np.random.default_rng(42)
    kf_h, kf_d = Kalman1D(r=2 * C.LIDAR_NOISE ** 2), Kalman1D(r=8 * C.LIDAR_NOISE ** 2)
    segs = stair_profile(0.35, 0.165, 0.33)
    for _ in range(12):
        obs = detect_step(cast(segs, C.LIDAR_NOISE, rng), ground_z_prior=-SENSOR_H)
        assert obs.step_ahead
        kf_h.update(obs.height)
        kf_d.update(obs.depth)
    assert abs(kf_h.x - 0.165) < 0.003
    assert abs(kf_d.x - 0.33) < 0.006
    assert kf_h.std < 0.003


def test_flat_floor_has_no_step():
    segs = [(-0.1, -SENSOR_H, 3.0, -SENSOR_H)]
    assert not detect_step(cast(segs), ground_z_prior=-SENSOR_H).step_ahead


def test_riser_taller_than_sensor_is_blocked():
    obs = detect_step(cast(stair_profile(0.35, 0.26, 0.35)), ground_z_prior=-SENSOR_H)
    assert obs.step_ahead and obs.blocked


def test_kalman_reduces_variance():
    kf = Kalman1D(r=1e-4)
    rng = np.random.default_rng(0)
    for _ in range(25):
        kf.update(0.2 + rng.normal(0, 0.01))
    assert abs(kf.x - 0.2) < 0.006 and kf.std < 0.003
