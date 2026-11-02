"""Testable geometry and selection policy, independent of ROS."""
from dataclasses import dataclass

import numpy as np

from hornet_calibration.models import validate_pair


@dataclass
class Estimate:
    xyz: np.ndarray
    covariance: np.ndarray
    range_m: float
    reprojection_rms_px: float

    @property
    def stddev_m(self):
        return float(np.sqrt(max(0.0, np.linalg.eigvalsh(self.covariance)[-1])))


class StereoGeometry:
    def __init__(self, left, right, min_range_m=0.2, max_range_m=15.0,
                 min_disparity_px=0.5, max_reprojection_px=2.0):
        self.baseline = validate_pair(left, right)
        self.left, self.right = left, right
        if not np.isfinite([min_range_m, max_range_m, min_disparity_px, max_reprojection_px]).all():
            raise ValueError('Geometry limits must be finite')
        if not 0 < min_range_m < max_range_m or min(min_disparity_px, max_reprojection_px) <= 0:
            raise ValueError('Invalid geometry limits')
        self.min_range, self.max_range = min_range_m, max_range_m
        self.min_disparity, self.max_reprojection = min_disparity_px, max_reprojection_px

    def _triangulate(self, pixels):
        rows = []
        for uv, model in zip(pixels.reshape(2, 2), (self.left, self.right)):
            rows.extend([uv[0] * model.p[2] - model.p[0], uv[1] * model.p[2] - model.p[1]])
        _, _, vt = np.linalg.svd(rows)
        point = vt[-1]
        if abs(point[3]) < 1e-10:
            raise ValueError('Triangulation is at infinity')
        xyz = point[:3] / point[3]
        if not np.isfinite(xyz).all() or xyz[2] <= 0:
            raise ValueError('Point is behind the left camera')
        return xyz

    def estimate(self, left_uv, right_uv, pixel_stddev):
        pixels = np.asarray([*left_uv, *right_uv], dtype=float)
        if pixels.shape != (4,) or not np.isfinite(pixels).all():
            raise ValueError('Require four finite rectified pixel coordinates')
        if not np.isfinite(pixel_stddev) or not 0 < pixel_stddev <= 100:
            raise ValueError('pixel_stddev_px must be positive and at most 100')
        for uv, model in zip(pixels.reshape(2, 2), (self.left, self.right)):
            if not (0 <= uv[0] < model.width and 0 <= uv[1] < model.height):
                raise ValueError('Pixel coordinates outside calibrated per-lens image')
        effective_disparity = pixels[0] - pixels[2] - (self.left.p[0, 2] - self.right.p[0, 2])
        if effective_disparity < self.min_disparity:
            raise ValueError('Nonpositive or too-small corrected disparity')
        xyz = self._triangulate(pixels)
        errors = []
        for uv, model in zip(pixels.reshape(2, 2), (self.left, self.right)):
            projected = model.p @ np.append(xyz, 1)
            if projected[2] <= 0:
                raise ValueError('Point is behind a camera')
            errors.append(float(np.linalg.norm(projected[:2] / projected[2] - uv)))
        if max(errors) > self.max_reprojection:
            raise ValueError('Stereo centers fail the reprojection consistency gate')
        distance = float(np.linalg.norm(xyz))
        if not self.min_range <= distance <= self.max_range:
            raise ValueError('Object range outside configured limits')
        # Propagate independent pixel noise through the actual triangulation.
        # This is conditional on exact calibration and robot pose, not true error.
        jacobian = np.zeros((3, 4))
        epsilon = 1e-3
        for index in range(4):
            delta = np.zeros(4)
            delta[index] = epsilon
            jacobian[:, index] = (self._triangulate(pixels + delta) - self._triangulate(pixels - delta)) / (2 * epsilon)
        covariance = jacobian @ jacobian.T * pixel_stddev ** 2
        if not np.isfinite(covariance).all():
            raise ValueError('Invalid triangulation uncertainty')
        return Estimate(xyz, (covariance + covariance.T) / 2, distance, float(np.sqrt(np.mean(np.square(errors)))))


def rotation_matrix(quaternion):
    """ROS quaternion order x,y,z,w. Reject absent/invalid orientation."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def transform_estimate(estimate, translation, quaternion):
    translation = np.asarray(translation, dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('Invalid transform translation')
    rotation = rotation_matrix(quaternion)
    return rotation @ estimate.xyz + translation, rotation @ estimate.covariance @ rotation.T


def select_source(estimates, desired='object_detection', enter_color_m=2.0,
                  exit_color_m=2.5, max_stddev_m=0.5):
    """Return chosen source, desired mode (with hysteresis), and explanation.

    Both sources are kept alive. A usable fallback is allowed if the preferred
    source is missing or too uncertain. Never average unrelated object centers.
    """
    if not 0 < enter_color_m < exit_color_m or max_stddev_m <= 0:
        raise ValueError('Require 0 < enter_color_m < exit_color_m and positive uncertainty limit')
    usable = {source: item for source, item in estimates.items() if item.stddev_m <= max_stddev_m}
    if not usable:
        return '', desired, 'No fresh estimate passes the stereo uncertainty gate'
    anchor = usable.get('object_detection', usable.get('color_threshold'))
    if desired == 'color_threshold':
        desired = 'object_detection' if anchor.range_m >= exit_color_m else desired
    elif anchor.range_m <= enter_color_m:
        desired = 'color_threshold'
    else:
        desired = 'object_detection'
    chosen = desired if desired in usable else min(usable, key=lambda key: usable[key].stddev_m)
    reason = 'Preferred source passes gates' if chosen == desired else 'Preferred source unavailable; using valid fallback'
    return chosen, desired, reason
