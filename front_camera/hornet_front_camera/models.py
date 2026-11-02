"""ROS-compatible pinhole calibration. No ROS runtime is imported here."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


def matrix(data, key, shape=None):
    entry = data[key]
    value = np.asarray(entry['data'], dtype=float).reshape(entry['rows'], entry['cols'])
    if not np.isfinite(value).all() or (shape is not None and value.shape != shape):
        raise ValueError(f'Invalid {key}')
    return value


@dataclass
class CameraModel:
    width: int
    height: int
    k: np.ndarray
    d: np.ndarray
    r: np.ndarray
    p: np.ndarray
    calibration_id: str
    frame_id: str
    example: bool

    @classmethod
    def load(cls, path, allow_example=False):
        with Path(path).expanduser().open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        if data.get('is_example', False) and not allow_example:
            raise ValueError('Synthetic example calibration is disabled; calibrate your cameras first')
        if data.get('units') != 'metres':
            raise ValueError('Calibration must explicitly declare units: metres')
        if data['distortion_model'] not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError('Only standard pinhole calibration is supported, not fisheye')
        model = cls(
            int(data['image_width']), int(data['image_height']),
            matrix(data, 'camera_matrix', (3, 3)), matrix(data, 'distortion_coefficients').ravel(),
            matrix(data, 'rectification_matrix', (3, 3)),
            matrix(data, 'projection_matrix', (3, 4)), str(data['calibration_id']),
            str(data['rectified_frame_id']), bool(data.get('is_example', False)),
        )
        if min(model.width, model.height) <= 0 or not model.calibration_id or not model.frame_id:
            raise ValueError('Positive image dimensions, calibration_id and rectified_frame_id are required')
        if model.d.size not in (4, 5, 8, 12, 14):
            raise ValueError('Unsupported pinhole distortion coefficient count')
        if min(model.k[0, 0], model.k[1, 1], model.p[0, 0], model.p[1, 1]) <= 0:
            raise ValueError('Focal lengths must be positive')
        if not np.allclose(model.r @ model.r.T, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(model.r), 1, atol=1e-4):
            raise ValueError('Rectification R must be a proper rotation')
        return model

    @property
    def size(self):
        return self.width, self.height

    def maps(self):
        import cv2
        return cv2.initUndistortRectifyMap(
            self.k, self.d, self.r, self.p[:, :3], self.size, cv2.CV_32FC1)


def validate_pair(left, right):
    """Validate the horizontal jointly rectified pair used by sparse depth."""
    if left.size != right.size or left.calibration_id != right.calibration_id:
        raise ValueError('Left/right calibration sizes and calibration_id must match')
    lp, rp = left.p, right.p
    canonical = lambda p: np.allclose(p[2], [0, 0, 1, 0], atol=1e-8) and np.allclose([p[0, 1], p[1, 0]], 0, atol=1e-8)
    if not canonical(lp) or not canonical(rp):
        raise ValueError('Expected canonical horizontal rectified projection matrices')
    if not np.allclose([lp[0, 0], lp[1, 1], lp[1, 2]], [rp[0, 0], rp[1, 1], rp[1, 2]], atol=1e-5):
        raise ValueError('Expected jointly rectified focal lengths and vertical principal points')
    if not np.allclose([lp[0, 3], lp[1, 3], rp[1, 3]], 0, atol=1e-8):
        raise ValueError('Expected zero left translation and horizontal right translation')
    baseline = -rp[0, 3] / rp[0, 0]
    if not np.isfinite(baseline) or baseline <= 1e-6:
        raise ValueError('Baseline must be positive; check left/right ordering and units')
    return float(baseline)
