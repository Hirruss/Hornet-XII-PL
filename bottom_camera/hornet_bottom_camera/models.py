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
