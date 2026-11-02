"""Independent projection, uncertainty, map transform, and handover tests."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'calibration_node'), str(ROOT / 'depth_estimation_node')]
from hornet_calibration.models import CameraModel, validate_pair
from hornet_depth.geometry import Estimate, StereoGeometry, select_source, transform_estimate


def models():
    directory = ROOT / 'calibration_node/config/examples'
    return [CameraModel.load(directory / f'{side}.yaml', allow_example=True) for side in ['left', 'right']]


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.left, self.right = models()
        self.geometry = StereoGeometry(self.left, self.right)

    def project(self, xyz):
        projected = [model.p @ np.append(xyz, 1) for model in [self.left, self.right]]
        return [item[:2] / item[2] for item in projected]

    def test_recovers_independent_projections_with_principal_point_offset(self):
        for point in [[.1, .048, 1.2], [-.2, -.1, 2.0], [0, 0, 5.0]]:
            result = self.geometry.estimate(*self.project(point), pixel_stddev=.5)
            np.testing.assert_allclose(result.xyz, point, atol=1e-8)
            self.assertAlmostEqual(result.range_m, np.linalg.norm(point))
            self.assertLess(result.reprojection_rms_px, 1e-8)

    def test_rejects_bad_matches_outside_bounds_and_wrong_depth(self):
        left, right = self.project([.1, .048, 1.2])
        with self.assertRaisesRegex(ValueError, 'reprojection'):
            self.geometry.estimate(left, right + [0, 30], .5)
        for uv in [[-1, 100], [320, 100], [100, 240], [np.nan, 0]]:
            with self.assertRaises(ValueError):
                self.geometry.estimate(uv, right, .5)
        with self.assertRaisesRegex(ValueError, 'disparity'):
            self.geometry.estimate([160, 120], [154, 120], .5)
        with self.assertRaisesRegex(ValueError, 'disparity'):
            self.geometry.estimate(*self.project([0, 0, -2]), .5)
        with self.assertRaisesRegex(ValueError, 'range'):
            self.geometry.estimate(*self.project([0, 0, 30]), .5)

    def test_farther_points_have_larger_depth_uncertainty(self):
        near = self.geometry.estimate(*self.project([0, 0, 1]), .5)
        far = self.geometry.estimate(*self.project([0, 0, 4]), .5)
        self.assertGreater(far.stddev_m, near.stddev_m * 10)
        self.assertGreaterEqual(np.linalg.eigvalsh(far.covariance).min(), -1e-10)

    def test_covariance_agrees_with_monte_carlo_pixel_noise(self):
        # Independent random perturbations, not an assertion of the Jacobian formula.
        uv = np.concatenate(self.project([.1, -.05, 2]))
        expected = self.geometry.estimate(uv[:2], uv[2:], .15)
        random = np.random.default_rng(14)
        points = []
        for noisy in random.normal(uv, .15, size=(500, 4)):
            points.append(self.geometry.estimate(noisy[:2], noisy[2:], .15).xyz)
        empirical = np.cov(np.asarray(points).T)
        np.testing.assert_allclose(np.diag(expected.covariance), np.diag(empirical), rtol=.18, atol=1e-6)

    def test_map_rotation_translation_and_covariance(self):
        estimate = Estimate(np.array([1., 0, 2]), np.diag([1., 4, 9]), np.sqrt(5), 0)
        # A +90-degree map rotation around Z maps camera X to map Y.
        xyz, covariance = transform_estimate(estimate, [10, 20, 30], [0, 0, np.sqrt(.5), np.sqrt(.5)])
        np.testing.assert_allclose(xyz, [10, 21, 32], atol=1e-12)
        np.testing.assert_allclose(covariance, np.diag([4, 1, 9]), atol=1e-12)
        with self.assertRaises(ValueError):
            transform_estimate(estimate, [0, 0, 0], [0, 0, 0, 0])

    def test_guards_synthetic_examples_and_mismatched_pair(self):
        with self.assertRaisesRegex(ValueError, 'Synthetic'):
            CameraModel.load(ROOT / 'calibration_node/config/examples/left.yaml')
        self.right.calibration_id = 'another-run'
        with self.assertRaisesRegex(ValueError, 'calibration_id'):
            validate_pair(self.left, self.right)
        self.right.calibration_id = self.left.calibration_id
        self.right.p[0, 3] *= -1
        with self.assertRaisesRegex(ValueError, 'Baseline'):
            validate_pair(self.left, self.right)


class SelectionTests(unittest.TestCase):
    def estimate(self, distance, sigma=.05):
        return Estimate(np.array([0, 0, distance]), np.eye(3)*sigma**2, distance, 0)

    def test_range_hysteresis_avoids_boundary_chatter(self):
        desired = 'object_detection'
        for distance, expected in [(4, 'object_detection'), (1.9, 'color_threshold'), (2.2, 'color_threshold'), (2.6, 'object_detection')]:
            sources = {key: self.estimate(distance) for key in ['object_detection', 'color_threshold']}
            chosen, desired, _ = select_source(sources, desired)
            self.assertEqual(chosen, expected)

    def test_missing_or_uncertain_preferred_source_uses_valid_fallback(self):
        sources = {'object_detection': self.estimate(1.5), 'color_threshold': self.estimate(1.5, 1.0)}
        chosen, desired, reason = select_source(sources)
        self.assertEqual((chosen, desired), ('object_detection', 'color_threshold'))
        self.assertIn('fallback', reason)
        self.assertEqual(select_source({'color_threshold': self.estimate(5)})[0], 'color_threshold')
        self.assertEqual(select_source({})[0], '')
        self.assertEqual(select_source({'object_detection': self.estimate(5, 2)})[0], '')


if __name__ == '__main__':
    unittest.main()
