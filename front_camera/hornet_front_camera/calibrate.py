"""Offline checkerboard calibration from saved images; all distances in metres."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import cv2
import numpy as np
import yaml


def matrix_entry(value):
    value = np.asarray(value)
    if value.ndim == 1:
        value = value.reshape(1, -1)
    return {'rows': value.shape[0], 'cols': value.shape[1], 'data': value.ravel().tolist()}


def camera_yaml(size, k, d, r, p, camera, calibration_id, rms):
    return {
        'image_width': size[0], 'image_height': size[1], 'camera_name': camera,
        'distortion_model': 'plumb_bob', 'camera_matrix': matrix_entry(k),
        'distortion_coefficients': matrix_entry(d), 'rectification_matrix': matrix_entry(r),
        'projection_matrix': matrix_entry(p), 'units': 'metres',
        'calibration_id': calibration_id, 'rectified_frame_id': f'{camera}_rect_optical_frame',
        'is_example': False, 'reprojection_rms_px': float(rms),
        'created_utc': datetime.now(timezone.utc).isoformat(),
    }


def image_files(directory):
    extensions = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'}
    files = {p.name: p for p in Path(directory).expanduser().iterdir() if p.suffix.lower() in extensions}
    if not files:
        raise ValueError(f'No calibration images in {directory}')
    return files


def collect(directories, board, square_size, min_views):
    lists = [image_files(path) for path in directories]
    if len(lists) == 2 and lists[0].keys() != lists[1].keys():
        raise ValueError('Stereo image filenames must match exactly in left and right directories')
    grid = np.zeros((board[0] * board[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * square_size
    objects, observations, size = [], [[] for _ in lists], None
    for name in sorted(lists[0]):
        points = []
        for files in lists:
            image = cv2.imread(str(files[name]), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f'Cannot read {files[name]}')
            current = (image.shape[1], image.shape[0])
            if size is None:
                size = current
            if current != size:
                raise ValueError('All images must have the same per-lens dimensions')
            # The sector-based detector returns subpixel corners directly and
            # avoids refinement windows spanning neighboring small squares.
            found, corners = cv2.findChessboardCornersSB(image, board, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not found:
                break
            points.append(corners)
        if len(points) == len(lists):
            objects.append(grid.copy())
            for target, point in zip(observations, points):
                target.append(point)
    if len(objects) < min_views:
        raise ValueError(f'Only {len(objects)} usable views; require at least {min_views}')
    return objects, observations, size



def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left-dir', required=True)
    parser.add_argument('--right-dir', required=True)
    parser.add_argument('--board-cols', type=int, required=True, help='Inner corners, not squares')
    parser.add_argument('--board-rows', type=int, required=True)
    parser.add_argument('--square-size-m', type=float, required=True)
    parser.add_argument('--min-views', type=int, default=12)
    parser.add_argument('--max-rms-px', type=float, default=1.0)
    parser.add_argument('--output-dir', required=True, help='New directory; existing data is never overwritten')
    args = parser.parse_args(argv)
    try:
        if min(args.board_cols, args.board_rows) < 3 or args.min_views < 3:
            raise ValueError('Board dimensions and minimum views must be at least three')
        if not np.isfinite([args.square_size_m, args.max_rms_px]).all() or min(args.square_size_m, args.max_rms_px) <= 0:
            raise ValueError('Square size and RMS limit must be finite and positive')
        output = Path(args.output_dir).expanduser()
        if output.exists():
            raise ValueError('Output directory already exists; choose a new run directory')
        objects, observations, size = collect([args.left_dir, args.right_dir], (args.board_cols, args.board_rows), args.square_size_m, args.min_views)
        fits = [cv2.calibrateCamera(objects, points, size, None, None) for points in observations]
        if any(not np.isfinite(fit[0]) or fit[0] > args.max_rms_px for fit in fits):
            raise ValueError(f'Camera calibration RMS exceeds limit: {[fit[0] for fit in fits]}')
        calibration_id = f'hornet-xii-{uuid4().hex[:16]}'
        _, kl, dl, _, _ = fits[0]
        _, kr, dr, _, _ = fits[1]
        rms, kl, dl, kr, dr, rotation, translation, _, _ = cv2.stereoCalibrate(
            objects, *observations, kl, dl, kr, dr, size, flags=cv2.CALIB_FIX_INTRINSIC)
        if not np.isfinite(rms) or rms > args.max_rms_px:
            raise ValueError(f'Stereo RMS {rms} exceeds limit {args.max_rms_px}')
        rl, rr, pl, pr, _, _, _ = cv2.stereoRectify(kl, dl, kr, dr, size, rotation, translation, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        if abs(pr[1, 3]) > 1e-7 or -pr[0, 3] / pr[0, 0] <= 0:
            raise ValueError('Expected horizontal stereo with the physical right camera to the right of left')
        documents = {
            'left.yaml': camera_yaml(size, kl, dl, rl, pl, 'front_left', calibration_id, rms),
            'right.yaml': camera_yaml(size, kr, dr, rr, pr, 'front_right', calibration_id, rms),
        }
        output.mkdir(parents=True, exist_ok=False)
        for filename, document in documents.items():
            (output / filename).write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')
        print(f'Saved {len(objects)} accepted views to {output}; calibration_id={calibration_id}')
        print('Validate held-out images and physical geometry before deployment.')
    except (OSError, ValueError, cv2.error) as error:
        parser.exit(1, f'Calibration failed: {error}\n')


if __name__ == '__main__':
    main()
