"""Calibrate from rendered boards with independently specified camera geometry."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'calibration_node'))
from hornet_calibration.calibrate import main, collect
from hornet_calibration.models import CameraModel, validate_pair


class OfflineCalibrationTests(unittest.TestCase):
    def render_dataset(self, root, count=16):
        columns, rows, square = 9,6,.025
        tile = 70
        yy,xx=np.indices(((rows+1)*tile,(columns+1)*tile))
        board = (((xx//tile+yy//tile) % 2) * 255).astype(np.uint8)
        k = np.array([[600.,0,320],[0,605.,240],[0,0,1.]])
        outer = np.array([[-square,-square,0],[columns*square,-square,0],
                          [columns*square,rows*square,0],[-square,rows*square,0]], np.float32)
        pixels = np.array([[0,0],[board.shape[1]-1,0],[board.shape[1]-1,board.shape[0]-1],[0,board.shape[0]-1]], np.float32)
        random=np.random.default_rng(19)
        for side in ['left','right']:
            (root/side).mkdir()
        for index in range(count):
            rotation=random.uniform([-.35,-.35,-.2],[.35,.35,.2])
            translation=random.uniform([-.13,-.09,.7],[-.03,-.03,1.15])
            for side, shift in [('left',0),('right',-.12)]:
                offset=translation+np.array([shift,0,0])
                projected,_=cv2.projectPoints(outer,rotation,offset,k,np.zeros(5))
                h=cv2.getPerspectiveTransform(pixels,projected.reshape(4,2).astype(np.float32))
                image=cv2.warpPerspective(board,h,(640,480),flags=cv2.INTER_LINEAR,borderValue=127)
                cv2.imwrite(str(root/side/f'{index:03}.png'),image)

    def test_stereo_and_mono_calibration_save_real_metadata_and_scale(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            self.render_dataset(root)
            args=['--board-cols','9','--board-rows','6','--square-size-m','.025','--min-views','12','--max-rms-px','1.0']
            with redirect_stdout(StringIO()):
                main(['--mode','stereo','--left-dir',str(root/'left'),'--right-dir',str(root/'right'),
                      '--output-dir',str(root/'stereo'),*args])
                main(['--mode','mono','--images-dir',str(root/'left'),'--output-dir',str(root/'mono'),*args])
            left=CameraModel.load(root/'stereo/left.yaml')
            right=CameraModel.load(root/'stereo/right.yaml')
            bottom=CameraModel.load(root/'mono/bottom.yaml')
            self.assertFalse(left.example)
            self.assertEqual(left.calibration_id,right.calibration_id)
            self.assertEqual(bottom.frame_id,'bottom_rect_optical_frame')
            self.assertAlmostEqual(validate_pair(left,right),.12,delta=.012)
            self.assertEqual(left.size,(640,480))
            with self.assertRaises(SystemExit):
                main(['--mode','mono','--images-dir',str(root/'left'),'--output-dir',str(root/'mono'),*args])

    def test_mismatched_pair_names_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for side,name in [('left','a.png'),('right','b.png')]:
                (root/side).mkdir()
                cv2.imwrite(str(root/side/name), np.zeros((100,100),np.uint8))
            with self.assertRaisesRegex(ValueError,'filenames'):
                collect([root/'left',root/'right'],(9,6),.025,3)


if __name__ == '__main__':
    unittest.main()
