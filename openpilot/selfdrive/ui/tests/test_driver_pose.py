import unittest

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.monitoring.policy import face_orientation_from_model
from openpilot.selfdrive.ui.onroad.driver_pose import pose_offsets, relative_face_orientation


def dm_pose(pitch, yaw, pitch_offset=0., yaw_offset=0., calibrated=True):
  msg = messaging.new_message('driverMonitoringState')
  pose = msg.driverMonitoringState.visionPolicyState.pose
  pose.pitch, pose.yaw, pose.calibrated = pitch, yaw, calibrated
  pose.pitchCalib.offset, pose.yawCalib.offset = pitch_offset, yaw_offset
  return pose


class TestDriverPose(unittest.TestCase):
  # the face icon shows the head relative to the driver's natural pose, not relative to the camera

  def test_matches_model_without_offsets(self):
    # face centred in the image, uncalibrated mount, nothing learned: the icon sees the model's own angles
    orient = [0.12, -0.3, 0.05]
    for is_rhd in (False, True):
      with self.subTest(is_rhd=is_rhd):
        pitch, yaw = face_orientation_from_model(orient, [0., 0.], [0., 0., 0.])
        if is_rhd:
          yaw *= -1  # as policy.DriverMonitoring does for a right-hand-drive driver
        np.testing.assert_allclose(relative_face_orientation(orient, dm_pose(pitch, yaw), is_rhd, (0., 0.)), orient, atol=1e-6)

  def test_natural_pose_is_straight_ahead(self):
    # Ram HD drive 2026-10-16: road-watching pose about 0.22 rad down and 0.22 rad to the side of the camera
    pose = dm_pose(-0.22, 0.22, pitch_offset=-0.22, yaw_offset=0.22)
    np.testing.assert_allclose(relative_face_orientation([0., 0., 0.07], pose, False, pose_offsets(pose, None)), [0., 0., 0.07], atol=1e-6)

  def test_offsets_before_calibration(self):
    pose = dm_pose(-0.22, 0.22, pitch_offset=0.011, yaw_offset=0.075, calibrated=False)
    self.assertEqual(pose_offsets(pose, (-0.2, 0.2)), (-0.2, 0.2))  # saved from an earlier drive
    self.assertAlmostEqual(pose_offsets(pose, None)[0], 0.011, places=6)  # monitoring's starting values
    pose.calibrated = True
    self.assertAlmostEqual(pose_offsets(pose, (-0.2, 0.2))[1], 0.075, places=6)  # this drive's, once learned


if __name__ == "__main__":
  unittest.main()
