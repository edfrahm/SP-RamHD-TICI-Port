import numpy as np


def pose_offsets(pose, saved: tuple[float, float] | None) -> tuple[float, float]:
  """(pitch, yaw) of the driver's natural road-watching pose, in driver monitoring's frame.

  Monitoring learns it each drive (about 1 min above 30 mph). Until then, use the one saved from an earlier drive,
  else monitoring's own starting values.
  """
  if pose.calibrated or saved is None:
    return pose.pitchCalib.offset, pose.yawCalib.offset
  return saved


def relative_face_orientation(model_orient, pose, is_rhd: bool, offsets: tuple[float, float]) -> np.ndarray:
  """Head (pitch, yaw, roll) relative to the driver's natural pose, in the model's convention the face icon draws.

  The model's angles are relative to the camera, so a driver who sits off its axis (a wide cab, a high mount) looks
  turned while watching the road. Monitoring's pose corrects for where the face is in the image and for the device's
  mount angle; subtracting the natural pose leaves where the driver is looking. The offsets are not clamped: this
  shows the head, not monitoring's distraction decision (selfdrive/monitoring/policy.py clamps the ones it uses).
  """
  pitch = pose.pitch - offsets[0]
  yaw = pose.yaw - offsets[1]
  # policy.face_orientation_from_model: pitch = pitch_model + ..., yaw = -yaw_model + ...; yaw negated again for RHD
  return np.array([pitch, yaw if is_rhd else -yaw, model_orient[2]], dtype=np.float32)
