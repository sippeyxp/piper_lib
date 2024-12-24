"""Teleoperation with logging."""

import threading
import time
from typing import Any, Optional

import cv2
import numpy as np
from absl import app

import piper as piper_lib
import episode_logging
import pollo


def ensure_joints_synced(pollo: pollo.PolloReceiver, piper: piper_lib.Piper, match_piper_to_pollo=True):
  # make sure pollo is not too far from zero
  n = 0
  nn = 0
  while True:
    joints = pollo.sensed_joints()
    if nn % 20 == 0:
      print("pollo", " ".join(f"{i:.2f}" for i in joints))
    if np.all(np.abs(joints[:3]) < 0.3) and np.all(np.abs(joints[3:6]) < 0.5):
      n += 1
    else:
      n = 0
    if n > 100:
      break
    time.sleep(0.01)
    nn += 1

  print("pollo zeroed.")
  if match_piper_to_pollo:
    for i in range(100):
      joints = pollo.get_reading()
      piper.command_joints(joints * i / 100)
      time.sleep(0.01)
    print("started")


class CameraController:
  """Controls camera capture and logging functionality.
  
  This class manages camera initialization, configuration, and continuous frame capture
  in a separate thread. Captured frames are logged using the provided logger.
  """
  
  DEFAULT_CONFIG = {
    "camera_name": "default_camera",
    "camera_id": 0,
    "fps": 30,
    "auto_exposure": 1,  # manual mode
    "exposure": 150,     # unit 100 us
    "gain": 100         # gain
  }
  
  def __init__(self, config: dict[str, Any], logger: episode_logging.Logger):
    """Initialize camera controller with specified parameters.
    
    Args:
      config: Optional dictionary containing camera configuration including:
        - camera_name: Identifier string for the camera
        - camera_id: Device ID for the camera
        - fps: Frames per second
        - auto_exposure: Auto exposure mode (1 for manual)
        - exposure: Exposure time in 100us units
        - gain: Camera gain
      logger: Logger instance to record captured frames
    """
    # Use default config if none provided, otherwise update defaults with provided config
    self._config = self.DEFAULT_CONFIG.copy()
    self._config.update(config)
    self._camera_name = self._config["camera_name"]

    self._cap = cv2.VideoCapture(self._config["camera_id"])
    self._set_camera_property()

    self._logger = logger
    self._thread = None
    self._running = False

  def _set_camera_property(self):
    # Set camera properties from config
    self._cap.set(cv2.CAP_PROP_FPS, self._config["fps"])
    self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, self._config["auto_exposure"])
    self._cap.set(cv2.CAP_PROP_EXPOSURE, self._config["exposure"])
    self._cap.set(cv2.CAP_PROP_GAIN, self._config["gain"])
    
    # Print actual settings
    print(f"Camera {self._camera_name} settings:")
    print(f"  FPS: {self._cap.get(cv2.CAP_PROP_FPS)}")
    print(f"  Auto exposure: {self._cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)}")
    print(f"  Exposure: {self._cap.get(cv2.CAP_PROP_EXPOSURE)}")
    print(f"  Gain: {self._cap.get(cv2.CAP_PROP_GAIN)}")

  def start(self):
    """Start the camera capture thread."""
    self._running = True
    self._thread = threading.Thread(target=self._capture_thread)
    self._thread.daemon = True

    if self._config["display"]:
      # Create window for display
      cv2.namedWindow(self._camera_name, cv2.WINDOW_NORMAL)
    self._thread.start()

  def stop(self):
    """Stop the camera capture thread and release resources."""
    self._running = False
    if self._thread:
      self._thread.join()
      self._thread = None
    self._cap.release()

    if self._config["display"]:
      cv2.destroyWindow(self._camera_name)

  def _capture_thread(self):
    """Camera capture thread function.
    
    Continuously captures frames and logs them through the logger
    while the _running flag is True.
    """
    while self._running:
      ret, frame = self._cap.read()
      if ret:
        event = episode_logging.LogEvent(
          timestamp_ns=time.time_ns(),
          event_type=episode_logging.EventType.IMAGE,
          event_name=self._camera_name,
          image=frame
        )
        self._logger.log_event(event)
        if self._config["display"]:
          # Display frame
          cv2.imshow(self._camera_name, frame)
          cv2.waitKey(1)  # Required to update window
      time.sleep(0.001)


class TeleopController:
  """Controls teleoperation functionality between leader and follower robots.
  
  This class manages the interaction between the leader (Pollo) and follower (Piper)
  robots, handling initialization, engagement, and control loop execution.
  """
  def __init__(self, config: dict[str, Any], logger: episode_logging.Logger):
    """Initialize the teleoperation controller.

    Args:
      logger: Logger instance for recording robot states and events.
    """
    self._pollo = pollo.PolloReceiver(config=config["pollo"])
    self._piper = piper_lib.Piper(config=config["piper"])
    self._logger = logger
    self._running = False
    self._thread = None

  def move_follower_to_init(self):
    """Move the follower robot to its initial position."""
    self._piper.start()
    self._piper.enable_motion()
    self._piper.command_joints(np.zeros(7))

  def wait_leader_to_init(self):
    """Wait for the leader robot to reach its initial position."""
    ensure_joints_synced(self._pollo, self._piper, match_piper_to_pollo=False)

  def engage(self):
    """Start teleoperation by engaging both robots.
    
    This method:
    1. Enables MIT mode on the follower
    2. Gradually matches follower position to leader
    3. Starts the control loop thread
    """
    self._running = True
    self._piper.enter_mit_mode()
    print("matching joint")
    # assume engage with joint near 0
    for i in range(100):
      joints = self._pollo.sensed_joints()
      self._piper.command_joints(joints * i / 100)
      time.sleep(0.01)
    for i in range(50):
      joints = self._pollo.sensed_joints()
      self._piper.command_joints(joints)
    print("matching done")

    self._thread = threading.Thread(target=self.run_control_loop)
    self._thread.daemon = True
    self._thread.start()

  def disengage(self):
    """Stop teleoperation by disengaging both robots.
    
    This method:
    1. Stops the control loop
    2. Gradually moves follower back to zero position
    3. Exits MIT mode
    """
    self._running = False
    if self._thread:
      self._thread.join()
    j0 = self._piper.sensed_joints()
    for i in np.arange(1, 0, -0.01):
      self._piper.command_joints(j0 * i)
      time.sleep(0.01)
    for i in range(100):
      self._piper.command_joints(j0 * 0)
      time.sleep(0.01)
    self._piper.enter_mit_mode(False)

  def move_follower_to_rest(self):
    """Move the follower robot to its rest position."""
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3
    self._piper.command_joints(rest_joints)
    time.sleep(1.5)

  def close(self):
    """Clean up resources and close connections to both robots."""
    if self._running:
      self.disengage()
    self._piper.disable_motion()
    self._piper.close()
    self._pollo.close()

  def run_control_loop(self):
    """Main control loop that runs teleoperation.
    
    Continuously:
    1. Reads joint positions from leader
    2. Commands follower to match positions
    3. Logs commanded and observed joint positions
    """
    metronome = piper_lib.Metronome(100)
    while self._running:
      ts = time.time_ns()
      joints = self._pollo.sensed_joints()
      joints[6] = np.clip(joints[6], 0, 0.08)
      self._piper.command_joints(joints)
      
      event = episode_logging.LogEvent(
        timestamp_ns=ts,
        event_type=episode_logging.EventType.JOINT_POSITION,
        event_name="commanded",
        joint_pos=joints.tolist()
      )
      self._logger.log_event(event)

      event = episode_logging.LogEvent(
        timestamp_ns=ts,
        event_type=episode_logging.EventType.JOINT_POSITION,
        event_name="observed",
        joint_pos=self._piper.sensed_joints().tolist()
      )
      self._logger.log_event(event)
      
      metronome.wait()