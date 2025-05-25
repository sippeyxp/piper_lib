"""Teleoperation with logging."""

import threading
import time
from typing import Any, Optional, Union

import cv2
import numpy as np
from absl import app

import piper as piper_lib
import episode_logging
import pollo
from timing import Metronome


def ensure_joints_synced(
    pairs: list[tuple[pollo.PolloReceiver, piper_lib.Piper]],
    match_piper_to_pollo=True
):
    """Ensure all pollo arms are near zero and optionally match all pipers to pollos.

    Args:
        pairs: List of (PolloReceiver, Piper) tuples
        match_piper_to_pollo: If True, move pipers to match pollos after sync
    """
    n = [0 for _ in pairs]
    nn = 0
    while True:
        all_synced = True
        for idx, (pollo_inst, _) in enumerate(pairs):
            joints = pollo_inst.sensed_joints()
            if nn % 20 == 0:
                print(f"pollo[{idx}]", " ".join(f"{i:.2f}" for i in joints))
            if np.all(np.abs(joints[:3]) < 0.3) and np.all(np.abs(joints[3:6]) < 0.5):
                n[idx] += 1
            else:
                n[idx] = 0
            if n[idx] <= 100:
                all_synced = False
        if all_synced:
            break
        time.sleep(0.01)
        nn += 1

    print("All pollos zeroed.")
    if match_piper_to_pollo:
        for i in range(100):
            for pollo_inst, piper_inst in pairs:
                joints = pollo_inst.get_reading()
                piper_inst.command_joints(joints * i / 100)
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

  @property
  def name(self):
    return self._camera_name

  def _set_camera_property(self):
    # Set camera properties from config
    # Reference https://stackoverflow.com/questions/54365170/exposure-mode-opencv-4-0-1
    auto_exposure = 3
    manual_exposure = 1

    self._cap.set(cv2.CAP_PROP_FPS, self._config["fps"])
    self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, auto_exposure if self._config.get("auto_exposure", True) else manual_exposure)
    if self._config.get("exposure", None) is not None:
      self._cap.set(cv2.CAP_PROP_EXPOSURE, self._config["exposure"])
    if self._config.get("gain", None) is not None:
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

    if self._config.get("display"):
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


class CameraGroupController:
    """Controls a group of cameras together.

    This class manages multiple CameraController instances as a coordinated group,
    providing unified start/stop functionality.
    """
    def __init__(self, camera_configs: Union[list[dict], dict], logger: episode_logging.Logger):
        """Initialize camera group controller.

        Args:
            camera_configs: List of camera configuration dictionaries
            logger: Logger instance for recording camera frames
        """
        self._controllers = []
        if isinstance(camera_configs, dict):
          camera_configs = [camera_configs]
        for config in camera_configs:
            controller = CameraController(
                config=config,
                logger=logger
            )
            self._controllers.append(controller)

    def start(self):
        """Start all cameras in the group."""
        for controller in self._controllers:
            controller.start()

    def stop(self):
        """Stop all cameras and release resources."""
        for controller in self._controllers:
            controller.stop()

    @property
    def names(self):
        """Get list of camera names in the group.

        Returns:
            list[str]: List of camera names
        """
        return [c.name for c in self._controllers]


class TeleopController:
  """Controls teleoperation functionality between leader and follower robots.
  
  This class manages the interaction between one or more leader (Pollo) and follower (Piper)
  robot pairs, handling initialization, engagement, and control loop execution.
  """
  def __init__(self, config: dict[str, Any] | list[dict[str, Any]], logger: episode_logging.Logger):
    """Initialize the teleoperation controller.

    Args:
      config: Either a dict for a single pair, or a list of dicts for multiple pairs.
      logger: Logger instance for recording robot states and events.
    """
    self._logger = logger
    self._running = False
    self._thread = None

    # Support both single and multiple pairs
    if isinstance(config, dict):
      config = [config]

    self._pairs = []
    for pair_cfg in config:
      pollo_inst = pollo.PolloReceiver(config=pair_cfg["pollo"])
      piper_inst = piper_lib.Piper(config=pair_cfg["piper"])
      self._pairs.append((pollo_inst, piper_inst))

  def move_follower_to_init(self):
    """Move all follower robots to their initial positions."""
    ## revisit
    for _, piper in self._pairs:
      piper.start()
      piper.enable_motion()
      piper.command_joints(np.zeros(7))

  def wait_leader_to_init(self):
    """Wait for all leader robots to reach their initial positions."""
    ensure_joints_synced(self._pairs, match_piper_to_pollo=False)

  def engage(self):
    """Start teleoperation by engaging all robot pairs.
    
    This method:
    1. Enables MIT mode on all followers
    2. Gradually matches follower position to leader
    3. Starts the control loop thread
    """
    self._running = True
    for _, piper in self._pairs:
      piper.enter_mit_mode()
    print("matching joint")
    # assume engage with joint near 0
    for i in range(100):
      for pollo_inst, piper_inst in self._pairs:
        joints = pollo_inst.sensed_joints()
        piper_inst.command_joints(joints * i / 100)
      time.sleep(0.01)
    for i in range(10):
      for pollo_inst, piper_inst in self._pairs:
        joints = pollo_inst.sensed_joints()
        piper_inst.command_joints(joints)
      time.sleep(0.01)
    print("matching done")

    self._thread = threading.Thread(target=self.run_control_loop)
    self._thread.daemon = True
    self._thread.start()

  def disengage(self):
    """Stop teleoperation by disengaging all robot pairs.
    
    This method:
    1. Stops the control loop
    2. Gradually moves all followers back to zero position
    3. Exits MIT mode
    """
    self._running = False
    if self._thread:
      self._thread.join()
    j0s = [piper.sensed_joints() for _, piper in self._pairs]
    for i in np.arange(1, 0, -0.01):
      for idx, (_, piper) in enumerate(self._pairs):
        piper.command_joints(j0s[idx] * i)
      time.sleep(0.01)
    for _ in range(100):
      for idx, (_, piper) in enumerate(self._pairs):
        piper.command_joints(j0s[idx] * 0)
      time.sleep(0.01)
    for _, piper in self._pairs:
      piper.enter_mit_mode(False)

  def move_follower_to_rest(self):
    """Move all follower robots to their rest positions."""
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3
    for _, piper in self._pairs:
      piper.command_joints(rest_joints)
    time.sleep(1.5)

  def close(self):
    """Clean up resources and close connections to all robots."""
    if self._running:
      self.disengage()
    for pollo_inst, piper_inst in self._pairs:
      piper_inst.disable_motion()
      piper_inst.close()
      pollo_inst.close()

  def run_control_loop(self):
    """Main control loop that runs teleoperation for all pairs.
    
    Continuously:
    1. Reads joint positions from each leader
    2. Commands each follower to match positions
    3. Logs commanded and observed joint positions for each pair
    """
    metronome = Metronome(100)
    while self._running:
      ts = time.time_ns()
      # Gather and clip joints from all pollos
      observed_joints = []
      for pollo_inst, _ in self._pairs:
        joints = pollo_inst.sensed_joints()
        observed_joints.append(joints.copy())

      # Command all pipers with their respective joints
      ts_commanded = time.time_ns()
      for idx, (_, piper_inst) in enumerate(self._pairs):
        piper_inst.command_joints(observed_joints[idx])

      # Log concatenated commanded joints
      concat_observed = [item for joints in observed_joints for item in joints]
      observed_event = episode_logging.LogEvent(
        timestamp_ns=ts,
        event_type=episode_logging.EventType.JOINT_POSITION,
        event_name="observed",
        joint_pos=concat_observed,
      )
      concat_commanded = [item for joints in observed_joints for item in joints]
      commanded_event = episode_logging.LogEvent(
        timestamp_ns=ts_commanded,
        event_type=episode_logging.EventType.JOINT_POSITION,
        event_name="commanded",
        joint_pos=concat_commanded,
      )

      self._logger.log_event(observed_event)
      self._logger.log_event(commanded_event)

      metronome.wait()
