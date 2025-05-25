#!/usr/bin/env python3
# -*-coding:utf8-*-
#
# Example:
#   python3 eval.py  --config_file=configs/home_eval_config.json  \
#     --checkpoint_path=/host/tmp/checkpoint_10000/
#

from typing import (
    Optional,
)
from absl import app
from absl import flags
import json
import piper as piper_lib
import time
import numpy as np
import sys
import teleop_lib
import episode_logging
from timing import Metronome

from model import easy_policy

flags.DEFINE_string("config_file", None, "Path to config file.", required=True)
flags.DEFINE_string("checkpoint_path", None, "Path to checkpoint", required=True)
flags.DEFINE_integer("control_freq_hz", 100, "Control frequency in Hz")
flags.DEFINE_string("episode_log_dir", None, "Directory to save episode logs.")
flags.DEFINE_enum("model_type", "diffusion", ["diffusion", "act"], "Type of model")
flags.DEFINE_bool(
    "use_mit_mode",
    True,
    "Use the faster dynamic control. For testing new model, set this to false for additional safety",
)
FLAGS = flags.FLAGS


class FrameGrabberLoggerWrapper:

  def __init__(self, logger: Optional[episode_logging.Logger] = None, wait_for_frame=True):
    # super().__init__(base_dir=None)
    self._latest_frame = None
    self._logger = logger

  def create_episode(self):
    # override
    if not self._logger:
      class DummyContext:
        def __enter__(self):
          return self
        def __exit__(self, exc_type, exc_val, exc_tb):
          pass
      return DummyContext()
    return self._logger.create_episode()

  def attach(self, context: episode_logging.EpisodeContext):
    # overrdide
    if not self._logger:
      raise RuntimeError("logger is not set, not expecting this to be called.")
    return self._logger.attach(context)

  def detach(self, context: episode_logging.EpisodeContext):
    # override
    if not self._logger:
      raise RuntimeError("logger is not set, not expecting this to be called.")
    return self._logger.detach(context)

  def log_event(self, event: episode_logging.LogEvent):
    if event.event_type == "image":
      self._latest_frame = event.image.copy()
    if self._logger:
      self._logger.log_event(event)

  def get_latest_frame(self):
    # Atomic opearation no need to lock
    return self._latest_frame

  def wait_until_frame(self):
    while True:
      if self._latest_frame is not None:
        break
      print("wait for frame")
      time.sleep(0.1)

  def clear_frame(self):
    self._latest_frame = None


def main(argv):
  # pollo = PolloReceiver()
  with open(FLAGS.config_file, "r") as f:
    config = json.load(f)

  logger = None
  if FLAGS.episode_log_dir:
    logger = episode_logging.Logger(FLAGS.episode_log_dir)

  frame_grabber_and_logger = FrameGrabberLoggerWrapper(logger=logger)
  camera_controller = teleop_lib.CameraController(config=config["camera"], logger=frame_grabber_and_logger)
  camera_controller.start()

  frame_grabber_and_logger.wait_until_frame()

  piper = piper_lib.Piper(config=config["robot"]["piper"])
  piper.start()
  piper.enable_motion()
  piper.command_joints(np.zeros(7))

  if FLAGS.use_mit_mode:
    time.sleep(0.5)
    piper.enter_mit_mode()

  match FLAGS.model_type:
    case "diffusion":
      policy = easy_policy.load_diffusion_policy(FLAGS.checkpoint_path, warm_up=False)
    case "act":
      policy = easy_policy.load_act_policy(FLAGS.checkpoint_path, warm_up=False)
    case _:
      raise ValueError(f"Unknown model type: {FLAGS.model_type}")

  policy.reset()

  try:
    metronome = Metronome(FLAGS.control_freq_hz)

    with frame_grabber_and_logger.create_episode() as episode:
      # episode.set_attribute("task", FLAGS.task)
      while True:
        image = frame_grabber_and_logger.get_latest_frame()
        joints = piper.sensed_joints()
        ts_sensed_joints = time.time_ns()

        observation = {
          "right_wrist_cam": image,
          "joints_pos": joints,
          # ? need task id?
        }
        command = policy.action(observation)
        piper.command_joints(command)
        ts_command_joints = time.time_ns()

        # log joints, camera images already logged in camera controller.
        event = episode_logging.LogEvent(
          timestamp_ns=ts_sensed_joints,
          event_type=episode_logging.EventType.JOINT_POSITION,
          event_name="observed",
          joint_pos=joints.tolist()
        )
        frame_grabber_and_logger.log_event(event)

        event = episode_logging.LogEvent(
          timestamp_ns=ts_command_joints,
          event_type=episode_logging.EventType.JOINT_POSITION,
          event_name="commanded",
          joint_pos=command.tolist()
        )
        frame_grabber_and_logger.log_event(event)

        metronome.wait()
  except KeyboardInterrupt:
    joints = piper.sensed_joints()
    for i in np.linspace(1, 0, FLAGS.control_freq_hz * 2):
      piper.command_joints(i * joints)
      metronome.wait()

    piper.enter_mit_mode(False) # exit mit mode
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3
    piper.command_joints(rest_joints)
    time.sleep(1.5)

  piper.disable_motion()
  piper.close()
  camera_controller.stop()

if __name__ == "__main__":
  app.run(main)
