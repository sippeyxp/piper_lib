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

from model import easy_policy

flags.DEFINE_string("config_file", None, "Path to config file.", required=True)
flags.DEFINE_string("checkpoint_path", None, "Path to checkpoint", required=True)

FLAGS = flags.FLAGS


class FrameGrabber(episode_logging.Logger):

  def __init__(self, wait_for_frame=True):
    super().__init__(base_dir=None)
    self._latest_frame = None

  def log_event(self, event: episode_logging.LogEvent):
    if event.event_type == "image":
      self._latest_frame = event.image.copy()

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

  frame_grabber = FrameGrabber()
  camera_controller = teleop_lib.CameraController(config=config["camera"], logger=frame_grabber)
  camera_controller.start()
  frame_grabber.wait_until_frame()

  piper = piper_lib.Piper(config=config["robot"]["piper"])
  piper.start()
  piper.enable_motion()
  piper.command_joints(np.zeros(7))
  # piper.enter_mit_mode()

  policy = easy_policy.load_diffusion_policy(FLAGS.checkpoint_path, warm_up=False)
  policy.reset()

  try:
    metronome = piper_lib.Metronome(100)
    while True:
      # joints = pollo.get_reading()
      image = frame_grabber.get_latest_frame()
      joints = piper.sensed_joints()
      observation = {
        "right_wrist_cam": image,
        "joints_pos": joints,
        }
      # import pdb;pdb.set_trace()
      command = policy.action(observation)
      # for i in range(100):
      #   command_interp = i/100 * joints + (1-i/100) * command
      #   piper.command_joints(command_interp)
      #   time.sleep(0.01)
      piper.command_joints(command)
      metronome.wait() 
  except KeyboardInterrupt:
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
