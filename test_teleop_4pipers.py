#!/usr/bin/env python3
# -*-coding:utf8-*-

# Setup: config the piper can id using the command below.
# The bus id can be figured out by connecting on device a time.
#
# bash can_activate.sh can_ll 1000000  1-2.1:1.0  # left leader
# bash can_activate.sh can_lf 1000000  1-2.2:1.0  # left follower
# bash can_activate.sh can_rl 1000000  1-5.1:1.0  # right leader
# bash can_activate.sh can_rf 1000000  1-5.2:1.0  # right follower
#

from typing import (
    Optional,
)
from absl import app
import piper as piper_lib
import time
import numpy as np
import threading
import serial
import sys



def ensure_joints_synced(piper_leads, pipers):
  # make sure pollo is not too far from zero
  n = 0
  nn = 0
  while True:
    joints_0 = piper_leads[0].sensed_joints()
    joints_1 = piper_leads[1].sensed_joints()
    if nn % 20 == 0:
      print("leader0", " ".join(f"{i:.2f}" for i in joints_0))
      print("leader1", " ".join(f"{i:.2f}" for i in joints_1))
      print()

    is_zeroed = (np.all(np.abs(joints_0[:3]) < 0.3) and np.all(np.abs(joints_0[3:6]) < 0.4)
                and np.all(np.abs(joints_1[:3]) < 0.3) and np.all(np.abs(joints_1[3:6]) < 0.4))
    if is_zeroed:
      n += 1
    else:
      n = 0
    if n > 100:
      break
    time.sleep(0.01)
    nn += 1

  print("leader zeroed, starting in a little bit, do not move yet")
  for i in range(100):
    joints_0 = piper_leads[0].sensed_joints()
    joints_1 = piper_leads[1].sensed_joints()
    pipers[0].command_joints(joints_0 * i / 100)
    pipers[1].command_joints(joints_1 * i / 100)
    time.sleep(0.01)

  print("alignment finished")


def main(argv):
  # pollo = PolloReceiver()
  piper_left_lead = piper_lib.Piper(config={"can_name": "can_ll", "is_leader": True})
  piper_left_lead.start()
  piper_right_lead = piper_lib.Piper(config={"can_name": "can_rl", "is_leader": True})
  piper_right_lead.start()
  print("leaders started")


  piper_left = piper_lib.Piper(config={"can_name": "can_lf", "is_leader": False}
  piper_left.start()
  piper_right = piper_lib.Piper(config={"can_name": "can_rf", "is_leader": False}
  piper_right.start()
  print("follower started")

  piper_left.enable_motion()
  piper_right.enable_motion()
  piper_left.command_joints(np.zeros(7))
  piper_right.command_joints(np.zeros(7))
  print("follower zeroed")

  # Sync joints before start
  ensure_joints_synced(piper_lead, piper)

  piper_left.enter_mit_mode()
  piper_right.enter_mit_mode()

  try:
    metronome = piper_lib.Metronome(100)
    while True:
      # joints = pollo.get_reading()
      left_joints = piper_left_lead.sensed_joints()
      right_joints = piper_right_lead.sensed_joints()
      piper_left.command_joints(left_joints)
      piper_right.command_joints(right_joints)
      metronome.wait() 
  except KeyboardInterrupt:
    left_piper.enter_mit_mode(False) # exit mit mode
    right_piper.enter_mit_mode(False) # exit mit mode
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3
    left_piper.command_joints(rest_joints)
    right_piper.command_joints(rest_joints)
    time.sleep(1.5)

  left_piper.disable_motion()
  left_piper.close()
  right_piper.disable_motion()
  right_piper.close()

if __name__ == "__main__":
    app.run(main)
