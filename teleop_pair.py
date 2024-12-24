#!/usr/bin/env python3
# -*-coding:utf8-*-
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



def ensure_joints_synced(piper_lead, piper):
  # make sure pollo is not too far from zero
  n = 0
  nn = 0
  while True:
    joints = piper_lead.sensed()
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

  print("pollo zeroed, starting in 2 sec")
  for i in range(100):
    joints = piper_lead.sensed()
    piper.command_joints(joints * i / 100)
    time.sleep(0.01)


def main(argv):
  # pollo = PolloReceiver()
  piper_lead = piper_lib.Piper("can_leader", leader_setup=True)
  piper_lead.start()
  print("lead started")


  piper = piper_lib.Piper("can_follower")
  piper.start()
  piper.enable_motion()
  piper.command_joints(np.zeros(7))
 
  # Sync joints before start
  ensure_joints_synced(piper_lead, piper)

  piper.enter_mit_mode()

  try:
    metronome = piper_lib.Metronome(100)
    while True:
      # joints = pollo.get_reading()
      joints = piper_lead.sensed()
      piper.command_joints(joints)
      metronome.wait() 
  except KeyboardInterrupt:
    piper.enter_mit_mode(False) # exit mit mode
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3
    piper.command_joints(rest_joints)
    time.sleep(1.5)

  piper.disable_motion()
  piper.close()

if __name__ == "__main__":
    app.run(main)
