#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import (
  Optional,
  Any
)
from absl import app
import piper as piper_lib
import time
import numpy as np
import pollo as pollo_lib


def _ensure_joints_synced(pollo: pollo_lib.PolloReceiver, piper: piper_lib.Piper, match_piper_to_pollo=True):
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
      joints = pollo.sensed_joints()
      piper.command_joints(joints * i / 100)
      time.sleep(0.01)
    print("started")

def main(argv):
  del argv
  pollo = pollo_lib.PolloReceiver(config={"port": "/dev/ttyACM0", "baudrate": 115200})

  piper = piper_lib.Piper(config={"can_name": "can0"})
  piper.start()
  piper.enable_motion()
  piper.command_joints(np.zeros(7))
 
  # Sync joints before start
  _ensure_joints_synced(pollo, piper)

  piper.enter_mit_mode()

  try:
    metronome = piper_lib.Metronome(100)
    while True:
      joints = pollo.sensed_joints()
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
