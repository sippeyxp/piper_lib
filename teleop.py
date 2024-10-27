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


class PolloReceiver:

    def __init__(self):
        self._serial = serial.Serial(
            port='/dev/ttyACM0',        # Replace with your Arduino's serial port
            baudrate=115200,      # Must match the Arduino's baudrate
            timeout=1           # Read timeout in seconds
        )

        self._reading = np.zeros(7)

        self._thread = threading.Thread(target=self._serial_thread)
        self._thread.daemon = True
        self._thread.start()

        self._int_zeros = np.array([0x750,0x250,0xe00,0x880,0x850,0x850,0x300])
        self._k_conv = np.array([
            -np.deg2rad(300)/4096, 
            np.deg2rad(300)/4096, 
            np.deg2rad(300)/4096, 
            -np.deg2rad(300)/4096, 
            np.deg2rad(300)/4096, 
            -np.deg2rad(300)/4096,
            0.08/3000
            ])


    def get_reading(self):
        return self._reading

    def _parse_reading(self, line_bytes):
        try:
            line = line_bytes.decode("utf-8").rstrip()
            if not line:
                return self._reading
        except:
            return self._reading

        if len(line) != 27:
            print("wrong line length ", len(line))
            return self._reading

        values = line.split(",")
        if len(values) !=7:
            print("len != 7")
            return self._reading
        
        # adding missing gripper
        # values.append("0")

        try:
            int_values = np.array([int(i, 16) for i in values])
            joints = self._k_conv * (int_values - self._int_zeros)
            joints *= [1, 1,1,1,1,1,1]
            return joints
        except Exception as e:
            print("something wrong", e)
            return self._reading

        # return [int_values[0] / 700 *np.pi - np.pi/2, 0, 0, 0, 0, 0, int_values[0]/700 *0.08]

    def _serial_thread(self):
        try:
            while True:
                #if self._serial.in_waiting > 0:
                line_bytes = self._serial.readline()
                # print("l = ", len(line_bytes))
                self._reading = np.array(self._parse_reading(line_bytes))
                #else:
                #    time.sleep(0.001)
        except Exception as e:
            print("serial thread exited !!!!", e)
            sys.exit(1)

def ensure_joints_synced(pollo, piper):
  # make sure pollo is not too far from zero
  n = 0
  nn = 0
  while True:
    joints = pollo.get_reading()
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
    joints = pollo.get_reading()
    piper.command_joints(joints * i / 100)
    time.sleep(0.01)


def main(argv):
  pollo = PolloReceiver()

  piper = piper_lib.Piper()
  piper.start()
  piper.enable_motion()
  piper.command_joints(np.zeros(7))
 
  # Sync joints before start
  ensure_joints_synced(pollo, piper)

  piper.enter_mit_mode()

  try:
    metronome = piper_lib.Metronome(100)
    while True:
      joints = pollo.get_reading()
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
