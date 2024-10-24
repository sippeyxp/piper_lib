#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import (
    Optional,
)
#from piper_sdk import *

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

        self._int_zeros = np.array([0x25a,0x042,0x2d0,0x1d0,0x153,0x241,0x0])
        self._k_conv = np.array([
            -np.deg2rad(300)/1024, 
            np.deg2rad(300)/1024, 
            np.deg2rad(300)/1024, 
            -np.deg2rad(300)/1024, 
            np.deg2rad(300)/1024, 
            -np.deg2rad(300)/1024,
            0.08/1024
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

        if len(line) != 23:
            print("wrong line length ", len(line))
            return self._reading

        values = line.split(",")
        if len(values) != 6:
            print("len !=6")
            return self._reading
        
        # adding missing gripper
        values.append("0")

        try:
            int_values = np.array([int(i, 16) for i in values])
            joints = self._k_conv * (int_values - self._int_zeros)
            joints *= [1, 1,1,1,1,1,0]
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

def main(argv):
  pollo = PolloReceiver()

  piper = piper_lib.Piper()
  piper.start()

  piper.enable_motion()

  t0 = time.time()

  try:
    while True:
      t = time.time() - t0
      joints = pollo.get_reading()
      piper.command_joints(joints)
      time.sleep(0.01)
  except KeyboardInterrupt:
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3
    piper.command_joints(rest_joints)
    time.sleep(3)

  piper.disable_motion()
  piper.close()
    
if __name__ == "__main__":
    app.run(main)
