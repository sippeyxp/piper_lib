"""Receives joint readings from Pollo via serial."""

import numpy as np
import threading
import serial
import sys
from typing import Any


class PolloReceiver:
  """Receives joint readings from Pollo via serial."""

  DEFAULT_CONFIG = {
    "port": "/dev/ttyACM0",
    "baudrate": 115200,
    "int_zeros": [0x750,0xfff,0xfff,0x880,0x850,0x850,0x300],
    "k_conv": [
      -np.deg2rad(300)/4096*0.66, 
      -np.deg2rad(300)/4096*0.66, 
      np.deg2rad(300)/4096*0.66, 
      -np.deg2rad(300)/4096*0.66, 
      np.deg2rad(300)/4096*0.66, 
      -np.deg2rad(300)/4096*0.66,
      0.08/3000
    ]
  }

  def __init__(self, config: dict[str, Any]):
    # Use default config if none provided, otherwise update defaults with provided config
    self._config = self.DEFAULT_CONFIG.copy()
    self._config.update(config)

    self._serial = serial.Serial(
      port=self._config["port"],
      baudrate=self._config["baudrate"],
      timeout=1  # Read timeout in seconds
    )

    self._reading = np.zeros(7)

    self._thread = threading.Thread(target=self._serial_thread)
    self._thread.daemon = True
    self._running = True
    self._thread.start()

    self._int_zeros = np.array(self._config["int_zeros"])
    self._k_conv = np.array(self._config["k_conv"])


  def sensed_joints(self):
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

    try:
      int_values = np.array([int(i, 16) for i in values])
      joints = self._k_conv * (int_values - self._int_zeros)
      joints *= [1,1,1,1,1,1,1]
      return joints
    except Exception as e:
      print("something wrong when parsing Pollo reading", e)
      return self._reading

  def _serial_thread(self):
    try:
      while self._running:
        line_bytes = self._serial.readline()
        self._reading = np.array(self._parse_reading(line_bytes))
    except Exception as e:
      print("serial thread exited !!!!", e)
      sys.exit(1)

  def close(self):
    self._running = False
    self._thread.join()