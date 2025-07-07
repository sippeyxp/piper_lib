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
from timing import Metronome


def main(argv):
  del argv

  piper = piper_lib.Piper(config={"can_name": "can0"})
  piper.start()

  while True:
    print(piper.sensed_joints())
    time.sleep(0.1)

  piper.close()

if __name__ == "__main__":
  app.run(main)
