from absl import app
from absl import flags
import piper as piper_lib
import numpy as np
import time


FLAGS = flags.FLAGS

flags.DEFINE_enum("trajectory", "test", ["test", "j6_10deg_1hz", "j6_10deg_halfhz"], "Preset trajectories")
flags.DEFINE_string("log_file", "log.txt", "Output log file path")
flags.DEFINE_bool("mit_mode", True, "Whether to turn on the MIT mode")


def main(argv):
  # testing code
  piper = piper_lib.Piper()
  piper.start()

  piper.enable_motion()

  print("Moving to zero")
  piper.command_joints(np.zeros(7))
  time.sleep(2)
  print("Moving to zero done")

  if FLAGS.mit_mode:
    print("Set MIT mode")
    piper.enter_mit_mode()

  mode = FLAGS.trajectory
  match mode:
    case "test":
      A = [0.3, 0.3, 0.4, 0.4, 0.4, -0.4, 0.04]
      phi = [0, 3* np.pi/2, np.pi/2, 0, 0, 0, 0]
      b = [0, 0.3, -0.4, 0.0, 0.0, 0.0, 0]
      w = 2 * np.pi / 2
    case "j6_10deg_1hz":
      A = [0, 0, 0, 0, 0, np.deg2rad(10), 0]
      phi = np.zeros(7)
      b = np.zeros(7)
      w = 2 * np.pi
    case "j6_10deg_halfhz":
      A = [0, 0, 0, 0, 0, np.deg2rad(10), 0]
      phi = np.zeros(7)
      b = np.zeros(7)
      w = 2 * np.pi / 2
    case _:
      raise ValueError(f"unknown trajectory {mode}")

  try:
    with open(FLAGS.log_file, "w") as f:
      metronome = piper_lib.Metronome(100)
      while True:
        t = metronome.t
        joints_sensed = piper.sensed()
        joints_command = A * np.sin(w * t + np.array(phi)) + b
        piper.command_joints(joints_command)

        f.write("%0.3f %s %s\n" % (
            t,
            " ".join(f"{i:.3f}" for i in joints_sensed),
            " ".join(f"{i:.3f}" for i in joints_command),
          )
        )
        # This does not actually reflect the actual setting for some reason.
        # Sett
        #if metronome.i % 100 == 0:
        #  print("Arm status", piper.get_arm_status())
        #  print("Ctrl Mode 2", piper.get_motion_ctrl2())

        metronome.wait()
  except KeyboardInterrupt:
    if FLAGS.mit_mode:
      piper.enter_mit_mode(False) # exit mit mode
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3 # j5 sag a bit naturally.
    piper.command_joints(rest_joints)
    time.sleep(2)

  # Turn off.
  piper.disable_motion()
  piper.close()


if __name__ == "__main__":
  app.run(main)
