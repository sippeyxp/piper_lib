"""Collects data from the robot."""

import json

from absl import app
from absl import flags

import episode_logging
import teleop_lib

from pynput.keyboard import Key, Listener


FLAGS = flags.FLAGS

flags.DEFINE_string("episode_log_dir", None, "Directory to save episode logs.", required=True)

flags.DEFINE_string("task", "teleop_task", "Task name.")

flags.DEFINE_integer("max_episode_length_sec", 600, "Max episode length in seconds.")

flags.DEFINE_string("config_file", None, "Path to config file.", required=True)

def main(argv):
  logger = episode_logging.Logger(FLAGS.episode_log_dir)

  with open(FLAGS.config_file, "r") as f:
    config = json.load(f)
  # set up camera group from config
  camera_config = config.get("cameras", config.get("camera", None))
  if not camera_config:
    raise RuntimeError("Cannot find valid camera config")
  camera_group = teleop_lib.CameraGroupController(
    camera_configs=camera_config,
    logger=logger
  )
  camera_group.start()

  teleop = teleop_lib.TeleopController(config=config["teleop"], logger=logger)

  while True:
    init_ok = False
    try:
      teleop.move_follower_to_init()
      teleop.wait_leader_to_init()
      init_ok = True
    except KeyboardInterrupt:
      break

    if init_ok:
      teleop.engage()
      print("Press mid pedal to start episode")
      try:
        with Listener(on_press=lambda key: key != Key.tab) as listener:
          listener.join()
      except KeyboardInterrupt:
        break

      with logger.create_episode() as episode:
        print("Recording started \a")
        episode.set_attribute("task", FLAGS.task)
        episode.set_attribute("cameras", camera_group.names)
        try:
          success = [None]
          def on_press(key):
            if key == Key.backspace:
              success[0] = False
              return False
            elif key == Key.enter:
              success[0] = True
              return False
          with Listener(on_press=on_press) as listener:
            listener.join()
        except KeyboardInterrupt:
          success[0] = None
          break
        finally:
          teleop.disengage()

          print("success = ", success[0])
          success = success[0]
          episode.set_attribute("success", success)

  camera_group.stop()
  teleop.move_follower_to_rest()
  teleop.close()

if __name__ == "__main__":
  app.run(main)
