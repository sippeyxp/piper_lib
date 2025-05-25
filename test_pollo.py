import pollo as pollo_lib
import json
import serial
import time


from absl import flags
from absl import app

FLAGS = flags.FLAGS

flags.DEFINE_string("port", "/dev/ttyACM0", "Pollo port.")
flags.DEFINE_integer("baudrate", 115200, "Pollo baudrate.")
flags.DEFINE_enum("mode", "joints", ["joints", "raw"], "Mode.")
flags.DEFINE_string("config_file", None, "Path to JSON config file for PolloReceiver.")
flags.DEFINE_string("config_path", None, "Path to JSON config file for PolloReceiver.")

def main(argv):
  del argv
  
  if FLAGS.config_file:
    with open(FLAGS.config_file, "r") as f:
      config = json.load(f)
    if FLAGS.config_path:
      for i in FLAGS.config_path.split("/"):
        config = config[int(i) if i.isdigit() else i] 
  else:
    config = {"port": FLAGS.port, "baudrate": FLAGS.baudrate}

  if FLAGS.mode == "joints":
    pollo = pollo_lib.PolloReceiver(config=config)

    while True:
      joints = pollo.sensed_joints()
      print("pollo", " ".join(f"{i:.2f}" for i in joints))
      time.sleep(0.1)
  else:
    serial_conn = serial.Serial(config["port"], config["baudrate"], timeout=0.01)
    while True:
      line = serial_conn.readline()
      if line:
        print(line.decode("utf-8").rstrip("\n"))

if __name__ == "__main__":
  app.run(main)
