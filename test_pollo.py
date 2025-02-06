import pollo as pollo_lib
import serial
import time


from absl import flags
from absl import app

FLAGS = flags.FLAGS

flags.DEFINE_string("port", "/dev/ttyACM0", "Pollo port.")
flags.DEFINE_integer("baudrate", 115200, "Pollo baudrate.")
flags.DEFINE_enum("mode", "joints", ["joints", "raw"], "Mode.")

def main(argv):
  del argv
  
  if FLAGS.mode == "joints":
    pollo = pollo_lib.PolloReceiver(config={"port": FLAGS.port, "baudrate": FLAGS.baudrate})

    while True:
      joints = pollo.sensed_joints()
      print("pollo", " ".join(f"{i:.2f}" for i in joints))
      time.sleep(0.1)
  else:
    serial_conn = serial.Serial(FLAGS.port, FLAGS.baudrate, timeout=0.01)
    while True:
      line = serial_conn.readline()
      if line:
        print(line.decode("utf-8").rstrip("\n"))

if __name__ == "__main__":
  app.run(main)
