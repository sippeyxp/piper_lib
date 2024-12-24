
import time
import teleop_lib

def test_teleop():
  logger = lambda: None
  logger.log_event = lambda event: print(
    f"Logged event: type={event.event_type}, "
    f"name={event.event_name}, "
    f"timestamp={event.timestamp_ns/1e9}"
  )  
    
  teleop = teleop_lib.TeleopController(
    config={
      "pollo": {
        "port": "/dev/ttyACM0",
        "baudrate": 115200
      },
      "piper": {
        "can_name": "can0"
      }
    },
    logger=logger
  )
  teleop.move_follower_to_init()

  init_ok = False
  try:
    teleop.wait_leader_to_init()
    init_ok = True
  except KeyboardInterrupt:
    pass

  if init_ok:
    teleop.engage()

    try:
      while True: time.sleep(0.1)
    except KeyboardInterrupt:
      pass
    finally:
      teleop.disengage()
    teleop.move_follower_to_rest()
  teleop.close()

if __name__ == "__main__":
  test_teleop()