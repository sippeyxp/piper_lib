import time
import episode_logging


def test_camera():
  logger = lambda: None
  logger.t0 = None
  logger.sn = 0
  recorder = episode_logging.VideoRecorder("test.mp4", 30)

  def log_event(event):
    if logger.t0 is None:
      logger.t0 = event.timestamp_ns
    print(logger.sn, (event.timestamp_ns - logger.t0)/1e9, event.image.shape)
    recorder.add_frame(event.image)
    logger.sn += 1
  logger.log_event = log_event

  # Create camera controller
  camera = episode_logging.CameraController(config={
    "camera_name": "test", "camera_id": 4, "fps": 30.0}, logger=logger)

  try:
    print("Starting camera capture (3 seconds test)...")
    camera.start()
    time.sleep(3)  # Run for 3 seconds

  except KeyboardInterrupt:
    print("\nTest interrupted by user")
  finally:
    print("Stopping camera...")
    camera.stop()
    recorder.close()

if __name__ == "__main__":
  test_camera()
