"""
Usage: python3 test_multicam_fps.py --config=configs/p620_collect.json
"""

import cv2
import time
import json
import threading
from absl import app
from absl import flags
import teleop_lib
from collections import deque

FLAGS = flags.FLAGS
flags.DEFINE_string('config_file', None, 'Path to camera config json file', required=True)


class FrameGrabber:
    def __init__(self, buffer_size=100):
        self._frame_buffers = {}
        self._buffer_size = buffer_size
        self._locks = {}
        
    def log_event(self, event):
        if event.event_type == "image":
            if event.event_name not in self._locks:
                self._locks[event.event_name] = threading.Lock()
                
            with self._locks[event.event_name]:
                if event.event_name not in self._frame_buffers:
                    self._frame_buffers[event.event_name] = deque(maxlen=self._buffer_size)
                
                self._frame_buffers[event.event_name].append({
                    'timestamp': event.timestamp_ns/1e9,
                    'image': event.image.copy()
                })
            
    def get_frames(self):
        frames = {}
        camera_names = list(self._frame_buffers.keys())
        for camera_name in camera_names:
            with self._locks[camera_name]:
                buffer = self._frame_buffers[camera_name]
                if buffer:  # If buffer is not emptyz
                    frames[camera_name] = list(buffer)  # Get all frames in buffer
                    buffer.clear()  # Clear buffer after getting frames
        return frames

def main(argv):
    # Load camera settings from config file
    with open(FLAGS.config_file, 'r') as f:
        config = json.load(f)

    frame_grabber = FrameGrabber()

    camera_config = config.get("cameras", config.get("camera", None))
    if not camera_config:
        raise RuntimeError("Cannot find valid camera config")
    camera_group = teleop_lib.CameraGroupController(
        camera_configs=camera_config,
        logger=frame_grabber
    )
    camera_group.start()

    try:
        fps_dict = {}
        frame_times = {}
        while True:
            frames = frame_grabber.get_frames()
            
            for camera_name, frame_list in frames.items():
                if camera_name not in frame_times:
                    frame_times[camera_name] = []
                
                for frame_data in frame_list:
                    frame_times[camera_name].append(frame_data['timestamp'])
                    if len(frame_times[camera_name]) > 30:
                        frame_times[camera_name].pop(0)
                
                if len(frame_times[camera_name]) >= 2:
                    fps = len(frame_times[camera_name])/(frame_times[camera_name][-1] - frame_times[camera_name][0])
                    fps_dict[camera_name] = fps
                
                # Show the most recent frame
                if frame_list:
                    cv2.imshow(camera_name, frame_list[-1]['image'])
            
            if fps_dict:
                fps_str = ", ".join([f"{name}: {fps_dict[name]:.1f}" for name in fps_dict])
                print(f"\rFPS - {fps_str:80s}", end="")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nStopping cameras...")
    
    finally:
        camera_group.stop()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    app.run(main)
