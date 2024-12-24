"""Event level logging for teleoperation episodes."""

import cv2
import dataclasses
import datetime
import json
import os
import threading
import time
from typing import Any, Optional
import enum


class EventType:
    """Event types for logging system."""
    INVALID = "invalid"
    JOINT_POSITION = "joint_pos" 
    IMAGE = "image"
    VIDEO_FRAME = "video_frame"


@dataclasses.dataclass
class LogEvent:
    """
    Data class representing a loggable event.

    Attributes:
        timestamp_ns: Timestamp in nanoseconds
        event_type: Type of event (e.g. JOINT_POSITION, IMAGE)
        event_name: Name of the event
        image: Optional image data
        joints_p: Optional joint position data
        frame_id: Optional frame ID for video events
    """
    timestamp_ns: int = 0
    event_type: str = EventType.INVALID
    event_name: str = "unknown"

    # if event_type == "image"
    image: Any = None

    # if event_type == "joint_pos"
    joint_pos: Any = None

    # if event_type == "video_frame"
    # == video {event_name}.mp4::frame_{video_frame} 0-based
    video_frame: int = None

    def as_dict(self):
        """Convert to a dictionary."""
        return dataclasses.asdict(self)
    
    def as_json_string(self):
        """Convert to a JSON string."""
        return json.dumps(self.as_dict())


class VideoRecorder:
  """Records video frames to an MP4 file."""

  def __init__(self, output_path: str, fps: float = 30):
    """
    Initialize the video recorder.

    Args:
      output_path: Path to save the video file
      fps: Frames per second (default: 30)
    """
    self._output_path = output_path
    self._writer = None
    self._fps = fps
    self._frame_count = 0
  
  def add_frame(self, frame):
    """
    Add a frame to the video.

    Args:
      frame: Image frame to add

    Returns:
      int: Frame ID of the added frame
    """
    if self._writer is None:
      height, width = frame.shape[:2]
      fourcc = cv2.VideoWriter_fourcc(*'mp4v')
      self._writer = cv2.VideoWriter(self._output_path, fourcc, self._fps, (width, height))
    self._writer.write(frame)
    self._frame_count += 1
    return self._frame_count - 1
  
  def close(self):
    """Close the video writer and release resources."""
    if self._writer:
      self._writer.release()


class EpisodeContext:
  """
  Context manager for logging an episode.
  """
  def __init__(self, logger, episode_dir: str):
    """
    Initialize episode context.

    Args:
      logger: Parent logger instance
      episode_dir: Directory to save episode data
    """
    self._logger = logger
    self._enabled = False
    self._video_recorder = {}
    self._video_num_frames = {}
    self._logs = []
    self._metadata = {}
    self._episode_dir = episode_dir

  def __enter__(self):
    """Enter the context manager."""
    self._logger.attach(self)
    self._enabled = True
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    """Exit the context manager."""
    del exc_type, exc_val, exc_tb
    self._enabled = False
    self._logger.detach(self)
    self.save_log()

  def set_attribute(self, key: str, value: Any):
    """
    Set episode metadata attribute.

    Args:
      key: Attribute key
      value: Attribute value
    """
    self._metadata[key] = value

  def log_event(self, event: LogEvent):
    """
    Log an event.

    Args:
      event: Event to log

    Raises:
      ValueError: If event type is not implemented
    """
    if not self._enabled:
      return
    if event.event_type == "joint_pos":
      if not isinstance(event.joint_pos, list):
        print("ignore event")
      self.log_vector(event)
    elif event.event_type == "image":
      self.log_video(event)
    else:
      raise ValueError(f"logging type {event.event_type} not implemented")

  def log_video(self, event: LogEvent):
    """
    Log a video frame event.

    Args:
      event: Video frame event to log
    """
    camera_name = event.event_name
    if camera_name not in self._video_recorder:
      video_path = os.path.join(self._episode_dir, f"{camera_name}.mp4")
      self._video_recorder[camera_name] = VideoRecorder(video_path)

    frame_id = self._video_recorder[camera_name].add_frame(event.image)
    event_copy = dataclasses.replace(event)
    event_copy.video_frame = frame_id
    event_copy.image = None
    self._logs.append(event_copy)

  def log_vector(self, event: LogEvent):
    """
    Log a vector event.

    Args:
      event: Vector event to log
    """
    self._logs.append(event)

  def save_log(self):
    """Save episode logs and metadata to disk."""
    metadata_path = os.path.join(self._episode_dir, "metadata.json")
    with open(metadata_path, "w") as f:
      json.dump(self._metadata, f, indent=2)

    # Sort events by timestamp before saving
    sorted_logs = sorted(self._logs, key=lambda x: x.timestamp_ns)
    
    logs_path = os.path.join(self._episode_dir, "events.jsonl")
    with open(logs_path, "w") as f:
      for event in sorted_logs:
        f.write(json.dumps(dataclasses.asdict(event)) + "\n")

    for recorder in self._video_recorder.values():
      recorder.close()


class Logger:
  """
  Main logger class for managing episode logging.
  """
  def __init__(self, base_dir: str):
    """
    Initialize logger.

    Args:
      base_dir: Base directory for saving episodes
    """
    self._current_context = None
    self._context_lock = threading.Lock()
    self._base_dir = base_dir

  def create_episode(self):
    """
    Create a new episode context.

    Returns:
      EpisodeContext: New episode context
    """
    ymd_hms = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    episode_dir = os.path.join(self._base_dir, ymd_hms)
    os.makedirs(episode_dir)
    return EpisodeContext(self, episode_dir)
  
  def log_event(self, event: LogEvent):
    """
    Log an event to current episode.

    Args:
      event: Event to log
    """
    if self._current_context is not None:
      self._current_context.log_event(event)

  def attach(self, context: EpisodeContext):
    """
    Attach an episode context.

    Args:
      context: Episode context to attach

    Raises:
      RuntimeError: If there is already an attached context
    """
    with self._context_lock:
      if self._current_context is not None:
        raise RuntimeError("Cannot attach if there is already one")
      self._current_context = context

  def detach(self, context: EpisodeContext):
    """
    Detach an episode context.

    Args:
      context: Episode context to detach

    Raises:
      RuntimeError: If trying to detach wrong context
    """
    with self._context_lock:
      if self._current_context is not context:
        raise RuntimeError("Cannot detach")
      self._current_context = None