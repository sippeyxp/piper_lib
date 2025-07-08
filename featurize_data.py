"""Feature extraction for teleoperation data."""

import os
import pathlib

from absl import app
from absl import flags
import numpy as np
import json
import cv2
import re
from typing import List, Dict, Any

import tfexample_io
import tensorflow as tf

FLAGS = flags.FLAGS

flags.DEFINE_string("episode_log_dir", None, "Directory containing episode logs.", required=True)

flags.DEFINE_string("output_dir", None, "Directory to save extracted features.", required=True)

flags.DEFINE_string("task_name", None, "Name of task to use", required=True)

flags.DEFINE_boolean("success_only", True, "Success episode only.")

flags.DEFINE_integer("action_stride", 1, "Stride of actions")
flags.DEFINE_integer("action_chunk_size", 50, "Chunk size of actions")

flags.DEFINE_string("include_dir_regex", None, "Regex of log dirs to select. If not specified, select all.")


def process_episode(episode_dir: str, num_future_steps: int = 50, stride: int = 1):
  # generate trainable example.

  # load metadata.json into metadata
  with open(os.path.join(episode_dir, "metadata.json"), "r") as f:
    metadata = json.load(f)

  print("  task = ", metadata.get("task", "invalid-task"))
  if metadata.get("task", "invalid-task") != FLAGS.task_name:
    return

  print("  success = ", metadata.get("success", "not set"))
  if FLAGS.success_only and metadata.get("success", None) != True:
    return

  # load events.jsonl file into a list of dict
  events = []
  with open(os.path.join(episode_dir, "events.jsonl"), "r") as f:
    for line in f:
      events.append(json.loads(line))

  # Get list of cameras from metadata
  cameras = metadata.get("cameras", ["right_wrist_cam"])

  # Open video files for each camera
  videos = {}
  for camera in cameras:
    video_path = os.path.join(episode_dir, f"{camera}.mp4")
    videos[camera] = cv2.VideoCapture(video_path)

  # Find all joint command events and image events
  joint_events = [e for e in events if e["event_type"] == "joint_pos" and e["event_name"] == "commanded"]
  joint_observed = [e for e in events if e["event_type"] == "joint_pos" and e["event_name"] == "observed"]
  
  # Group image events by camera
  image_events = {}
  for camera in cameras:
    image_events[camera] = [e for e in events if e["event_type"] == "image" and e["event_name"] == camera]

  def binary_search_ts(timestamp, events):
    left, right = 0, len(events)
    while left < right:
      mid = (left + right) // 2
      if events[mid]["timestamp_ns"] <= timestamp:
        left = mid + 1
      else:
        right = mid
    start_idx = left
    return start_idx if start_idx != len(events) else None

  examples = []
  # Use first camera's events as reference for timing
  reference_camera = cameras[0]
  for i, image_event in enumerate(image_events[reference_camera]):
    # Binary search to find first joint event after image
    image_time = image_event["timestamp_ns"]
    start_idx = binary_search_ts(image_time, joint_events)
    if start_idx is None:
      break
    # Get future joint commands with stride
    future_joints = []
    curr_idx = start_idx

    valid = []
    while len(future_joints) < num_future_steps and curr_idx < len(joint_events):
      future_joints.append(joint_events[curr_idx]["joint_pos"])
      curr_idx += stride
      valid.append(1)

    # Pad with last joint position if needed
    while len(future_joints) < num_future_steps:
      future_joints.append(future_joints[-1] if future_joints else np.zeros(7))
      valid.append(0)

    # Read the corresponding frame from each camera
    frames = {}
    skip_example = False
    for camera, video in videos.items():
      video.set(cv2.CAP_PROP_POS_FRAMES, i)
      ret, frame = video.read()
      if not ret:
        skip_example = True
        break
      frames[camera] = frame
    
    if skip_example:
      continue

    # Create example tuple
    example = {
       "observation": {
         **{camera: frame for camera, frame in frames.items()},
         "joints_pos": np.array(joint_observed[start_idx]["joint_pos"]),
       },
       "action":  np.array(future_joints),
       "task": metadata["task"],
       "valid": np.array(valid, dtype=np.int64),
       "metadata": metadata,
    }
    yield example

  # Release all video captures
  for video in videos.values():
    video.release()


def main(argv):
  del argv
  # loop all directory in episode_log_dir

  n_examples = 0

  def all_examples():
    episode_dirs = sorted(pathlib.Path(FLAGS.episode_log_dir).glob("202*"))
    if FLAGS.include_dir_regex:
      episode_dirs = [d for d in episode_dirs if re.match(FLAGS.include_dir_regex, d.name)]
    for episode_dir in episode_dirs:
      if not episode_dir.is_dir():
        continue
      print("Processing", episode_dir)
      for example in process_episode(str(episode_dir),
                                   num_future_steps=FLAGS.action_chunk_size,
                                   stride=FLAGS.action_stride):
        yield example

  action_list = []
  joints_pos_list= []

  i = 0

  os.makedirs(FLAGS.output_dir, exist_ok=True)
  with tf.io.TFRecordWriter(os.path.join(FLAGS.output_dir, "featurized_data.tfrec")) as writer:
    for item in all_examples():
      example = tfexample_io.serialize_example(
              observation=item["observation"],
              action=item["action"],
              valid=item["valid"],
              task=item["task"],
              camera_names=item["metadata"]["cameras"],
              )
      writer.write(example)

      for valid, action in zip(item["valid"], item["action"]):
        if not valid:
          continue
        action_list.append(action)
      joints_pos_list.append(item["observation"]["joints_pos"])

      i += 1

  action_list = np.array(action_list)
  joints_pos_list = np.array(joints_pos_list)
  stats = {
      "action": {
          "min": np.min(action_list, axis=0).tolist(),
          "max": np.max(action_list, axis=0).tolist(),
          "mean": np.mean(action_list, axis=0).tolist(),
          "std": np.std(action_list, axis=0).tolist()
      },
      "joints_pos": {
          "min": np.min(joints_pos_list, axis=0).tolist(),
          "max": np.max(joints_pos_list, axis=0).tolist(),
          "mean": np.mean(joints_pos_list, axis=0).tolist(),
          "std": np.std(joints_pos_list, axis=0).tolist()
      }
  }
  with open(os.path.join(FLAGS.output_dir, "stats.json"), "w") as f:
    json.dump(stats, f)


if __name__ == "__main__":
  app.run(main)
