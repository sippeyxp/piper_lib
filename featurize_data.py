"""Feature extraction for teleoperation data."""

import os
import pathlib

from absl import app
from absl import flags
import numpy as np
import json
import cv2
from typing import List, Dict, Any

import tfexample_io
import tensorflow as tf

FLAGS = flags.FLAGS

flags.DEFINE_string("episode_log_dir", None, "Directory containing episode logs.", required=True)

flags.DEFINE_string("output_dir", None, "Directory to save extracted features.", required=True)

flags.DEFINE_string("task_name", None, "Name of task to use", required=True)

flags.DEFINE_boolean("success_only", True, "Success episode only.")


def process_episode(episode_dir: str, num_future_steps: int = 50, stride: int = 1):
  # generate trainable example.

  # load metadata.json into metadata
  with open(os.path.join(episode_dir, "metadata.json"), "r") as f:
    metadata = json.load(f)

  if metadata["task"] != "red-fish-into-bowl":
    return

  print("success = ", metadata.get("success", "not set"))
  if metadata["success"] != True:
    return

  # load events.jsonl file into a list of dict
  events = []
  with open(os.path.join(episode_dir, "events.jsonl"), "r") as f:
    for line in f:
      events.append(json.loads(line))

  # open video file right_wrist_cam.mp4
  video = cv2.VideoCapture(os.path.join(episode_dir, "right_wrist_cam.mp4"))

  # Find all joint command events and image events
  joint_events = [e for e in events if e["event_type"] == "joint_pos" and e["event_name"] == "commanded"]
  joint_observed = [e for e in events if e["event_type"] == "joint_pos" and e["event_name"] == "observed"]
  image_events = [e for e in events if e["event_type"] == "image" and e["event_name"] == "right_wrist_cam"]


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
  for i, image_event in enumerate(image_events):
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

    # Read the corresponding frame from video
    video.set(cv2.CAP_PROP_POS_FRAMES, i)
    ret, frame = video.read()
    if not ret:
      continue

    # Create example tuple
    example = {
       "observation": {
         "right_wrist_cam": frame,
         "joints_pos": np.array(joint_observed[start_idx]["joint_pos"]),
       },
       "action":  np.array(future_joints),
       "task": metadata["task"],
       "valid": np.array(valid, dtype=np.int64),
    }
    yield example

  video.release()


def main(argv):
  del argv
  # loop all directory in episode_log_dir

  n_examples = 0

  def all_examples():
    episode_dirs = sorted(pathlib.Path(FLAGS.episode_log_dir).glob("202*"))
    for episode_dir in episode_dirs:
      if not episode_dir.is_dir():
        continue
      print("Processing", episode_dir)
      for example in process_episode(str(episode_dir)):
        yield example

  #print("Number of examples", n_examples)

  # examples = process_episode("/host/tmp/episode_log/20241118_031107")
  # import pdb; pdb.set_trace()
  i=0
  with tf.io.TFRecordWriter(os.path.join(FLAGS.output_dir, "featurized_data.recio")) as writer:
    for item in all_examples():
      example = tfexample_io.serialize_example(
              observation=item["observation"],
              action=item["action"],
              valid=item["valid"],
              task=item["task"],
              )
      writer.write(example)
      i+=1
      #if i>10:
      #    break


if __name__ == "__main__":
  app.run(main)
