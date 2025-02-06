"""visualize log"""
from absl import flags
from absl import app
import json
import os
import matplotlib
matplotlib.use('TkAgg')  # Set the backend before importing pyplot
import matplotlib.pyplot as plt
import numpy as np


flags.DEFINE_string("episode_log_dir", None, "Directory to load logs.", required=True)
FLAGS = flags.FLAGS

def load_jsonl(file_path: str):
  """Load a JSONL file.
  
  Args:
    file_path: Path to the JSONL file
    
  Returns:
    List of dictionaries, each representing a JSON object from a line
  """
  events = []
  with open(file_path, "r") as f:
    for line in f:
      events.append(json.loads(line.strip()))
  return events


def parse_joints(data):
  commanded = []
  observed = []
  commanded_timestamps = []
  observed_timestamps = []

  for d in data:
    if d['event_type'] != "joint_pos":
      continue
    if d['event_name'] == 'commanded':
      commanded.append(d['joint_pos'])
      commanded_timestamps.append(d['timestamp_ns'])
    elif d['event_name'] == 'observed':
      observed.append(d['joint_pos'])
      observed_timestamps.append(d['timestamp_ns'])
    else:
      raise ValueError(f"Unknown event_name: {d['event_name']}")

  commanded = np.array(commanded)
  observed = np.array(observed)

  t0 = observed_timestamps[0]
  observed_timestamps = np.array(observed_timestamps)
  observed_timestamps = (observed_timestamps - t0) / 1e9

  commanded_timestamps = np.array(commanded_timestamps)
  commanded_timestamps = (commanded_timestamps - t0) / 1e9
  
  return (commanded_timestamps, commanded), (observed_timestamps, observed)

def main(_):
  events_file = os.path.join(FLAGS.episode_log_dir, "events.jsonl")
  print("Loading events from ", events_file)

  events_data = load_jsonl(events_file)
  (commanded_timestamps, commanded), (observed_timestamps, observed) = parse_joints(events_data)

  N = 7

  plt.figure()
  for i in range(N):
    plt.subplot(2,4,i+1)
    plt.plot(commanded_timestamps, commanded[:,i], label='commanded')
    plt.plot(observed_timestamps, observed[:,i], label='observed')
    plt.legend()
  plt.show()

  
if __name__ == "__main__":
  app.run(main)
