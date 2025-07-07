"""Visualize joints, commands, and video streams using rerun.io"""

from absl import flags
from absl import app
import os
import json
import cv2
import numpy as np
import rerun as rr
import argparse

flags.DEFINE_string("episode_log_dir", None, "Directory to load logs.", required=True)
FLAGS = flags.FLAGS

def load_jsonl(file_path: str):
    events = []
    with open(file_path, "r") as f:
        for line in f:
            events.append(json.loads(line.strip()))
    return events

def main(_):
    parser = argparse.ArgumentParser(description="Logs rich data using the Rerun SDK.")
    rr.script_add_args(parser)
    # ignore the default flags
    parser.add_argument("--episode_log_dir", type=str, required=True)
    args = parser.parse_args()
    rr.script_setup(args, "rerun_example_dna_abacus")


    episode_dir = FLAGS.episode_log_dir
    events_file = os.path.join(episode_dir, "events.jsonl")
    metadata_file = os.path.join(episode_dir, "metadata.json")

    print(f"Loading events from {events_file}")
    events = load_jsonl(events_file)
    with open(metadata_file, "r") as f:
        metadata = json.load(f)

    cameras = metadata.get("cameras", ["right_wrist_cam"])

    # Parse joint events
    commanded = []
    observed = []
    commanded_ts = []
    observed_ts = []
    frame_index_per_cam = {c: [] for c in cameras}
    frame_ts_per_cam = {c: [] for c in cameras}
    for d in events:
        if d['event_type'] == "joint_pos":
            if d['event_name'] == 'commanded':
                commanded.append(d['joint_pos'])
                commanded_ts.append(d['timestamp_ns'])
            elif d['event_name'] == 'observed':
                observed.append(d['joint_pos'])
                observed_ts.append(d['timestamp_ns'])
        elif d['event_type'] == "image":
            camera = d['event_name']
            frame_index_per_cam[camera].append(d['video_frame'])
            frame_ts_per_cam[camera].append(d['timestamp_ns'])
    commanded = np.array(commanded)
    observed = np.array(observed)
    commanded_ts = np.array(commanded_ts)
    observed_ts = np.array(observed_ts)
    if len(observed_ts) == 0:
        print("No observed joint data found.")
        return

    rr.log(f"joints_commanded", rr.SeriesLines(colors=[255,0,0], names=[f"commanded_{i}" for i in range(commanded.shape[1])]), static=True)
    rr.send_columns(f"joints_commanded", indexes=[rr.TimeColumn("step", timestamp=commanded_ts)]*commanded.shape[1], columns=rr.Scalars.columns(scalars=commanded))

    rr.log(f"joints_observed", rr.SeriesLines(colors=[0,255,0], names=[f"observed_{i}" for i in range(observed.shape[1])]), static=True)
    rr.send_columns(f"joints_observed", indexes=[rr.TimeColumn("step", timestamp=observed_ts)]*observed.shape[1], columns=rr.Scalars.columns(scalars=observed))

    # Log video frames
    for camera in cameras:
        video_path = os.path.join(episode_dir, f"{camera}.mp4")
        video = cv2.VideoCapture(video_path)
        print(f"Logging video for {camera}")
        frame_idx = 0

        while True:
            ret, frame = video.read()
            if not ret:
                break
            rr.set_time("step", timestamp=np.datetime64(frame_ts_per_cam[camera][frame_idx], 'ns'))
            rr.log(f"video/{camera}", rr.Image(frame, color_model="BGR"))
            frame_idx += 1

        video.release()

    print("Visualization complete. Run 'rerun' viewer to see the results.")
    input("Waiting for all video to load. Press Enter to continue...")
    rr.script_teardown(args)

if __name__ == "__main__":
    app.run(main) 