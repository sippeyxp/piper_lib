import matplotlib
print(matplotlib.get_backend())
matplotlib.use('agg')

from absl import app
from absl import flags
import os
import sys
import time
import numpy as np
import cv2

from safari_sdk.model import gemini_robotics_policy
from safari_sdk.model import genai_robotics
import matplotlib.pyplot as plt


import tensorflow as tf

_IMAGE_SIZE = (480, 848)
_ALOHA_CAMERAS = {
    'overhead_cam': _IMAGE_SIZE,
    'worms_eye_cam': _IMAGE_SIZE,
    'wrist_cam_left': _IMAGE_SIZE,
    'wrist_cam_right': _IMAGE_SIZE,
}
_ALOHA_JOINTS = {'joints_pos': 14}


_SERVE_ID = 'gemini_robotics_on_device'


from array_record.python.array_record_module import ArrayRecordReader

FLAGS = flags.FLAGS
flags.DEFINE_string('arrayrecord_file', None, 'Path to the arrayrecord file to load.')
flags.DEFINE_integer('record_number', 0, 'Index of the record to load from the arrayrecord file.')
flags.mark_flag_as_required('arrayrecord_file')

def load_data(file_path, record_number=0):
  """Load data from a file.

  Args:
    file_path: The path to the array_record file.
    record_number: The index of the record to load.

  Returns:
    A tuple containing the observation and the action chunk.
  """
  reader = ArrayRecordReader(file_path)
  results = reader.read_all()

  print(f"Number of records in {file_path}: {len(results)}")

  if not results:
    raise ValueError(f"No data found in {file_path}")

  if record_number >= len(results):
    raise IndexError(f"Record number {record_number} out of range (max {len(results)-1})")

  data = results[record_number]
  
  example = tf.train.Example()
  example.ParseFromString(data)
  print("parsed")

  observation = {
      'overhead_cam': example.features.feature['observation/cam_0'].bytes_list.value[0],
      'worms_eye_cam': example.features.feature['observation/cam_1'].bytes_list.value[0],
      'wrist_cam_left': example.features.feature['observation/cam_2'].bytes_list.value[0],
      'wrist_cam_right': example.features.feature['observation/cam_3'].bytes_list.value[0],
      'joints_pos': np.array(example.features.feature['observation/joints_pos'].float_list.value),
  }
  total_action_dim = sum(_ALOHA_JOINTS.values())

  action_chunk = np.array(example.features.feature['action'].float_list.value).reshape([-1, total_action_dim])
  return observation, action_chunk
  


def resize_and_decode_image(image_bytes, target_size=(480, 848)):
  """Decodes a JPEG image from bytes and resizes it."""
  img_np = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
  if img_np is None:
    raise ValueError("Failed to decode image")
  resized_img = cv2.resize(img_np, (target_size[1], target_size[0]))  # cv2.resize expects (width, height)
  return resized_img

def main(args):
  instruction = "pick up the toy fish and place it in the bowl"

  policy = gemini_robotics_policy.GeminiRoboticsPolicy(
    serve_id=_SERVE_ID,
    task_instruction=instruction,
    inference_mode=gemini_robotics_policy.InferenceMode.SYNCHRONOUS,
    cameras=_ALOHA_CAMERAS,
    joints=_ALOHA_JOINTS,
    robotics_api_connection=genai_robotics.RoboticsApiConnectionType.LOCAL,
  )
  policy.setup()

  observation, action_chunk = load_data(FLAGS.arrayrecord_file, FLAGS.record_number)

  for camera_name in _ALOHA_CAMERAS:
    try:
      observation[camera_name] = resize_and_decode_image(observation[camera_name])
    except Exception as e:
      print(f"Error processing camera {camera_name}: {e}")
      raise

  action = policy.step(observation)
  rest_of_action = policy._model_output
  predicted_action = np.concatenate((action[None, :], rest_of_action))

  # action_chunk is ground truth, dim (50, 14)

  def plot_result(ground_truth, predicted, overhead_img, wrist_right_img):
    num_joints = ground_truth.shape[1]
    time_steps = ground_truth.shape[0]
    joints_per_col = num_joints // 2

    fig, axes = plt.subplots(joints_per_col + 1, 2, figsize=(14, 2.5 * (joints_per_col + 1)))

    # Top row: images
    ax_img_left = axes[0, 0]
    ax_img_left.imshow(cv2.cvtColor(overhead_img, cv2.COLOR_BGR2RGB))
    ax_img_left.axis('off')
    ax_img_left.set_title('Overhead Camera Image')

    ax_img_right = axes[0, 1]
    ax_img_right.imshow(cv2.cvtColor(wrist_right_img, cv2.COLOR_BGR2RGB))
    ax_img_right.axis('off')
    ax_img_right.set_title('Right Wrist Camera Image')

    # Joint plots
    for i in range(joints_per_col):
      ax_left = axes[i + 1, 0]
      ax_left.plot(ground_truth[:, i], label='Ground Truth')
      ax_left.plot(predicted[:, i], label='Predicted')
      ax_left.set_ylabel(f'Joint {i+1}')
      ax_left.legend()
      ax_left.set_xlim(0, time_steps)
      if i == joints_per_col - 1:
        ax_left.set_xlabel('Time Step')

      ax_right = axes[i + 1, 1]
      ax_right.plot(ground_truth[:, i + joints_per_col], label='Ground Truth')
      ax_right.plot(predicted[:, i + joints_per_col], label='Predicted')
      ax_right.set_ylabel(f'Joint {i+1+joints_per_col}')
      ax_right.legend()
      ax_right.set_xlim(0, time_steps)
      if i == joints_per_col - 1:
        ax_right.set_xlabel('Time Step')

    plt.tight_layout()
    plt.savefig('output.png')
    print("Plot saved to output.png")

  plot_result(action_chunk, predicted_action, observation['overhead_cam'], observation['wrist_cam_right'])


if __name__ == "__main__":
  app.run(main)
  # observation, chunk = load_data("/home/pengxu/Downloads/train.arrayrecord-00048-of-00061")