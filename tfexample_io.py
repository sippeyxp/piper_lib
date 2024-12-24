
import tensorflow as tf
import numpy as np
import cv2


def _bytes_feature(value):
  """Returns a bytes_list from a string / byte."""
  if isinstance(value, type(tf.constant(0))):
    value = value.numpy() # BytesList won't unpack a string from an EagerTensor.
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))

def _float_feature(value):
  """Returns a float_list from a float / double."""
  return tf.train.Feature(float_list=tf.train.FloatList(value=value))

def _int64_feature(value):
  """Returns an int64_list from a bool / enum / int / uint."""
  return tf.train.Feature(int64_list=tf.train.Int64List(value=value))

def serialize_example(observation, action, valid, task):
    """
    Creates a tf.train.Example message ready to be written to a file.
    """
    # Create a dictionary mapping the feature name to the tf.train.Example-compatible
    # data type.
    def cv2_image_encode(image):
      ret, encoded =  cv2.imencode(".jpg", image)
      return encoded.tobytes()

    feature = {
        'observation/right_wrist_cam': _bytes_feature(cv2_image_encode(observation["right_wrist_cam"])),
        'observation/joints_pos': _float_feature(observation["joints_pos"].tolist()),
        'action': _float_feature(action.flatten().tolist()),
        'valid': _int64_feature(valid.tolist()),
        'task': _bytes_feature(task.encode()),
    }

    # Create a Features message using tf.train.Example.

    example_proto = tf.train.Example(features=tf.train.Features(feature=feature))
    return example_proto.SerializeToString()



def write_data_generator_to_file(data, filename):
    with tf.io.TFRecordWriter(filename) as writer:
        for item in data:
            example = serialize_example(item["observation"], item["action"], item["valid"])
            writer.write(example)


def get_parse_function(camera_names=("right_wrist_cam",), n_joints=7, chunk_size=50):
  def _parse_function(example_proto):
      camera_feature_description = {
              f'observation/{name}': tf.io.FixedLenFeature([], tf.string) for name in camera_names
      }
      feature_description = camera_feature_description | {
          'observation/joints_pos': tf.io.FixedLenFeature([n_joints,], tf.float32), # shape (14,)
          'action': tf.io.FixedLenFeature([chunk_size, n_joints], tf.float32),  # shape (100, 14)
          'valid': tf.io.FixedLenFeature([chunk_size], tf.int64),  # shape (100,)
      }
      print(feature_description)
      data = tf.io.parse_single_example(example_proto, feature_description)
      example = {
          "observation": {
              "right_wrist_cam": tf.io.decode_png(data["observation/right_wrist_cam"]),
              "joints_pos": data["observation/joints_pos"],
            } | {
              name: tf.io.decode_png(data[f"observation/{name}"]) for name in camera_names
            },
          "action": data["action"],
          "valid": data["valid"],
      }
      return example
  return _parse_function

