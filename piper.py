import time
import can
from can.message import Message
from typing import (
  Optional,
)
from typing import Type
import threading
import can_utils
import numpy as np
from piper_can_ids import CanIDPiper
import struct

def _joint_can_to_rad(joint_can: int) -> float:
  factor = 57324.840764 #1000*180/3.14
  joint_rad = joint_can / factor
  return joint_rad

def _joint_rad_to_can(joint_rad) -> int:
  factor = 57324.840764 #1000*180/3.14
  joint_can = joint_rad * factor
  return int(round(joint_can))

def _gripper_can_to_m(gripper_can: int) -> float:
  factor = 1000*1000
  gripper_m = gripper_can / factor
  return gripper_m

def _gripper_m_to_can(gripper_m) -> int:
  factor = 1000*1000
  gripper_can = gripper_m * factor
  return int(round(gripper_can))


class Piper:

  DEFAULT_CONFIG = {
    "can_name": "can0",
    "bitrate": 1000000,
    "is_leader": False,
  }
    
  def __init__(self, config: dict = None) -> None:
    """Initialize Piper robot interface.
    
    Args:
        config: Configuration dictionary with the following optional keys:
            - can_name: Name of the CAN interface (default: "can0")
            - bitrate: Expected CAN bitrate in bps (default: 1000000)
            - is_leader: Whether the arm is in set up as leader.
    """
    # Use default config if none provided, otherwise update defaults with provided config
    self._config = self.DEFAULT_CONFIG | config

    self._can_channel_name = self._config["can_name"]
    self._can_bus = None
    self._expected_bitrate = self._config.get("bitrate", 1000000)
    self._is_leader = self._config.get("is_leader", False)

    self._check_can_info()

    self._can_bus = can.interface.Bus(channel=self._can_channel_name, interface="socketcan")

    self._read_can_thread = threading.Thread(target=self._read_can_loop)
    self._read_can_thread.daemon = True

    self._sensed_valid = 0
    self._sensed = np.zeros(7)
    self._commanded = np.zeros(7)
    self._gripper_effort = 0
    self._gripper_status = 0

    # arm status
    self._ctrl_mode = 0
    self._arm_status = 0
    self._mode_feed = 0
    self._teach_status = 0
    self._motion_status = 0
    self._trajectory_num = 0
    self._error_code = 0

    # MotionCtrl2 read back
    self._ctrl_mode2 = 0
    self._move_mode = 0
    self._rate_ctrl = 0
    self._mit_mode_read = 0
    self._residence_time = 0

    # Mit mode
    self._mit_mode = False
    self._mit_mode_req = False


  def __del__(self):
    if self._can_bus is not None:
      self.close()

  def close(self):
    """Close can."""
    if self._can_bus is None:
      print("CAN bus was not open.")
      return

    can_bus = self._can_bus
    self._can_bus = None

    if self._read_can_thread.is_alive():
      self._read_can_thread.join()
      print("CAN read thread joined")

    try:
        can_bus.shutdown()
        # print("CAN bus connection properly shut down.")
    except AttributeError:
        print("CAN bus connection was not properly initialized.")
    except Exception as e:
        print(f"Error occurred while shutting down CAN bus: {e}")
  
  def _check_can_info(self):
    """Check info befo"""
    # check existence of CAN port
    if not can_utils.is_can_socket_available(self._can_channel_name):
        raise ValueError(f"CAN socket {self._can_channel_name} does not exist.")
    print(self._can_channel_name, " is exist")
    # check CAN interface is up.
    if not can_utils.is_can_port_up(self._can_channel_name):
        raise RuntimeError(f"CAN port {self._can_channel_name} is not UP.")
    print(self._can_channel_name, " is UP")
    # check CAN bitrate is as expected.
    actual_bitrate = can_utils.get_can_bitrate(self._can_channel_name)
    if self._expected_bitrate is not None and not (actual_bitrate == self._expected_bitrate):
        raise ValueError(f"CAN port {self._can_channel_name} bitrate is {actual_bitrate} bps, expected {self._expected_bitrate} bps.")
    print(self._can_channel_name, " bitrate is ", self._expected_bitrate)

  def _is_can_bus_ok(self) -> bool:
    """Check CAN bus in operable state."""
    if self._can_bus is None:
      return False
    bus_state = self._can_bus.state
    if bus_state == can.BusState.ACTIVE:
        # print("CAN bus state: ACTIVE - Bus is functioning normally")
        return True
    elif bus_state == can.BusState.PASSIVE:
        print("CAN bus state: PASSIVE - Warning level errors are occurring")
        return False
    elif bus_state == can.BusState.ERROR:
        print("CAN bus state: ERROR - Communication may be impaired")
        return False
    else:
        print(f"Unknown CAN bus state: {bus_state}")
        return False

  def _read_can_loop(self):
    while self._can_bus:
      if not self._is_can_bus_ok():
        print("CAN bus is not OK, skipping message read")
        continue

      rx_message = self._can_bus.recv()
      self._parse_message(rx_message)
    print("CAN Read loop exited.")

  def start(self, wait=True):
    if not self._read_can_thread.is_alive():
      self._read_can_thread.start()

    if wait:
      n = 0
      while self._sensed_valid != 7:
        if n > 10:
          raise RuntimeError("Timed out")
        time.sleep(0.1)

  def _parse_message(self, rx_message: Optional[can.Message]):
    can_id:int = rx_message.arbitration_id
    can_data:bytearray = rx_message.data

    match can_id:
      # arm and gripper joint angle feedback
      case CanIDPiper.ARM_STATUS_FEEDBACK.value:
          self._ctrl_mode = can_data[0]
          self._arm_status = can_data[1]
          self._mode_feed = can_data[2]
          self._teach_status = can_data[3]
          self._motion_status = can_data[4]
          self._trajectory_num = can_data[5]
          self._error_code = struct.unpack(">h", can_data[6:8])
      case CanIDPiper.ARM_JOINT_FEEDBACK_12.value:
        self._sensed[0] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
        self._sensed[1] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
        self._sensed_valid |= 1
      case CanIDPiper.ARM_JOINT_FEEDBACK_34.value:
        self._sensed[2] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
        self._sensed[3] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
        self._sensed_valid |= 2
      case CanIDPiper.ARM_JOINT_FEEDBACK_56.value:
        self._sensed[4] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
        self._sensed[5] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
        self._sensed_valid |= 4
      case CanIDPiper.ARM_GRIPPER_FEEDBACK.value:
        self._sensed[6] = _gripper_can_to_m(struct.unpack(">l", can_data[:4])[0])
        self._gripper_effort = struct.unpack(">h", can_data[4:6])[0]
        self._gripper_status = struct.unpack("B", can_data[6:7])[0]
      case CanIDPiper.ARM_MOTION_CTRL_2.value:
        self._ctrl_mode2 = struct.unpack("B", can_data[0:1])[0]
        self._move_mode = struct.unpack("B", can_data[1:2])[0]
        self._rate_ctrl = struct.unpack("B", can_data[2:3])[0]
        self._mit_mode_read = struct.unpack("B", can_data[3:4])[0]
        self._residence_time = struct.unpack("B", can_data[0:1])[0]

    if self._is_leader: # arm in leader setup send feedback as command.
      match can_id:
        case CanIDPiper.ARM_JOINT_CTRL_12.value:
          self._sensed[0] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
          self._sensed[1] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
          self._sensed_valid |= 1
        case CanIDPiper.ARM_JOINT_CTRL_34.value:
          self._sensed[2] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
          self._sensed[3] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
          self._sensed_valid |= 2
        case CanIDPiper.ARM_JOINT_CTRL_56.value:
          self._sensed[4] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
          self._sensed[5] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
          self._sensed_valid |= 4
        case CanIDPiper.ARM_GRIPPER_CTRL.value:
          self._sensed[6] = _gripper_can_to_m(struct.unpack(">l", can_data[:4])[0])
          self._gripper_effort = struct.unpack(">h", can_data[4:6])[0]
          self._gripper_status = struct.unpack("B", can_data[6:7])[0]


  def _send_message(self, arbitration_id, data):
    message = can.Message(channel=self._can_channel_name,
        arbitration_id=arbitration_id, 
        data=data, 
        dlc=8,
        is_extended_id=False)

    if not self._is_can_bus_ok():
      print("can bus in bad state, cannot send")

    try:
      self._can_bus.send(message)
    except can.CanError:
      print("send message failed")

  def sensed_joints(self):
    return np.array(self._sensed)

  def get_arm_status(self):
    self._ctrl_mode = 0
    self._arm_status = 0
    self._mode_feed = 0
    self._teach_status = 0
    self._motion_status = 0
    self._trajectory_num = 0
    self._error_code = 0
    return {"ctrl mode": self._ctrl_mode,
            "arm_status": self._arm_status,
            "mode_feed": self._mode_feed,
            "teach_status": self._teach_status,
            "motion_status": self._motion_status,
            "trajectory_num": self._trajectory_num,
            "error_code": self._error_code,
            }


  def get_motion_ctrl2(self):
    return {"ctrl_mode": self._ctrl_mode2,
            "move_mode": self._move_mode,
            "rate_ctrl": self._rate_ctrl,
            "mit_mode": self._mit_mode_read,
            "residence_time": self._residence_time,
            }

  def enter_mit_mode(self, mit=True):
    # CAN mode, joint control, speed 100
    self._mit_mode = mit
    self._send_message(
            arbitration_id=CanIDPiper.ARM_MOTION_CTRL_2.value,
            data=bytearray([0x1, 0x1, 0x64, 0xAD if mit else 0x0, 0x0, 0x0, 0x0, 0x0]),
    )

  def enable_motion(self):
    # CAN mode, joint control, speed 100
    print("mit mode = ", self._mit_mode)

    self._send_message(
            arbitration_id=CanIDPiper.ARM_MOTION_CTRL_2.value,
            data=bytearray([0x1, 0x1, 0x64, 0xAD if self._mit_mode else 0x0, 0x0, 0x0, 0x0, 0x0]),
    )
    # Without these sleep, sometime initial joint command is not accepted.
    # Some of these sleeps may not be necessary.
    time.sleep(.1)

    # Enable arm(7)
    self._send_message(arbitration_id=CanIDPiper.ARM_MOTOR_ENABLE_DISABLE_CONFIG.value, data=bytearray([0x7, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]))
    time.sleep(.1)

    # Gripper ctrl, 0 open, 1000=1Nm, enable
    self._send_message(arbitration_id=CanIDPiper.ARM_GRIPPER_CTRL.value, data=bytearray([0x0, 0x0, 0x0, 0x0, 0x3, 0xe8, 0x1, 0x0]))
    time.sleep(.1)

    # set motion ctrl 2 again
    self._send_message(
            arbitration_id=CanIDPiper.ARM_MOTION_CTRL_2.value,
            data=bytearray([0x1, 0x1, 0x64, 0xAD if self._mit_mode else 0x0, 0x0, 0x0, 0x0, 0x0]),
    )
    time.sleep(.1)

  def disable_motion(self):
    # Diable arm (7)
    self._send_message(arbitration_id=CanIDPiper.ARM_MOTOR_ENABLE_DISABLE_CONFIG.value, data=bytearray([0x7, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]))

    # Gripper ctrl: disable
    self._send_message(arbitration_id=CanIDPiper.ARM_GRIPPER_CTRL.value, data=bytearray([0x0, 0x0, 0x0, 0x0, 0x3, 0xe8, 0x0, 0x0]))

  def command_joints(self, joints):
    if self._is_leader:
      print("leader arm does not accept joint command")
      return
    assert len(joints) == 7
    self._commanded = np.array(joints)

    arm_joints_can = [_joint_rad_to_can(i) for i in self._commanded[:6]]
    self._send_message(arbitration_id=CanIDPiper.ARM_JOINT_CTRL_12.value, data=bytearray(struct.pack(">ll", arm_joints_can[0], arm_joints_can[1])))
    self._send_message(arbitration_id=CanIDPiper.ARM_JOINT_CTRL_34.value, data=bytearray(struct.pack(">ll", arm_joints_can[2], arm_joints_can[3])))
    self._send_message(arbitration_id=CanIDPiper.ARM_JOINT_CTRL_56.value, data=bytearray(struct.pack(">ll", arm_joints_can[4], arm_joints_can[5])))

    # Gripper ctrl, 0 open, 1000=1Nm, enable
    gripper_can = _gripper_m_to_can(self._commanded[6])
    self._send_message(arbitration_id=CanIDPiper.ARM_GRIPPER_CTRL.value, data=bytearray(struct.pack(">l", gripper_can)) + bytearray([0x3, 0xe8, 0x1, 0x0]))

class Metronome:
  """Simple timing class."""

  def __init__(self, freq):
    self._tick = 1/freq
    self.reset()

  def reset(self):
    self._t0 = time.time()
    self._next_t = self._tick
    self._tick_i = 0

  @property
  def t(self):
    return time.time() - self._t0

  @property
  def i(self):
    return self._tick_i

  def wait(self):
    while self.t < self._next_t:
      time.sleep(self._tick / 10)
    self._next_t += self._tick
    self._tick_i += 1
    return self.t

