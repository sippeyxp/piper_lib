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

  def __init__(self, can_name:str = "can0") -> None:
    self._can_channel_name = can_name
    self._can_bus = None
    self._expected_bitrate = 1000000

    self._check_can_info()

    self._can_bus = can.interface.Bus(channel=self._can_channel_name, bustype="socketcan")

    self._read_can_thread = threading.Thread(target=self._read_can_loop)
    self._read_can_thread.daemon = True

    self._sensed = np.zeros(7)
    self._commanded = np.zeros(7)
    self._gripper_effort = 0
    self._gripper_status = 0

    self._ctrl_mode = 0
    self._arm_status = 0
    self._mode_feed = 0
    self._teach_status = 0
    self._motion_status = 0
    self._trajectory_num = 0
    self._error_code = 0


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
        can_bus.shutdown()  # 关闭 CAN 总线
        # print("CAN bus connection properly shut down.")
    except AttributeError:
        print("CAN bus connection was not properly initialized.")
    except Exception as e:
        print(f"Error occurred while shutting down CAN bus: {e}")
  
  def _check_can_info(self):
    """Check info befo"""
    # 检查 CAN 端口是否存在
    if not can_utils.is_can_socket_available(self._can_channel_name):
        raise ValueError(f"CAN socket {self._can_channel_name} does not exist.")
    print(self._can_channel_name, " is exist")
    # 检查 CAN 端口是否 UP
    if not can_utils.is_can_port_up(self._can_channel_name):
        raise RuntimeError(f"CAN port {self._can_channel_name} is not UP.")
    print(self._can_channel_name, " is UP")
    # 检查 CAN 端口的比特率
    actual_bitrate = can_utils.get_can_bitrate(self._can_channel_name)
    if self._expected_bitrate is not None and not (actual_bitrate == self._expected_bitrate):
        raise ValueError(f"CAN port {self._can_channel_name} bitrate is {actual_bitrate} bps, expected {self._expected_bitrate} bps.")
    print(self._can_channel_name, " bitrate is ", self._expected_bitrate)

  def _is_can_bus_ok(self) -> bool:
    """
    检查CAN总线状态是否正常。
    """
    if self._can_bus is None:
      return False
    bus_state = self._can_bus.state
    if bus_state == can.BusState.ACTIVE:
        # print("CAN bus state: ACTIVE - Bus is functioning normally")
        return True
    elif bus_state == can.BusState.PASSIVE:
        print("CAN bus state: PASSIVE - Warning level errors are occurring")
        return False  # 可以根据需要调整
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

  def start(self):
    if not self._read_can_thread.is_alive():
      self._read_can_thread.start()

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
      case CanIDPiper.ARM_JOINT_FEEDBACK_34.value:
        self._sensed[2] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
        self._sensed[3] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
      case CanIDPiper.ARM_JOINT_FEEDBACK_56.value:
        self._sensed[4] = _joint_can_to_rad(struct.unpack(">l", can_data[:4])[0])
        self._sensed[5] = _joint_can_to_rad(struct.unpack(">l", can_data[4:8])[0])
      case CanIDPiper.ARM_GRIPPER_FEEDBACK.value:
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

  def sensed(self):
    return np.array(self._sensed)


  def enable_motion(self):
    # CAN mode, joint control, speed 100
    self._send_message(arbitration_id=0x151, data=bytearray([0x1, 0x1, 0x64, 0x0, 0x0, 0x0, 0x0, 0x0]))

    # Enable arm(7)
    self._send_message(arbitration_id=0x471, data=bytearray([0x7, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]))

    # Gripper ctrl, 0 open, 1000=1Nm, enable
    self._send_message(arbitration_id=0x159, data=bytearray([0x0, 0x0, 0x0, 0x0, 0x3, 0xe8, 0x1, 0x0]))

  def disable_motion(self):
    # Diable arm (7)
    self._send_message(arbitration_id=0x471, data=bytearray([0x7, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]))

    # Gripper ctrl: disable
    self._send_message(arbitration_id=0x159, data=bytearray([0x0, 0x0, 0x0, 0x0, 0x3, 0xe8, 0x0, 0x0]))

  def command_joints(self, joints):
    assert len(joints) == 7
    self._commanded = np.array(joints)
    # CAN mode, joint control, speed 100
    self._send_message(arbitration_id=0x151, data=bytearray([0x1, 0x1, 0x64, 0x0, 0x0, 0x0, 0x0, 0x0]))

    # joint 1/2
    arm_joints_can = [_joint_rad_to_can(i) for i in self._commanded[:6]]
    self._send_message(arbitration_id=0x155, data=bytearray(struct.pack(">ll", arm_joints_can[0], arm_joints_can[1])))
    self._send_message(arbitration_id=0x156, data=bytearray(struct.pack(">ll", arm_joints_can[2], arm_joints_can[3])))
    self._send_message(arbitration_id=0x157, data=bytearray(struct.pack(">ll", arm_joints_can[4], arm_joints_can[5])))

    # Gripper ctrl, 0 open, 1000=1Nm, enable
    gripper_can = _gripper_m_to_can(self._commanded[6])
    self._send_message(arbitration_id=0x159, data=bytearray(struct.pack(">l", gripper_can)) + bytearray([0x3, 0xe8, 0x1, 0x0]))

    
if __name__  == "__main__":
  piper = Piper()
  piper.start()

  for i in range(100):
    print(piper.sensed())
    #time.sleep(0.1)

  piper.enable_motion()

  A = [0.3, 0.3, 0.4, 0.4, 0.4, -0.4, 0.04]
  phi = [0, 3* np.pi/2, np.pi/2, 0, 0, 0, 0]
  b = [0, 0.3, -0.4, 0.0, 0.0, 0.0, -0.04]
  w = 2 * np.pi / 2
  
  t0 = time.time()

  try:
    while True:
      t = time.time() - t0
      joints = A * np.sin(w * t + np.array(phi)) + b
      #joint_control(piper, joints)
      piper.command_joints(joints)
      #print(joints)
      time.sleep(0.01)
  except KeyboardInterrupt:
    rest_joints = np.zeros(7)
    rest_joints[4] = 0.3
    piper.command_joints(rest_joints)
    time.sleep(3)
  #for i in range(100):
  #  cmd = np.zeros(7)
  #  cmd[0] = i/100* 0.1
  #  print(cmd)
  #  piper.command_joints(cmd)
  #  time.sleep(0.1)  
  piper.disable_motion()

  piper.close()
  
