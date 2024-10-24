
import os
import subprocess

def is_can_socket_available(channel_name: str) -> bool:
    """
    检查给定的 CAN 端口是否存在。
    """
    try:
        with open(f"/sys/class/net/{channel_name}/operstate", "r") as file:
            state = file.read().strip()
            return state == "up"
    except FileNotFoundError:
        return False

def get_can_ports(self) -> list:
    """
    获取系统中所有可用的 CAN 端口。
    """
    can_ports = []
    for item in os.listdir('/sys/class/net/'):
        if item.startswith("can"):
            can_ports.append(item)
    return can_ports

def can_port_info(channel_name: str) -> str:
    """
    获取指定 CAN 端口的详细信息，包括状态、类型和比特率。
    """
    try:
        with open(f"/sys/class/net/{channel_name}/operstate", "r") as file:
            state = file.read().strip()
        with open(f"/sys/class/net/{channel_name}/type", "r") as file:
            port_type = file.read().strip()
        bitrate = get_can_bitrate(channel_name)
        return f"CAN port {channel_name}: State={state}, Type={port_type}, Bitrate={bitrate} bps"
    except FileNotFoundError:
        return f"CAN port {channel_name} not found."

def is_can_port_up(channel_name: str) -> bool:
    """
    检查 CAN 端口是否为 UP 状态。
    """
    try:
        with open(f"/sys/class/net/{channel_name}/operstate", "r") as file:
            state = file.read().strip()
            return state == "up"
    except FileNotFoundError:
        return False

def get_can_bitrate(channel_name: str) -> str:
    """
    获取指定 CAN 端口的比特率。
    """
    try:
        result = subprocess.run(['ip', '-details', 'link', 'show', channel_name],
                                capture_output=True, text=True)
        output = result.stdout
        for line in output.split('\n'):
            print(line)
            if 'bitrate' in line:
                return int(line.split('bitrate ')[1].split(' ')[0])
        return "Unknown"
    except Exception as e:
        print(f"Error while getting bitrate: {e}")
        return "Unknown"
