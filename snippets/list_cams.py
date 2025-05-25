import cv2
import subprocess
import time
import numpy as np

def list_cameras():
  import os
  cameras = []
  video_devices = [dev for dev in os.listdir('/dev') if dev.startswith('video')]
  video_indices = sorted([int(dev.replace('video', '')) for dev in video_devices if dev.replace('video', '').isdigit()])
  for index in video_indices:
    cap = cv2.VideoCapture(index)
    if cap.isOpened():
      cameras.append(index)
    cap.release()
  return cameras

def get_usb_device_id_for_video(index):
  dev_path = f"/dev/video{index}"
  try:
    cmd = ["udevadm", "info", "--query=all", "--name", dev_path]
    output = subprocess.check_output(cmd, encoding="utf-8")
    usb_info = {}
    for line in output.splitlines():
      if "ID_SERIAL=" in line:
        usb_info["ID_SERIAL"] = line.split("=", 1)[1]
      if "ID_MODEL_ID=" in line:
        usb_info["ID_MODEL_ID"] = line.split("=", 1)[1]
      if "ID_VENDOR_ID=" in line:
        usb_info["ID_VENDOR_ID"] = line.split("=", 1)[1]
    if usb_info:
      return usb_info
    else:
      return None
  except Exception as e:
    return None

camera_indices = list_cameras()
print("USB device info for detected cameras:")
for idx in camera_indices:
  info = get_usb_device_id_for_video(idx)
  if info:
    print(f"/dev/video{idx}: {info}")
  else:
    print(f"/dev/video{idx}: Not a USB camera or no USB info found")

caps = []
for idx in camera_indices:
  cap = cv2.VideoCapture(idx)
  if cap.isOpened():
    caps.append((idx, cap))
  else:
    print(f"Failed to open /dev/video{idx}")

if not caps:
  print("No cameras to display.")
  exit()

print("Press 'q' in any window to quit.")
while True:
  for idx, cap in caps:
    ret, frame = cap.read()
    if ret:
      cv2.imshow(f"Camera {idx} (/dev/video{idx})", frame)
    else:
      cv2.imshow(
        f"Camera {idx} (/dev/video{idx})",
        cv2.putText(
          np.zeros((240,320,3), dtype='uint8'),
          "No Frame", (30,120), cv2.FONT_HERSHEY_SIMPLEX,
          1, (0,0,255), 2, cv2.LINE_AA
        )
      )
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break
for _, cap in caps:
  cap.release()
cv2.destroyAllWindows()
