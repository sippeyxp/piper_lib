import numpy as np
import matplotlib.pyplot as plt
import sys

f = "log.txt"
if len(sys.argv) == 2:
  f = sys.argv[1]

# Sample input data as provided
with open(f, "r") as f:
  data = f.read()


# Convert data to a NumPy array
data = np.array([list(map(float, line.split())) for line in data.strip().split("\n")])

# Extract columns: time, sensed joints (columns 1 to 7), commanded joints (columns 8 to 14)
time = data[:, 0]
sensed_joints = data[:, 1:8]
commanded_joints = data[:, 8:15]

# Plot each pair of sensed and commanded joints
plt.figure()
for i in range(sensed_joints.shape[1]):
    plt.subplot(3, 3, i+1)
    plt.plot(time, sensed_joints[:, i], label=f'Sensed Joint {i + 1}')
    plt.plot(time, commanded_joints[:, i], label=f'Commanded Joint {i + 1}', linestyle='--')
    plt.xlabel('Time')
    plt.ylabel('Joint Values')
    plt.title(f'Comparison of Sensed vs Commanded Joint {i + 1}')
    #plt.legend()
    plt.grid(True)
plt.show()

