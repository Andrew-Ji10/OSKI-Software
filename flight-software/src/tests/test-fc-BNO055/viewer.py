import math
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PORT = "/dev/tty.usbmodem14301"
BAUD = 921600

ser = serial.Serial(PORT, BAUD, timeout=0)
ser.reset_input_buffer()

def quat_to_rotmat(w, x, y, z):
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])

L, W, H = 2.0, 1.0, 0.4
verts = np.array([
    [-L/2, -W/2, -H/2],
    [ L/2, -W/2, -H/2],
    [ L/2,  W/2, -H/2],
    [-L/2,  W/2, -H/2],
    [-L/2, -W/2,  H/2],
    [ L/2, -W/2,  H/2],
    [ L/2,  W/2,  H/2],
    [-L/2,  W/2,  H/2],
])

faces_idx = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
]

latest_q = [1.0, 0.0, 0.0, 0.0]

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
ax.set_box_aspect([1, 1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("BNO055 Live Orientation")

poly = Poly3DCollection([], alpha=0.8)
ax.add_collection3d(poly)

ax.plot([0, 1.5], [0, 0], [0, 0])
ax.plot([0, 0], [0, 1.5], [0, 0])
ax.plot([0, 0], [0, 0], [0, 1.5])

text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)

def read_latest_serial():
    global latest_q
    newest_line = None

    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                newest_line = line

        if newest_line is None:
            return

        parts = newest_line.split(",")
        if len(parts) != 4:
            return

        latest_q = [float(p) for p in parts]

    except Exception:
        pass

def update(frame):
    read_latest_serial()

    # axis mapping you said looked correct
    w, x, y, z = latest_q[0], latest_q[2], latest_q[1], latest_q[3]

    R = quat_to_rotmat(w, x, y, z)
    rotated = verts @ R.T
    faces = [[rotated[i] for i in face] for face in faces_idx]

    poly.set_verts(faces)
    text.set_text(f"q = [{w:.3f}, {x:.3f}, {y:.3f}, {z:.3f}]")
    return poly, text

ani = FuncAnimation(fig, update, interval=15, blit=False, cache_frame_data=False)
plt.show()