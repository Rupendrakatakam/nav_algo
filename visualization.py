import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import math
import numpy as np

# Load Data
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
data_path = os.path.join(script_dir, "sim_log.csv")
costmap_path = os.path.join(script_dir, "costmap.csv")

data = pd.read_csv(data_path)
costmap = pd.read_csv(costmap_path, header=None).values

fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(-1, 21)
ax.set_ylim(-1, 21)
ax.set_title("God Mode: Dynamic Obstacle Replanning (Complex Map)")
ax.grid(True)

# Draw Costmap Background (0=white, 100=black)
cmap_bg = np.zeros((20, 20))
for y in range(20):
    for x in range(20):
        cmap_bg[y][x] = costmap[y][x] / 100.0  # Normalize to 0-1

ax.imshow(cmap_bg, extent=[-0.5, 19.5, 19.5, -0.5], cmap='gray', alpha=0.5, zorder=0)

# Draw Static Walls (3-wall S-curve maze matches C++)
# Wall 1: x=4, rows 0..10
for i in range(0, 11):
    ax.add_patch(Rectangle((4 - 0.5, i - 0.5), 1.0, 1.0, color='#333333', zorder=1))
# Wall 2: x=9, rows 9..19
for i in range(9, 20):
    ax.add_patch(Rectangle((9 - 0.5, i - 0.5), 1.0, 1.0, color='#333333', zorder=1))
# Wall 3: x=14, rows 0..10
for i in range(0, 11):
    ax.add_patch(Rectangle((14 - 0.5, i - 0.5), 1.0, 1.0, color='#333333', zorder=1))

ax.plot(18, 18, 'go', markersize=10, label="Goal", zorder=10)

# Robot and Humans
robot_size = 1.2
robot_patch = Rectangle((0, 0), robot_size, robot_size, color='blue', zorder=5, label="Robot", alpha=0.7)
human1_patch = Circle((0, 0), 0.5, color='red', zorder=6, label="Moving Human 1")
human2_patch = Circle((0, 0), 0.5, color='darkred', zorder=6, label="Moving Human 2")
ax.add_patch(robot_patch)
ax.add_patch(human1_patch)
ax.add_patch(human2_patch)

trail, = ax.plot([], [], 'b--', alpha=0.5, zorder=4)
hx, hy = [], []

def update(frame):
    row = data.iloc[frame]
    
    # Update Robot
    rx, ry, rtheta = row['rx'], row['ry'], row['rtheta']
    off_x = rx - (robot_size/2)*math.cos(rtheta) + (robot_size/2)*math.sin(rtheta)
    off_y = ry - (robot_size/2)*math.sin(rtheta) - (robot_size/2)*math.cos(rtheta)
    robot_patch.set_xy((off_x, off_y))
    robot_patch.angle = math.degrees(rtheta)
    
    hx.append(rx)
    hy.append(ry)
    trail.set_data(hx, hy)
    
    # Update Humans
    human1_patch.center = (row['hx1'], row['hy1'])
    human2_patch.center = (row['hx2'], row['hy2'])
    
    return robot_patch, human1_patch, human2_patch, trail

# Down-sample frames to keep GIF generation fast
FRAME_SKIP = 5  # show every 5th simulation step (adjust for smoother/faster output)
ani = animation.FuncAnimation(fig, update, frames=range(0, len(data), FRAME_SKIP), interval=50, blit=False)
plt.legend(loc="upper left")
# Save as MP4 video (ffmpeg) – faster and more reliable than GIF
print('Saving video to simulation.mp4...')
ani.save('simulation.mp4', writer='ffmpeg', fps=30)