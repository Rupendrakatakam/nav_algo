import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import math

# Load Data
data = pd.read_csv("sim_log.csv")

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-1, 21)
ax.set_ylim(-1, 21)
ax.set_title("God Mode: Dynamic Obstacle Replanning (Complex Map)")
ax.grid(True)

# Draw Static Walls (3-wall S-curve maze matches C++)
# Wall 1: x=4, rows 0..10
for i in range(0, 11):
    ax.add_patch(Rectangle((4 - 0.5, i - 0.5), 1.0, 1.0, color='#333333'))
# Wall 2: x=9, rows 9..19
for i in range(9, 20):
    ax.add_patch(Rectangle((9 - 0.5, i - 0.5), 1.0, 1.0, color='#333333'))
# Wall 3: x=14, rows 0..10
for i in range(0, 11):
    ax.add_patch(Rectangle((14 - 0.5, i - 0.5), 1.0, 1.0, color='#333333'))

ax.plot(18, 18, 'go', markersize=10, label="Goal")

# Robot and Humans
robot_size = 0.8
robot_patch = Rectangle((0, 0), robot_size, robot_size, color='blue', zorder=5, label="Robot")
human1_patch = Circle((0, 0), 0.5, color='red', zorder=6, label="Moving Human 1")
human2_patch = Circle((0, 0), 0.5, color='darkred', zorder=6, label="Moving Human 2")
ax.add_patch(robot_patch)
ax.add_patch(human1_patch)
ax.add_patch(human2_patch)

trail, = ax.plot([], [], 'b--', alpha=0.5)
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

ani = animation.FuncAnimation(fig, update, frames=len(data), interval=50, blit=True)
plt.legend(loc="upper left")
ani.save('simulation.gif', writer='pillow', fps=20)