import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import math

# Load Data
data = pd.read_csv("sim_log.csv")

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-1, 16)
ax.set_ylim(-1, 16)
ax.set_title("God Mode: Dynamic Obstacle Replanning")
ax.grid(True)

# Draw Static Wall (Matches C++ grid)
for i in range(4, 11):
    ax.add_patch(Rectangle((5 - 0.5, i - 0.5), 1.0, 1.0, color='#333333'))

ax.plot(13, 13, 'go', markersize=10, label="Goal")

# Robot and Human
robot_size = 0.8
robot_patch = Rectangle((0, 0), robot_size, robot_size, color='blue', zorder=5, label="Robot")
human_patch = Circle((0, 0), 0.5, color='red', zorder=6, label="Moving Human")
ax.add_patch(robot_patch)
ax.add_patch(human_patch)

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
    
    # Update Human
    human_patch.center = (row['hx'], row['hy'])
    
    return robot_patch, human_patch, trail

ani = animation.FuncAnimation(fig, update, frames=len(data), interval=50, blit=True)
plt.legend(loc="upper left")
ani.save('simulation.gif', writer='pillow', fps=20)