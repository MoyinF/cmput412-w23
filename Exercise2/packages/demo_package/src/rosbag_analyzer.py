import rosbag
import matplotlib.pyplot as plt
import numpy as np

# Open rosbag containing worldframe values
bag = rosbag.Bag('/data/bags/world_frame.bag')

# Read each timestamp, x, y, and theta into an array
timestamps = []
x_vals = []
y_vals = []
theta_vals = []

total_vals = 0
total_msgs = 0

first_timestamp = None
rel_timestamps = []

for topic, msg, t in bag.read_messages(topics=['timestamp', 'x', 'y', 'theta']):
    
    if topic == "timestamp":
        timestamps.append(msg.data)
        total_vals += 1
    elif topic == "x":
        x_vals.append(msg.data)
    elif topic == "y":
        y_vals.append(msg.data)
    elif topic == "theta":
        theta_vals.append(msg.data)
        
    total_msgs += 1
    
print("Total messages: " + str(total_msgs))
print("Total vals: " + str(total_vals))

# close the rosbag
bag.close()

# Plot x values with respect to time:
xpoints = np.array(timestamps)
xpoints = xpoints - xpoints[0]

ypoints = np.array(x_vals)

plt.title("X Position in the World Frame vs Time")
plt.xlabel("Seconds")
plt.ylabel("X (metres)")

plt.plot(xpoints, ypoints)
plt.savefig("/data/logs/xworld_vs_time.png")
plt.clf()

# Plot the y values with respect to time
xpoints = np.array(timestamps)
xpoints = xpoints - xpoints[0]

ypoints = np.array(y_vals)

plt.title("Y Position in the World Frame vs Time")
plt.xlabel("Seconds")
plt.ylabel("Y (metres)")

plt.plot(xpoints, ypoints)
plt.savefig("/data/logs/yworld_vs_time.png")
plt.clf()

# Plot the theta values with respect to time
xpoints = np.array(timestamps)
xpoints = xpoints - xpoints[0]

ypoints = np.array(theta_vals)

plt.title("Theta Position in the World Frame vs Time")
plt.xlabel("Seconds")
plt.ylabel("Theta (radians)")

plt.plot(xpoints, ypoints)
plt.savefig("/data/logs/thetaworld_vs_time.png")
plt.clf()

# Plot the x values with respect to y values
xpoints = np.array(x_vals)
ypoints = np.array(y_vals)

plt.title("X, Y Positions in the World Frame")
plt.xlabel("X (metres)")
plt.ylabel("Y (metres)")

plt.plot(xpoints, ypoints)
plt.savefig("/data/logs/xworld_vs_yworld.png")
plt.clf()


>>>>>>> 51a9d3a43969793fa8797d8517df910d6177995b
