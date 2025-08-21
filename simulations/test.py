import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Sample data
x = [1, 2, 3, 4]
y = [5, 6, 7, 8]
z = [9, 10, 11, 12]

# Create a 3D figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot scatter points
ax.scatter(x, y, z, c='r', marker='o')

# Draw lines connecting sequential points
#ax.plot(x, y, z, c='b')

# Example: Draw a line between specific points (e.g., first and last)
ax.plot([x[0], x[-1]], [y[0], y[-1]], [z[0], z[-1]], c='g', linestyle='--')

plt.show()