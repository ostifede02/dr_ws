import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the coordinates of the points
x_values = [1, 3]  # x-coordinates of the points
y_values = [2, 5]  # y-coordinates of the points
z_values = [3, 4]  # z-coordinates of the points

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
ax.plot(x_values, y_values, z_values, marker='o')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('point to point path')
# Set the face color of the planes to white
ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

# Set axis scale to equal
ax.set_box_aspect([1,1,1])

# Show the plot
plt.show()
