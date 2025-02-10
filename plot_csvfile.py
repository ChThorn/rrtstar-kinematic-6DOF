import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Read the CSV file
df = pd.read_csv('build/path_data.csv')

# Create 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot path
ax.plot(df['x'], df['y'], df['z'], 'b-', linewidth=2, label='Path')
ax.scatter(df['x'], df['y'], df['z'], c='red', marker='o', s=50)

# Plot start and end points
ax.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], 
          c='green', marker='*', s=200, label='Start')
ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], 
          c='red', marker='*', s=200, label='Goal')

# Add labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('RRT* Path')

# Add legend
ax.legend()

# Add grid
ax.grid(True)

# Save plot
plt.savefig('path_visualization.png')
plt.show()

# Optional: Create a second plot showing joint angles over path
plt.figure(figsize=(12, 6))
for i in range(6):
    plt.plot(df[f'joint{i+1}'], label=f'Joint {i+1}')

plt.xlabel('Node Index')
plt.ylabel('Joint Angle (rad)')
plt.title('Joint Angles Along Path')
plt.legend()
plt.grid(True)
plt.savefig('joint_angles.png')
plt.show()