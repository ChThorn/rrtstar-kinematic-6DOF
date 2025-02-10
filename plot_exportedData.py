import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

def load_data(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    original_path = []
    smoothed_path = []
    obstacles = []
    current_section = None

    for line in lines:
        line = line.strip()
        if line == "original_path":
            current_section = "original_path"
            continue
        elif line == "smoothed_path":
            current_section = "smoothed_path"
            continue
        elif line == "obstacles":
            current_section = "obstacles"
            continue

        if current_section == "original_path":
            x, y, z = map(float, line.split(','))
            original_path.append([x, y, z])
        elif current_section == "smoothed_path":
            x, y, z = map(float, line.split(','))
            smoothed_path.append([x, y, z])
        elif current_section == "obstacles":
            min_x, min_y, min_z, max_x, max_y, max_z = map(float, line.split(','))
            obstacles.append([[min_x, min_y, min_z], [max_x, max_y, max_z]])

    return np.array(original_path), np.array(smoothed_path), obstacles

def plot_3d(original_path, smoothed_path, obstacles):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot original path
    ax.plot(original_path[:, 0], original_path[:, 1], original_path[:, 2], 
            label="Original Path", color="orange", linestyle="--", linewidth=2)

    # Plot smoothed path
    ax.plot(smoothed_path[:, 0], smoothed_path[:, 1], smoothed_path[:, 2], 
            label="Smoothed Path", color="blue", linewidth=2)

    # Plot obstacles
    for obs in obstacles:
        min_point, max_point = obs
        vertices = [
            [min_point, [max_point[0], min_point[1], min_point[2]], [max_point[0], max_point[1], min_point[2]], [min_point[0], max_point[1], min_point[2]]],
            [min_point, [min_point[0], min_point[1], max_point[2]], [max_point[0], min_point[1], max_point[2]], [max_point[0], min_point[1], min_point[2]]],
            [[min_point[0], min_point[1], max_point[2]], [min_point[0], max_point[1], max_point[2]], [max_point[0], max_point[1], max_point[2]], [max_point[0], min_point[1], max_point[2]]],
            [[min_point[0], max_point[1], min_point[2]], [min_point[0], max_point[1], max_point[2]], [max_point[0], max_point[1], max_point[2]], [max_point[0], max_point[1], min_point[2]]]
        ]
        poly = Poly3DCollection(vertices, alpha=0.5, edgecolor='k', facecolor='gray')
        ax.add_collection3d(poly)

    # Set labels and legend
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.title("RRT* Path Planning in 3D")
    plt.show()

# Load data from file
filename = "/home/thornch/Documents/Cpp/PathPlanning/rrtstar_sixDoF/path_data.txt"
original_path, smoothed_path, obstacles = load_data(filename)

# Plot the data
plot_3d(original_path, smoothed_path, obstacles)