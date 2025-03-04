# import numpy as np
# import matplotlib.pyplot as plt

# def dh_transform(theta, d, a, alpha):
#     return np.array([
#         [np.cos(theta), -np.cos(alpha)*np.sin(theta), np.sin(alpha)*np.sin(theta), a*np.cos(theta)],
#         [np.sin(theta), np.cos(alpha)*np.cos(theta), -np.sin(alpha)*np.cos(theta), a*np.sin(theta)],
#         [0, np.sin(alpha), np.cos(alpha), d],
#         [0, 0, 0, 1]
#     ])

# def forward_kinematics(thetas, dh_params):
#     T = np.eye(4)
#     joint_positions = [(0,0,0)]
#     for i, (theta, d, a, alpha) in enumerate(dh_params):
#         T = T @ dh_transform(theta, d, a, alpha)
#         x = T[0,3]
#         y = T[1,3]
#         z = T[2,3]
#         joint_positions.append((x, y, z))
#     return joint_positions

# def intersect_line_segment_aabb(P1, P2, aabb_min, aabb_max):
#     # Parameterize line segment: P(t) = P1 + t*(P2 - P1), t in [0,1]
#     D = P2 - P1
#     P = P1
#     t_min = -np.inf
#     t_max = np.inf
#     for axis in range(3):
#         if D[axis] == 0:
#             if P[axis] < aabb_min[axis] or P[axis] > aabb_max[axis]:
#                 return False, None, None
#             continue
#         t_entry = (aabb_min[axis] - P[axis]) / D[axis]
#         t_exit = (aabb_max[axis] - P[axis]) / D[axis]
#         if t_entry > t_exit:
#             t_entry, t_exit = t_exit, t_entry
#         t_min = max(t_min, t_entry)
#         t_max = min(t_max, t_exit)
#     if t_min <= t_max + 1e-6 and 0 <= t_min <= 1 and 0 <= t_max <= 1:
#         return True, t_min, t_max
#     return False, None, None

# def plot_aabb(ax, aabb_min, aabb_max):
#     vertices = [
#         [aabb_min[0], aabb_min[1], aabb_min[2]],
#         [aabb_min[0], aabb_min[1], aabb_max[2]],
#         [aabb_min[0], aabb_max[1], aabb_min[2]],
#         [aabb_min[0], aabb_max[1], aabb_max[2]],
#         [aabb_max[0], aabb_min[1], aabb_min[2]],
#         [aabb_max[0], aabb_min[1], aabb_max[2]],
#         [aabb_max[0], aabb_max[1], aabb_min[2]],
#         [aabb_max[0], aabb_max[1], aabb_max[2]],
#     ]
#     edges = [
#         (0,1), (0,2), (0,4),
#         (1,3), (1,5),
#         (2,3), (2,6),
#         (3,7),
#         (4,5), (4,6),
#         (5,7),
#         (6,7),
#     ]
#     for edge in edges:
#         v1 = vertices[edge[0]]
#         v2 = vertices[edge[1]]
#         ax.plot([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], 'k-')

# def plot_arm(ax, joint_positions, intersections):
#     for i in range(len(joint_positions)-1):
#         P1 = np.array(joint_positions[i])
#         P2 = np.array(joint_positions[i+1])
#         color = 'g' if i in intersections else 'b'
#         ax.plot([P1[0], P2[0]], [P1[1], P2[1]], [P1[2], P2[2]], color+'-')

# def main():
#     L1, L2 = 1, 1
#     theta1, theta2 = np.pi/4, np.pi/4
#     dh_params = [
#         (theta1, 0, L1, np.pi/2),  # Joint 1: theta, d, a, alpha
#         (theta2, 0, L2, 0)        # Joint 2: theta, d, a, alpha
#     ]
#     joint_positions = forward_kinematics([theta1, theta2], dh_params)
#     aabb_min = np.array([0,0,0])
#     aabb_max = np.array([1,1,1])
#     intersections = []
#     for i in range(len(joint_positions)-1):
#         P1 = np.array(joint_positions[i])
#         P2 = np.array(joint_positions[i+1])
#         intersects, _, _ = intersect_line_segment_aabb(P1, P2, aabb_min, aabb_max)
#         if intersects:
#             intersections.append(i)
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     plot_aabb(ax, aabb_min, aabb_max)
#     plot_arm(ax, joint_positions, intersections)
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     ax.set_xlim(-2, 2)
#     ax.set_ylim(-2, 2)
#     ax.set_zlim(-2, 2)
#     ax.view_init(30, 45)
#     plt.show()

# if __name__ == "__main__":
#     main()


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

L1 = 1
L2 = 1

# Define AABB
aabb_min = np.array([0.5, 0.5])
aabb_max = np.array([1.5, 1.5])

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)

# Plot AABB
ax.add_patch(Rectangle(aabb_min, width=aabb_max[0] - aabb_min[0], height=aabb_max[1] - aabb_min[1], edgecolor='k', facecolor='none'))

line1, = ax.plot([], [], 'b-')
line2, = ax.plot([], [], 'b-')
dot, = ax.plot([], [], 'ro')

def intersect_segment_aabb(P1, P2, aabb_min, aabb_max):
    # Parameterize the segment: P(t) = P1 + t*(P2 - P1), t in [0,1]
    if P2[0] == P1[0]:
        if P1[0] < aabb_min[0] or P1[0] > aabb_max[0]:
            return False
        t_min_y = max(0, (aabb_min[1] - P1[1]) / (P2[1] - P1[1]) if P2[1] != P1[1] else 0)
        t_max_y = min(1, (aabb_max[1] - P1[1]) / (P2[1] - P1[1]) if P2[1] != P1[1] else 1)
        if P2[1] > P1[1]:
            t_entry_y = (aabb_min[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] < aabb_min[1] else 0
            t_exit_y = (aabb_max[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] < aabb_max[1] else 1
        else:
            t_entry_y = (aabb_max[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] > aabb_max[1] else 0
            t_exit_y = (aabb_min[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] > aabb_min[1] else 1
        t_min = max(0, t_entry_y)
        t_max = min(1, t_exit_y)
        return t_min <= t_max
    else:
        if P2[0] > P1[0]:
            t_entry_x = (aabb_min[0] - P1[0]) / (P2[0] - P1[0]) if P1[0] < aabb_min[0] else 0
            t_exit_x = (aabb_max[0] - P1[0]) / (P2[0] - P1[0]) if P1[0] < aabb_max[0] else 1
        else:
            t_entry_x = (aabb_max[0] - P1[0]) / (P2[0] - P1[0]) if P1[0] > aabb_max[0] else 0
            t_exit_x = (aabb_min[0] - P1[0]) / (P2[0] - P1[0]) if P1[0] > aabb_min[0] else 1
        if P2[1] == P1[1]:
            if P1[1] < aabb_min[1] or P1[1] > aabb_max[1]:
                return False
            t_min_x = max(0, t_entry_x)
            t_max_x = min(1, t_exit_x)
            return t_min_x <= t_max_x
        else:
            if P2[1] > P1[1]:
                t_entry_y = (aabb_min[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] < aabb_min[1] else 0
                t_exit_y = (aabb_max[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] < aabb_max[1] else 1
            else:
                t_entry_y = (aabb_max[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] > aabb_max[1] else 0
                t_exit_y = (aabb_min[1] - P1[1]) / (P2[1] - P1[1]) if P1[1] > aabb_min[1] else 1
            t_min_x = max(0, t_entry_x)
            t_max_x = min(1, t_exit_x)
            t_min_y = max(0, t_entry_y)
            t_max_y = min(1, t_exit_y)
            t_min = max(t_min_x, t_min_y)
            t_max = min(t_max_x, t_max_y)
            return t_min <= t_max

def init():
    line1.set_data([],[])
    line2.set_data([],[])
    dot.set_data([],[])
    return line1, line2, dot

def animate(i):
    t = i * 0.1  # Time parameter for animation
    theta1 = np.pi * np.sin(t)  # Example: sinusoidal motion
    theta2 = np.pi * np.cos(t/2)
    
    # Calculate positions
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    
    # Check intersections
    P1_base = np.array([0,0])
    P1 = np.array([x1, y1])
    P2 = np.array([x2, y2])
    
    intersect1 = intersect_segment_aabb(P1_base, P1, aabb_min, aabb_max)
    intersect2 = intersect_segment_aabb(P1, P2, aabb_min, aabb_max)
    
    # Set colors based on intersection
    color1 = 'r' if intersect1 else 'b'
    color2 = 'r' if intersect2 else 'b'
    
    # Update lines
    line1.set_data([0, x1], [0, y1])
    line1.set_color(color1)
    line2.set_data([x1, x2], [y1, y2])
    line2.set_color(color2)
    dot.set_data([x2], [y2])
    
    return line1, line2, dot

anim = FuncAnimation(fig, animate, init_func=init, frames=200, interval=25, blit=True)
plt.show()