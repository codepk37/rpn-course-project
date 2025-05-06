import open3d as o3d
import numpy as np

# Load the original point cloud
pcd = o3d.io.read_point_cloud("5.ply")

# Convert point cloud to numpy array
points = np.asarray(pcd.points)

# Define filtering conditions
x_min, x_max = -0.45, 0.3
y_min, y_max = -0.3, 0.3
z_min, z_max = -1.3, 100

# Apply filter
mask = (points[:, 0] > x_min) & (points[:, 0] < x_max) & \
       (points[:, 1] > y_min) & (points[:, 1] < y_max) & \
         (points[:, 2] > z_min) & (points[:, 2] < z_max)

# Filtered points
filtered_points = points[mask]

# Create new point cloud from filtered points
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

# Optional: retain colors if present
if pcd.has_colors():
    colors = np.asarray(pcd.colors)
    filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

# Save filtered point cloud
o3d.io.write_point_cloud("filtered5.ply", filtered_pcd)

# Create coordinate frame
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

# Visualize
o3d.visualization.draw_geometries([filtered_pcd, axis], window_name="Filtered PCD + Axis Viewer")
