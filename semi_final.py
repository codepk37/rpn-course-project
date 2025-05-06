import open3d as o3d
import numpy as np

# Load the point clouds
pcd1 = o3d.io.read_point_cloud("icp_filtered4.ply")
pcd2 = o3d.io.read_point_cloud("icp_filtered5.ply")
pcd3 = o3d.io.read_point_cloud("transformed_filtered1.ply")

# Check they loaded properly
if not (pcd1.has_points() and pcd2.has_points() and pcd3.has_points()):
    print("One or more point clouds failed to load or are empty.")
    exit()

# Concatenate point clouds
combined = pcd1 + pcd2 + pcd3

# Create rotation matrices
Rz = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, np.pi])        # 180° about Z
Ry = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.pi/2, 0])     # 90° about new Y

# Apply both rotations (note: Ry is applied after Rz)
combined.rotate(Rz, center=(0, 0, 0))  # First rotate 180° about Z
combined.rotate(Ry, center=(0, 0, 0))  # Then rotate 90° about new Y

# Visualize with coordinate axis
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
o3d.visualization.draw_geometries([combined, axis], window_name="Rotated Concatenated Point Cloud")

# Save transformed cloud
o3d.io.write_point_cloud("combined_icp_transformed.ply", combined)
