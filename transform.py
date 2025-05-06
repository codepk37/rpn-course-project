import open3d as o3d
import numpy as np

# Load the point cloud
input_file = "filtered2.ply"
output_file = "transformed_filtered2.ply"
point_cloud = o3d.io.read_point_cloud(input_file)

# Define the transformation matrix
transformation_matrix = np.linalg.inv(np.array([
    [-0.218, -0.964, -0.151, 0.314],
    [-0.623, 0.257, -0.739, 0.417],
    [0.751, -0.067, -0.657, 0.119],
    [0.000, 0.000, 0.000, 1.000]
]))

# Visualize the transformed point cloud
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

# Apply the transformation
point_cloud.transform(transformation_matrix)
o3d.visualization.draw_geometries([point_cloud,axis], window_name="Transformed Point Cloud")

# Save the transformed point cloud
o3d.io.write_point_cloud(output_file, point_cloud)

print(f"Transformation applied and saved to {output_file}")