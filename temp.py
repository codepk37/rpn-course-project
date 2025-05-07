import open3d as o3d

# Load the PLY file as a point cloud
pcd = o3d.io.read_point_cloud("combined_icp_transformed.ply")

# Create a coordinate frame (axis) with length 0.5
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

# Check if the point cloud is loaded
if not pcd.is_empty():
    print("Point cloud loaded successfully")
else:
    print("Failed to load point cloud")

# Visualize the point cloud along with the coordinate axes
o3d.visualization.draw_geometries([pcd, axis])
