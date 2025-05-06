import open3d as o3d

# Load the point cloud (use .ply or .pcd as needed)
pcd = o3d.io.read_point_cloud("filtered.ply")  # or "scene_complete.pcd"

# Create a coordinate frame with size 0.1
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

# Visualize both
o3d.visualization.draw_geometries([pcd, axis], window_name="PCD + Axis Viewer") 