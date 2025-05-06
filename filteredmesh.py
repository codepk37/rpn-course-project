import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree

# Load the filtered point cloud
pcd = o3d.io.read_point_cloud("transformed_filtered1.ply")

# Estimate normals (required for ball pivoting)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

# Compute average distance between points to set ball radius
dists = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(dists)
radius = avg_dist * 2.5  # adjust factor as needed

# Apply Ball Pivoting to generate mesh
radii = [radius]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))

# Interpolate point colors to mesh using nearest neighbor
pcd_points = np.asarray(pcd.points)
pcd_colors = np.asarray(pcd.colors)
mesh_vertices = np.asarray(mesh.vertices)

# KD-tree for nearest neighbor interpolation
kdtree = cKDTree(pcd_points)
_, idx = kdtree.query(mesh_vertices, k=1)
mesh.vertex_colors = o3d.utility.Vector3dVector(pcd_colors[idx])

# Create coordinate axis
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])

o3d.io.write_triangle_mesh("1.obj", mesh)

# Visualize mesh with axis
o3d.visualization.draw_geometries([mesh, axis], mesh_show_back_face=True, window_name="Ball Pivoting Mesh + Axis")
