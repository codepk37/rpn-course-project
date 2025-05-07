import open3d as o3d
import numpy as np
# Load the point cloud
pcd = o3d.io.read_point_cloud("last.ply")
ori = o3d.io.read_point_cloud("combined_icp_transformed.ply")


theta = np.pi / 2
R_y = np.array([
    [ np.cos(theta), 0, np.sin(theta)],
    [ 0,             1, 0            ],
    [-np.sin(theta), 0, np.cos(theta)]
])
T_inv = np.eye(4)
T_inv[:3, :3] = R_y.T

# Reflection across the YZ-plane (flip X)
Reflect_X = np.eye(4)
Reflect_X[0, 0] = -1
# Combine the transformations
Combined = Reflect_X @ T_inv


# Apply the combined transformation
pcd.transform(Combined)


#grasp data
gripper_data = np.load("gripperdata_last.npy", allow_pickle=True).item()
translation = gripper_data['translations'][0]   # (3,)
rotation = gripper_data['rotations'][0]         # (3, 3)
T = np.eye(4)
T[:3, :3] = rotation
T[:3, 3] = translation

print("Original Grasp Transformation:\n", T)
T_new = Combined @ T
print("Transformed Grasp Matrix:\n", T_new)
new_rotation = T_new[:3, :3]
new_translation = T_new[:3, 3]

sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
sphere.paint_uniform_color([1, 0, 0])  # Red color
sphere.translate(new_translation)
## grasp end

# Create a coordinate frame (axis) of size 0.4
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4, origin=[0, 0, 0])

# Visualize the point cloud along with the axis
o3d.visualization.draw_geometries([pcd,ori,sphere, axis],
                                  window_name="Open3D - last.ply with Axis",
                                  width=800,
                                  height=600,
                                  point_show_normal=False)
