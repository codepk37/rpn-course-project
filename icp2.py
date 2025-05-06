import open3d as o3d
import numpy as np

# Load source (to be aligned) and target (reference) point clouds
source = o3d.io.read_point_cloud("transformed_filtered4.ply")
target = o3d.io.read_point_cloud("transformed_filtered1.ply")

# Optional: downsample for faster ICP
source_down = source.voxel_down_sample(voxel_size=0.02)
target_down = target.voxel_down_sample(voxel_size=0.02)

# Estimate normals for ICP
source_down.estimate_normals()
target_down.estimate_normals()

# Run ICP
threshold = 2  # max correspondence distance (adjust if needed)
trans_init = np.eye(4)  # initial transformation (identity)

print("Running ICP...")
reg_p2p = o3d.pipelines.registration.registration_icp(
    source_down, target_down, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

# Show transformation
print("Transformation Matrix:")
print(reg_p2p.transformation)

# Transform original source cloud and visualize
source.transform(reg_p2p.transformation)
o3d.visualization.draw_geometries([source, target], window_name="Aligned Point Clouds")

# Optional: save aligned cloud
o3d.io.write_point_cloud("icp_filtered4.ply", source)
