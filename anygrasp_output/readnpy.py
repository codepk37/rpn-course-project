import numpy as np

# Load the saved gripper data
gripper_data = np.load("gripperdata_last.npy", allow_pickle=True).item()

# Extract the first gripper grasp pose
translation = gripper_data['translations'][0]   # shape (3,)
rotation = gripper_data['rotations'][0]         # shape (3,3)

# Create 4x4 transformation matrix
T = np.eye(4)
T[:3, :3] = rotation
T[:3, 3] = translation

print("Transformation Matrix:\n", T)

# Optional: Also print other data
print("Widths:", gripper_data['widths'][0])
print("Heights:", gripper_data['heights'][0])
print("Scores:", gripper_data['scores'][0])
