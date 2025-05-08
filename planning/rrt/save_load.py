import pybullet as p
import pybullet_data
import numpy as np
import time
import os
from collision_utils import get_collision_fn, get_joint_positions

def save_path(path, object_name,id, folder="waypoints"):
    """Save a path to a NumPy .npy file."""
    if path is None:
        print("No path to save")
        return
        
    if not os.path.exists(folder):
        os.makedirs(folder)
    
    path_array = np.array(path)
    filename = os.path.join(folder, f"{object_name}_{id}.npy")
    np.save(filename, path_array)
    print(f"Path saved to {filename}")

def load_path(object_name,id, folder="waypoints"):
    """Load a path from a NumPy .npy file."""
    filename = os.path.join(folder, f"{object_name}_{id}.npy")
    
    if not os.path.exists(filename):
        print(f"No saved path found for {object_name} {id}")
        return None
    
    path_array = np.load(filename)
    path = [tuple(config) for config in path_array]
    print(f"Loaded path from {filename}")
    return path


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)
