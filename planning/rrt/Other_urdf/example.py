import pybullet as p
import pybullet_data
import time

# Initial environment set-up
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
plane = p.loadURDF("plane.urdf")
control_dt = 1. / 240.

# Camera properties
camera_distance = 0.8
camera_yaw = 135.0
camera_pitch = -45.0
camera_target_position = [0.0, 0.0, 0.3]

# Set camera properties
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Object start pose
object1_start_position = [0.0, 0.0, 0.1]
object1_start_orientation_e = [0.0, 0.0, 0.0]
object1_start_orientation_q = p.getQuaternionFromEuler(object1_start_orientation_e)

# You can load more objects like this:
object2_start_position = [-0.5, 0.0, 0.1]
object2_start_orientation_e = [0.0, 0.0, 0.0]
object2_start_orientation_q = p.getQuaternionFromEuler(object2_start_orientation_e)

object3_start_position = [0.0, -0.5, 0.1]
object3_start_orientation_e = [0.0, 0.0, 0.0]
object3_start_orientation_q = p.getQuaternionFromEuler(object3_start_orientation_e)
# etc.

# Scale of the objects
global_scaling = 0.08
import os
os.chdir("./")

# Load the object in the environment
object1_model = p.loadURDF("002_master_chef_can.urdf", object1_start_position, object1_start_orientation_q, useFixedBase=False, globalScaling=global_scaling)

# You can load more objects like this:
object2_model = p.loadURDF("003_cracker_box.urdf", object2_start_position, object2_start_orientation_q, useFixedBase=False, globalScaling=global_scaling)
object3_model = p.loadURDF("/home/pavan/Desktop/my_pybullet-main/YcbChipsCan/model.urdf", object3_start_position, object3_start_orientation_q, useFixedBase=False, globalScaling=1)


import random

# List of URDF objects
urdf_objects = [
    "002_master_chef_can.urdf", "003_cracker_box.urdf", "004_sugar_box.urdf", "005_tomato_soup_can.urdf",
    "006_mustard_bottle.urdf", "007_tuna_fish_can.urdf", "008_pudding_box.urdf", "009_gelatin_box.urdf",
    "010_potted_meat_can.urdf", "011_banana.urdf", "012_strawberry.urdf", "013_apple.urdf",
    "014_lemon.urdf", "015_peach.urdf", "016_pear.urdf", "017_orange.urdf", "018_plum.urdf",
    "019_pitcher_base.urdf", "021_bleach_cleanser.urdf", "022_windex_bottle.urdf", "024_bowl.urdf",
    "025_mug.urdf", "026_sponge.urdf", "028_skillet_lid.urdf", "029_plate.urdf", "030_fork.urdf",
    "031_spoon.urdf", "032_knife.urdf", "033_spatula.urdf", "035_power_drill.urdf", "036_wood_block.urdf",
    "037_scissors.urdf", "038_padlock.urdf", "040_large_marker.urdf", "042_adjustable_wrench.urdf",
    "043_phillips_screwdriver.urdf", "044_flat_screwdriver.urdf", "048_hammer.urdf",
    "050_medium_clamp.urdf", "051_large_clamp.urdf", "052_extra_large_clamp.urdf",
    "053_mini_soccer_ball.urdf", "054_softball.urdf", "055_baseball.urdf", "065-c_cups.urdf",
    "065-d_cups.urdf", "065-e_cups.urdf", "072-a_toy_airplane.urdf", "073-f_lego_duplo.urdf",
    "077_rubiks_cube.urdf"
]

print(len(urdf_objects))



x_bounds = (-1, 1)  # X-axis bounds
y_bounds = (-1, 1)  # Y-axis bounds
z_bounds = (0.2, 0.5)  # Z-axis bounds
object_start_orientation_q = p.getQuaternionFromEuler([0, 0, 0])  # Neutral orientation
global_scaling = 0.08  # Scaling factor

# Generate a random position within specified bounds
def generate_random_position(x_bounds, y_bounds, z_bounds):
    x = random.uniform(*x_bounds)
    y = random.uniform(*y_bounds)
    z = random.uniform(*z_bounds)
    return [x, y, z]

# Load objects in batches of 5
batch_size = 25
for i in range(0, len(urdf_objects), batch_size):
    batch = urdf_objects[i:i+batch_size]
    loaded_objects = []
    
    # Load objects in the current batch
    for obj in batch:
        position = generate_random_position(x_bounds, y_bounds, z_bounds)
        print(f"Loading {obj} at position {position}")
        obj_id = p.loadURDF(obj, position, object_start_orientation_q, useFixedBase=False, globalScaling=global_scaling)
        loaded_objects.append(obj_id)
    
    # Wait for visualization
    print(f"Visualizing batch {i // batch_size + 1}...")
    time.sleep(5)  # Adjust delay as needed for visualization
    
    # Remove objects from the simulation
    for obj_id in loaded_objects:
        p.removeBody(obj_id)
    print(f"Batch {i // batch_size + 1} removed.")
    
# Disconnect PyBullet
p.disconnect()