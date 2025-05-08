import numpy as np

def lerp(start, end, t):
    """
    Perform Linear Interpolation (LERP) between two points in 3D space.
    
    Parameters:
        start: numpy array, shape (3,) - Initial position [x, y, z]
        end: numpy array, shape (3,) - Final position [x, y, z]
        t: float - Interpolation parameter in range [0, 1]
        
    Returns:
        numpy array, shape (3,) - Interpolated position
    """
    start = np.array(start)
    end = np.array(end)
    return (1 - t) * start + t * end


if __name__ =='__main__':
    # Define the initial and final poses
    pose_start = np.array([0, 0, 7])  # Example: Initial position
    pose_end = np.array([10, 10, 10]) # Example: Final position

    # Interpolate and print 100 values
    interpolations = 100
    results = []
    for i in range(interpolations + 1):
        t = i / interpolations  # Calculate t in [0, 1]
        interpolated_pose = lerp(pose_start, pose_end, t)
        results.append(interpolated_pose)
        print(f"t={t:.2f}: {interpolated_pose}")
