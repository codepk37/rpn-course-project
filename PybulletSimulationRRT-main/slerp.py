import numpy as np

# SLERP Function
def slerp(q1, q2, t):
    q1 = np.array(q1)  # Ensure q1 is a NumPy array
    q2 = np.array(q2)  # Ensure q2 is a NumPy array
    
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)
    dot_product = np.dot(q1, q2)

    if dot_product < 0.0:
        q2 = -q2
        dot_product = -dot_product

    dot_product = np.clip(dot_product, -1.0, 1.0)
    theta = np.arccos(dot_product)

    if theta < 1e-6:
        return (1.0 - t) * q1 + t * q2

    sin_theta = np.sin(theta)
    q_t = (np.sin((1.0 - t) * theta) / sin_theta) * q1 + (np.sin(t * theta) / sin_theta) * q2
    return q_t / np.linalg.norm(q_t)

if __name__=='__main__':
    # Define two quaternions
    q1 = np.array([1, 0, 0, 0])  # Identity quaternion
    q2 = np.array([0, 1, 0, 0])  # 180-degree rotation around Y-axis


    # Interpolate and print 100 values
    interpolations = 100
    results = []
    for i in range(interpolations + 1):
        t = i / interpolations  # Calculate t in [0, 1]
        interpolated_q = slerp(q1, q2, t)
        results.append(interpolated_q)
        print(f"t={t:.2f}: {interpolated_q}")
