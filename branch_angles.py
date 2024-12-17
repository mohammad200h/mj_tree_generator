import numpy as np
from scipy.spatial.transform import Rotation as R




def rotation_matrix_to_quaternion(rotation_matrix):
    """
    Convert a rotation matrix to a quaternion.
    """
    rotation = R.from_matrix(rotation_matrix)
    return rotation.as_quat()

def random_vector_upper_hemisphere():
    """
    Generate a random unit vector on the upper hemisphere (z > 0).
    """
    # Random angles
    theta = np.random.uniform(0, 2 * np.pi)  # Azimuthal angle
    phi = np.random.uniform(0, np.pi / 2)    # Polar angle (upper hemisphere only)

    # Spherical to Cartesian conversion
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)

    return np.array([x, y, z])

def align_z_to_vector(target_vector):
    """
    Compute a rotation matrix to align [0,0,1] with the given target vector.
    """
    # Normalize the target vector
    target_vector = target_vector / np.linalg.norm(target_vector)

    # Original z-axis
    z_axis = np.array([0, 0, 1])

    # Compute the rotation axis
    rotation_axis = np.cross(z_axis, target_vector)
    rotation_axis_norm = np.linalg.norm(rotation_axis)

    if rotation_axis_norm < 1e-6:  # Vectors are already aligned
        return np.eye(3)

    rotation_axis = rotation_axis / rotation_axis_norm

    # Compute the rotation angle
    cos_theta = np.dot(z_axis, target_vector)
    angle = np.arccos(np.clip(cos_theta, -1, 1))

    # Construct the rotation using Rodrigues' formula
    rotation = R.from_rotvec(angle * rotation_axis)
    return rotation

# Generate a random target vector in the upper hemisphere
target_vector = random_vector_upper_hemisphere()
print(f"Target vector: {target_vector}")

# Compute the rotation
rotation = align_z_to_vector(target_vector)

# Print the rotation matrix
print("Rotation matrix:")
print(rotation.as_matrix())

# Convert rotation matrix to quaternion
rotation_quaternion = rotation_matrix_to_quaternion(rotation.as_matrix())
print("Rotation quaternion:")
print(rotation_quaternion)

# Verify the result
aligned_vector = rotation.apply([0, 0, 1])
print(f"Aligned vector: {aligned_vector}")