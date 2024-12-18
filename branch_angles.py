import numpy as np
from scipy.spatial.transform import Rotation as R
import math

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
# https://www.youtube.com/watch?v=4WRmjKDit2I
def rotation_matrix_to_quaternion(rotation_matrix):
    """
    Convert a rotation matrix to a quaternion.
    """
    rotation = R.from_matrix(rotation_matrix)
    return rotation.as_quat()

def sampling_from_upper_hemisphere(num_samples,phi_lower_limit=0, phi_upper_limit = np.pi / 2):
    samples = []
    samples_angle = []
    for i in range(num_samples):
        slice = (2 * np.pi)/4
        limit = [i * slice , (i+1) * slice  ]
        theta = np.random.uniform(limit[0],limit[1])  # Azimuthal angle
        phi = np.random.uniform(phi_lower_limit ,phi_upper_limit)    # Polar angle (upper hemisphere only)

        # Spherical to Cartesian conversion
        x = np.sin(phi) * np.cos(theta)
        y = np.sin(phi) * np.sin(theta)
        z = np.cos(phi)

        samples_angle.append([theta,phi])
        samples.append( [x, y, z] )

    return samples ,samples_angle

def get_frame_rotation(target_in_parent_frame):

    # mujoco makes capsules with z going through the capsule
    # Therefore if both child and parent frame are on top of each other
    # we can assume a unit vector [0, 0, 1] in both frames

    # Now we are going to rotate the child frame in a way that unit vector [0, 0, 1]
    # is on top of target vector in parent frame

    # target is a vector sampled from a upper hemisphere made in parent frame
    # therefore we have the following equation v_parent = R . v_child
    # v_parent = target vector in parent frame
    # v_child is [0, 0, 1]
    # we have to find R

    # R is Rz . Ry . Rx
    # Finally we extract rpy from R

    # v_parent = R . [0,0,1]^T -> v_parent = [[cos(beta) cos (gama)],[cos(beta) sin(gamma)],[-sin(beta)]]
    x = target_in_parent_frame[0]
    y = target_in_parent_frame[1]
    z = target_in_parent_frame[1]

    print(f"x::{x}")
    print(f"y::{y}")
    print(f"z::{z}")

    beta = math.asin(-z)
    gamma = math.asin(y/beta)

    return beta, gamma


