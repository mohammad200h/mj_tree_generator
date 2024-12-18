import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from recursive_branching_tree import sampling_from_upper_hemisphere


def plot_spherical_coordinate_vectors(points):
    soa = []
    # adding origin to vectors
    for p in points:
        point = [float(num) for num in p]
        vector = [0,0,0] + point
        soa.append(vector)



    soa = np.array(soa)

    X, Y, Z, U, V, W = zip(*soa)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(X, Y, Z, U, V, W)
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    plt.show()


vectors = sampling_from_upper_hemisphere(4)
plot_spherical_coordinate_vectors(vectors)