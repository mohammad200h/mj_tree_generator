import mujoco as mj
import mujoco.viewer
import random
import numpy as np
import math
import uuid


def sampling_from_upper_hemisphere(num_samples, phi_lower_limit=0, phi_upper_limit = np.pi / 2):
    # https://www.youtube.com/watch?v=Ex_g2w4E5lQ&t=201s
    samples = []
    for i in range(num_samples):
        slice = (2 * np.pi)/num_samples
        limit = [i * slice , (i+1) * slice  ]
        theta = np.random.uniform(limit[0],limit[1])  # Azimuthal angle
        phi = np.random.uniform(phi_lower_limit ,phi_upper_limit)    # Polar angle (upper hemisphere only)

        # Spherical to Cartesian conversion
        x = np.sin(phi) * np.cos(theta)
        y = np.sin(phi) * np.sin(theta)
        z = np.cos(phi)

        samples.append( [x, y, z] )

    return samples

def tree(num_child_branch = 4, branch_length = 0.5, thickness = 0.05,
         max_levels = 5, current_body = None,
         spec = None ):

  root, spec = None, None

  rgba = [random.uniform(0, 1),
          random.uniform(0,1),
          random.uniform(0,1), 1]

  # Initialization
  if current_body == None :
    spec = mj.MjSpec()
    current_body = spec.worldbody
    spec.compiler.degree = False

    # Defaults for joint and geom
    main = spec.default()
    main.geom.type = mj.mjtGeom.mjGEOM_CAPSULE
    main.geom.mass = 1

    # Trunk
    pos1 = [0, 0, 0]
    pos2 = [0, 0, branch_length]
    current_body = current_body.add_body(name = "trunk")
    current_body.add_geom( fromto = pos1 + pos2, size = [thickness, 0, 0], rgba = [1,1,0,1])

  thickness = (2/4) * thickness
  branch_length *= 0.6

  sample_vectors = sampling_from_upper_hemisphere(num_child_branch, 0.2, np.pi / 3)

  for i in range(num_child_branch):
    if max_levels <= 0:
      break

    pos1 = [0, 0, 0]
    pos2 = [0,0, branch_length]

    # Branch pose
    z = random.uniform(0.9 * branch_length, branch_length)
    b_pos =  [0, 0, z]
    zaxis = sample_vectors[i]

    # Branch Creation
    branch = current_body.add_body(name = "b" + str(i) + str(uuid.uuid4()) ,
                                   pos = b_pos, zaxis = zaxis )
    branch.add_geom(fromto = pos1 + pos2, size = [thickness, 0, 0], rgba = rgba)

    tree(branch_length = branch_length, thickness = thickness,
         max_levels = max_levels - 1, current_body = branch, spec = spec )

  return root , spec

if __name__ == "__main__":
  root, spec = tree()
  model = spec.compile()
  data = mj.MjData(model)

  # Visualization
  with mj.viewer.launch_passive(
        model = model, data = data, show_left_ui = False, show_right_ui = False
    ) as viewer:
    mj.mjv_defaultFreeCamera(model, viewer.cam)
    mj.mj_forward(model, data)
    while viewer.is_running():
      mj.mj_step(model, data)
      viewer.sync()
