import mujoco as mj
import mujoco.viewer
import random
import numpy as np
import math
import uuid

from leaf import leaf

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

def tree(damping = None, stiffness = None, j_range = None, num_child_branch = 3,  branch_length = 0.5, thickness = 0.05,
         max_levels = 4, current_body = None,
         spec = None, bodies =[] ):

  root, spec = None, None

  rgba = [random.uniform(0, 1),
          random.uniform(0, 1),
          random.uniform(0, 1), 1]

  # Initialization
  if current_body == None :
    spec = mj.MjSpec()
    current_body = spec.worldbody
    spec.compiler.degree = False

    # Defaults for joint and geom
    main = spec.default()
    main.geom.type = mj.mjtGeom.mjGEOM_CAPSULE
    main.geom.mass = 0.001

    # Joint Defaults
    main.joint.type = mj.mjtJoint.mjJNT_BALL
    if j_range !=None:
      main.joint.range = [0,j_range]

    if damping != None:
      main.joint.damping = damping
    if stiffness != None:
      main.joint.stiffness = stiffness


    # Trunk
    pos1 = [0, 0, 0]
    pos2 = [0, 0, branch_length]
    current_body = current_body.add_body(name = "trunk")
    current_body.add_geom( fromto = pos1 + pos2, size = [thickness, 0, 0], rgba = [1,1,0,1])
    root = current_body
    bodies.append(current_body.name)

  if max_levels <= 0:
      # Create  leafs
      slice = np.pi * 2/3
      for l in range(3):
        leaf = current_body.add_body( name = "leaf" + str(uuid.uuid4()),
                                      pos = [ 0,0, branch_length + thickness ],
                                      euler = [0, 0, slice * l ]
                                      )

        leaf.add_geom(type =  mj.mjtGeom.mjGEOM_BOX, size = [0.01, 0.01, 0.001],
                      pos = [0.02, 0, 0], rgba = rgba)
        leaf.add_geom(type =  mj.mjtGeom.mjGEOM_BOX, size = [0.0071, 0.0071, 0.001],
                      pos = [0.03 , 0, 0],
                      euler = [0, 0, 0.785398],rgba = rgba)
        leaf.add_geom(type =  mj.mjtGeom.mjGEOM_BOX, size = [0.0071, 0.0071, 0.001],
                      pos = [0.01 , 0, 0],
                      euler = [0, 0, 0.785398],rgba = rgba)
        bodies.append(leaf.name)
  else:
    thickness = (2/4) * thickness
    branch_length *= 0.6
    sample_vectors = sampling_from_upper_hemisphere(num_child_branch, 0.2, np.pi / 3)
    for i in range(num_child_branch):
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

      branch.add_joint(name = "b" + str(i) + str(uuid.uuid4()))

      bodies.append(branch.name)

      tree(branch_length = branch_length, thickness = thickness,
           max_levels = max_levels - 1, current_body = branch, spec = spec )


  return root , spec, bodies

if __name__ == "__main__":
  root, spec, _ = tree()
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

# https://mujoco.readthedocs.io/en/latest/overview.html#divergence