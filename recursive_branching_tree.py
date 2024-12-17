import mujoco as mj
import mujoco.viewer
import random
import numpy as np
import math
import uuid

from branch_angles import (rotation_matrix_to_quaternion,
                           align_z_to_vector,
                           random_vector_upper_hemisphere
)

def circle_coordinates(radius, angle):
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    return x, y

def tree(num_child_branch = 6, branch_length = 0.5, thickness = 0.05,bhpr = [0.3 , 0.5],
         max_levels = 4, current_body = None,
         spec = None ):

  root, spec = None, None

  if current_body == None :
    spec = mj.MjSpec()
    current_body = spec.worldbody
    spec.compiler.degree = False

    # Defining color
    spec.add_material(name = 'yellow', rgba = [1,1,0,1])

    # Defaults for joint and geom
    main = spec.default()
    main.joint.damping = 10
    main.geom.material = 'yellow'
    main.geom.type = mj.mjtGeom.mjGEOM_CAPSULE
    main.geom.mass = 1

    # trunk
    pos1 = [0, 0, 0]
    pos2 = [0, 0, branch_length]
    current_body = current_body.add_body(name = "trunk")
    current_body.add_geom( fromto = pos1 + pos2, size = [thickness, 0, 0])


  # z position of branch on trunk
  thickness = (2/3) * thickness
  bhpr = [i*2/3 for i in bhpr]
  branch_length *= 2/3
  rgba = [random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),1]
  for i in range(num_child_branch):
    if max_levels <= 0:
      break
    z_angle = 2 * i * np.pi / num_child_branch
    x_angle = random.uniform(0.5,2)

    pos1 = [0, 0, 0]
    pos2 = [0,0, 0.3]

    z = random.uniform(bhpr[0],bhpr[1])


    b_pos =  [0, 0, z]
    # quat
    target_vector = random_vector_upper_hemisphere()
    R = align_z_to_vector(target_vector)
    quat = rotation_matrix_to_quaternion(R.as_matrix())

    branch = current_body.add_body(name = "b" + str(i) + str(uuid.uuid4()) ,
                                   pos = b_pos, quat = quat )
    branch.add_geom(fromto = pos1 + pos2, size = [thickness, 0, 0], rgba = rgba)

    tree(branch_length = branch_length,thickness = thickness,
            bhpr= bhpr , max_levels= max_levels - 1,
            current_body = branch, spec = spec )

  return root , spec

if __name__ == "__main__":
  root, spec = tree()
  model = spec.compile()
  data = mj.MjData(model)

  # visualization
  with mj.viewer.launch_passive(
        model = model, data = data, show_left_ui = False, show_right_ui = False
    ) as viewer:
    mj.mjv_defaultFreeCamera(model, viewer.cam)
    mj.mj_forward(model, data)
    while viewer.is_running():
      mj.mj_step(model, data)
      viewer.sync()
