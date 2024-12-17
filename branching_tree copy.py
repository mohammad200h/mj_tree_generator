import mujoco as mj
import mujoco.viewer
import random
import numpy as np
import math


def circle_coordinates(radius, angle):
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    return x, y

def tree(num_child_branch = 6):
  spec = mj.MjSpec()
  root = spec.worldbody
  spec.compiler.degree = False

  # Defining color
  spec.add_material(name = 'yellow', rgba = [1,1,0,1])

  # Defaults for joint and geom
  main = spec.default()
  main.joint.damping = 10
  main.geom.material = 'yellow'
  main.geom.type = mj.mjtGeom.mjGEOM_CAPSULE
  main.geom.mass = 1

  # branch height position range
  bhpr = [0.3 , 0.5]

  root = spec.worldbody

  # trunk
  pos1 = [0, 0, 0]
  pos2 = [0, 0, 0.5]
  trunk = root.add_body(name="trunk")
  trunk.add_geom( fromto= pos1 + pos2, size=[0.05, 0, 0])

  for i in range(num_child_branch):
    angle = 2 * i * np.pi / num_child_branch
    pos1 = [0, 0, 0]
    x,y = circle_coordinates(0.3,angle)
    pos2 = [x,y, 0.2]
    z = random.uniform(0.3, 0.5)
    b_pos = [0, 0, z]

    branch = trunk.add_body(name = "b"+str(i) ,pos = b_pos)
    branch.add_geom(fromto= pos1 + pos2, size=[0.03, 0, 0])

  return root , spec


if __name__ == "__main__":
  root, spec = tree()
  model = spec.compile()
  data = mj.MjData(model)

  # visualization
  with mj.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
    mj.mjv_defaultFreeCamera(model, viewer.cam)
    mj.mj_forward(model, data)
    while viewer.is_running():
        mj.mj_step(model, data)
        viewer.sync()
