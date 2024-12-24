from recursive_branching_tree import tree
from arena import Arena
import numpy as np
import mujoco as mj

if __name__ =="__main__":
  arena = Arena()

  root, spec , bodies = tree()

  # removing contacts
  for body in bodies:
    for other_body in bodies:
        if other_body == body:
            continue
        spec.add_exclude(bodyname1=body,bodyname2=other_body)

  arena.add_fixed_asset(root,[0,0,0.05])

  model = arena.spec.compile()
  # print(f"model.opt::{model.opt}")
  model.opt.wind = np.array([10,0,0])

  ######### Saving Model ###########
  with open("model.xml", "w") as file:
    file.write(arena.spec.to_xml())

  duration = 4   # (seconds)
  framerate = 60  # (Hz)
  frames = []
  slowdown = 2
  r_width, r_height = 640, 480

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
