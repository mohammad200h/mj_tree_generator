from recursive_branching_tree import tree
from arena import Arena
import numpy as np
import mujoco as mj
import cv2

from tqdm import tqdm

if __name__ =="__main__":


  damping = {
    "start":10,
    "end":100,
    "step":10
  }

  for d in tqdm(range(damping["start"],damping["end"],damping["step"])):
    print(f"-----recording for damping: {d}----")
    arena = Arena()
    root, spec , bodies = tree(damping = d)
    # removing contacts
    for body in bodies:
      for other_body in bodies:
          if other_body == body:
              continue
          spec.add_exclude(bodyname1=body,bodyname2=other_body)

    arena.add_fixed_asset(root,[0,0,0.05])

    model = arena.spec.compile()
#     print(f"model.opt::{model.opt}")
#     model.opt.wind = np.array([10,0,0])
    # model.opt.timestep = 0.00005
    data = mj.MjData(model)

#     print(f"warning::{data.warning}")

    ######### Saving Model ###########
    with open("model.xml", "w") as file:
      file.write(arena.spec.to_xml())

    duration = 10   # (seconds)
    framerate = 60  # (Hz)
    frames = []
    slowdown = 2
    r_width, r_height = 640, 480

    mj.mj_resetData(model, data)
    with mj.Renderer(model, width = r_width, height = r_height) as renderer:
      while data.time < duration:
        mj.mj_step(model, data)
        if len(frames) <  data.time * framerate  :

          renderer.update_scene(data)
          pixels = renderer.render()
          frames.append(pixels)

    # Define video writer
    out = cv2.VideoWriter('damping_'+str(d)+'.mp4', cv2.VideoWriter_fourcc(*'mp4v'), framerate, (r_width, r_height))
    for frame in frames:
        out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))  # Convert RGB to BGR for OpenCV

    out.release()
    print("Video saved as "+'damping_'+str(d)+'.mp4')
