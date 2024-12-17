import mujoco as mj
import mujoco.viewer

def tree():
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

  root = spec.worldbody

  # level 1
  pos1 =[0,0,0]
  pos2 =[0,0,0.5]
  pbody = root.add_body(name="trunk")
  pbody.add_geom( fromto= pos1 + pos2, size=[0.05,0,0])

  # level 2
  pos3  = [0,0.2,0.6]
  body1 = pbody.add_body(name="branch1")
  body1.add_geom(fromto = pos2+pos3, size=[0.03,0,0] )

  pos_neg_3  = [0,-0.2,0.6]
  body2 = pbody.add_body(name="branch2")
  body2.add_geom(fromto = pos2+pos_neg_3, size=[0.03,0,0] )

  # level 3
  pos4 = [0,0.3,0.7]
  body = body1.add_body(name="sub-branch")
  body.add_geom(fromto = pos3+pos4, size=[0.02,0,0] )





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







