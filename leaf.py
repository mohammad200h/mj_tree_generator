import mujoco as mj
import mujoco.viewer


def leaf():

    spec = mj.MjSpec()
    root = spec.worldbody
    spec.compiler.degree = False

    # Defaults for joint and geom
    main = spec.default()
    main.geom.type = mj.mjtGeom.mjGEOM_BOX
    main.geom.mass = 1

    body = root.add_body(name = "leaf" )
    body.add_geom(type =  mj.mjtGeom.mjGEOM_BOX, size = [0.01, 0.01, 0.001],
                  pos = [0.02, 0, 0])
    body.add_geom(type =  mj.mjtGeom.mjGEOM_BOX, size = [0.0071, 0.0071, 0.001],
                  pos = [0.03 , 0, 0],
                  euler = [0, 0, 0.785398])
    body.add_geom(type =  mj.mjtGeom.mjGEOM_BOX, size = [0.0071, 0.0071, 0.001],
                  pos = [0.01 , 0, 0],
                  euler = [0, 0, 0.785398])

    return root, spec


if __name__ == "__main__":
  root, spec = leaf()
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