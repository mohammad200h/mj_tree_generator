import mujoco as mj



class Arena:
  def __init__(self):
    self.spec = mj.MjSpec()
    self.model = self.spec.worldbody
    self.spec.compiler.degree = False
    self.scene_option = mj.MjvOption()

    # Make arena with textured floor.
    chequered = self.spec.add_texture(
        name="chequered", type=mj.mjtTexture.mjTEXTURE_2D,
        builtin=mj.mjtBuiltin.mjBUILTIN_CHECKER,
        width=300, height=300, rgb1=[.2, .3, .4], rgb2=[.3, .4, .5])
    grid = self.spec.add_material(
        name='grid', texrepeat=[5, 5], reflectance=.2
        ).textures[mj.mjtTextureRole.mjTEXROLE_RGB] = 'chequered'
    self.model.add_geom(
        type=mj.mjtGeom.mjGEOM_PLANE, size=[2, 2, .1], material='grid')
    for x in [-2, 2]:
      self.model.add_light(pos=[x, -1, 3], dir=[-x, 1, -2])

  def add_tracking_camera(self,body,name="eye",pos=[0,0,1],euler=[0,0,0]):
    # camera
    self.model.add_camera(name=name,
                          mode =mj.mjtCamLight.mjCAMLIGHT_TRACK ,
                          targetbody=body,
                          pos= pos,
                          euler=euler)
    return name

  def visulize_camera(self):
    self.scene_option.flags[mj.mjtVisFlag.mjVIS_CAMERA] = True

  def add_movable_asset(self,body,pos=[0,0,0],prefix='_'):
    frame = self.model.add_frame(pos=pos)
    body = frame.attach_body(body,  prefix,'')
    body.add_freejoint()
    return body

  def add_fixed_asset(self,body,pos=[0,0,0],prefix='_'):
    frame = self.model.add_frame(pos=pos)
    body = frame.attach_body(body,  prefix,'')
    return body

  def remove_asset(self,body):
    self.spec.detach_body(body)