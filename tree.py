import mujoco as mj

"""
Trunk-to-Branch Ratio in Excurrent Trees
Diameter Ratio:

The trunk is significantly thicker than the branches at the same height.
Typical Diameter Ratio: The trunk's diameter at a given height is about 3-5 times
larger than the lateral branches emerging from it.
Cross-Sectional Area (CSA) Ratio:

The cross-sectional area of the trunk at a given level is approximately
equal to the sum of the cross-sectional areas of the lateral branches at
that level (consistent with Leonardo's Rule).
Length Proportions:

Lateral branches are shorter than the central trunk and typically follow a
tapered length distribution as they ascend the trunk.
Example: A trunk segment 10 meters tall might have lateral branches that are 1-3 meters
long at the base and progressively shorter toward the apex.

Excurrent Trees (Conical Shape):

Trees like pines, spruces, and firs with a dominant central trunk and
shorter lateral branches.

Typical Ratio: Branch length is 10-30% of the trunk height.
Example: A 30-meter (100-foot) tall pine might have branches around 3–9 meters (10–30 feet) long.
"""
# trees optimize for structural stability - tree_cs_ratio
branch_to_trunk_ratio_cross_section_range = [3, 5] # 3 to 5 time
# tree branch length to trunk ratio - tree_length_ratio
branch_to_trunk_length_percentage_range = [0.1, 0.3] # 10 to 30 %

total_tree_mass = 200
trunk_height = 2
trunk_diameter = 0.1

# tree depth determines number branches per depth
# more branch at each level consume more mass which leads to
# less mass left for next level
tree_depth = 3

def tree(tree_cs_ratio = 3,
         tree_length_ratio = 0.2,
         left_mass = 200,
         parent_branch_height = 2,
         parent_trunk_diameter = 0.1
        ):
  spec = mj.MjSpec()
  spec.compiler.degree = False

  spec.add_material(name = "yellow",rgba = [1,1,0,1])

  # defaults for joint and geom
  main = spec.default()
  main.joint.damping = 10
  main.geom.material = "yellow"
  main.geom.type = mj.mjtGeom.mjGEOM_CAPSULE

  model = spec.worldbody

def dumb_tree():
  spec = mj.MjSpec()
  spec.compiler.degree = False

  spec.add_material(name = "yellow",rgba = [1,1,0,1])

  # defaults for joint and geom
  main = spec.default()
  main.joint.damping = 10
  main.geom.material = "yellow"
  main.geom.type = mj.mjtGeom.mjGEOM_CAPSULE

  model = spec.worldbody
  body = model










