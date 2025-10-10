import pinocchio as pin
from os.path import dirname, join, abspath

from pinocchio.visualize import RVizVisualizer

# Load the URDF model from example
# Conversion with str seems to be necessary when executing this file with ipython
# current_dir = abspath(dirname(str(abspath(__file__))))
# print(current_dir)
# model_path = join(current_dir, "example-robot-data/robots")
# mesh_dir = current_dir
# urdf_filename = "talos_reduced.urdf"
# urdf_model_path = join(join(model_path, "talos_data/robots"), urdf_filename)

# Load the URDF model from flygripper
current_dir = abspath(join(dirname(str(abspath(__file__))),".."))
mesh_dir = join(current_dir, "src")
urdf_filename = "flygripper.urdf"
urdf_model_path = urdf_filename

model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())

# data = model.createData()
# NQ = model.nq
# NV = model.nv
# print('Dimension of the configuration vector representation: ' + str(NQ))
# print('Dimension of the velocity: ' + str(NV))

# Create data required by the algorithms
data = model.createData()

# Sample a random configuration
q = pin.randomConfiguration(model)
print('q: %s' % q.T)

# Perform the forward kinematics over the kinematic tree
pin.forwardKinematics(model, data, q)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}"
          .format( name, *oMi.translation.T.flat )))

# Initialize the viewer.
viz = RVizVisualizer(model, collision_model, visual_model)
viz.initViewer()
viz.loadViewerModel("pinocchio")

# Display a robot configuration.
q0 = pin.neutral(model)

viz.display(q0)

# # Display another robot.
# viz2 = RVizVisualizer(model, collision_model, visual_model)
# viz2.initViewer(viz.viewer)
# viz2.loadViewerModel(rootNodeName="pinocchio2")

# q[1] = 1.0
# viz2.display(q)

input("Press enter to exit...")

viz.clean()