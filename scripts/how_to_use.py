from dynamics_wrapper import kdl_interface

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

link_0 = String("panda_link0")
link_8 = String("panda_link8")

# Create Interface
interface = kdl_interface("kdl_test")

# Init KDL
response = interface.initialize(link_0, link_8)

q = Float64MultiArray(data=[0, 0, 0, -1.5707, 0, 1.5707, 0])

q_d = Float64MultiArray(data=[0, 0, 0, -1.5707, 0, 0, 0])

qd = Float64MultiArray(data=[0, 1, 0, 0, 0, 0, 0])

pose_euler = interface.pose_euler(q)

pose_error_euler = interface.pose_error_euler(q, q_d)

pose_quaternion = interface.pose_quaternion(q)

pose_error_quaternion = interface.pose_error_quaternion(q, q_d)

jac_geometric = interface.jac_geometric(q)

jac_analytic = interface.jac_analytic(q)

gravity = interface.gravity(q)

coriolis = interface.coriolis(q, qd)

inertia_matrix = interface.inertia_matrix(q)




