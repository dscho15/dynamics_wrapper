from dynamics_wrapper import kdl_interface
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

link_0 = String("panda_link0")
link_8 = String("panda_link8")

# Create Interface
interface = kdl_interface("kdl_test")

# Init KDL
response = interface.initialize(link_0, link_8)
# print("The interface is initialized with code: ", response.data)

for i in range(10000):

    q = Float64MultiArray(data=[0, 0, 0, -1.5707, 0, 1.5707, 0])
    # print("Joint-values : ", q.data)

    q_d = Float64MultiArray(data=[0, 0, 0, -1.5707, 0, 0, 0])
    # print("Joint-values : ", q_d.data)

    cart_pose = interface.car_euler(q)
    # print("Cartesian Pose : ", np.round(cart_pose, 3))

    car_error_euler = interface.car_error_euler(q, q_d)
    # print("Cartesian Pose : ", np.round(car_error_euler, 3))

    car_error_quaternion = interface.car_error_quaternion(q, q_d)
    # print("Cartesian Pose : ", np.round(car_error_quaternion, 3))

    jac_geometric = interface.jac_geometric(q)
    # print("Jacobian Geometric : ", np.round(jac_geometric, 3))

    jac_analytic = interface.jac_analytic(q)
    # print("Jacobian Analytic : ", np.round(jac_analytic, 3))





