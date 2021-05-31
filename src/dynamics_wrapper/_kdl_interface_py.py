import rospy

from io import BytesIO

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

from dynamics_wrapper._kdl_interface_cpp import kdl_interface_wrapper


class kdl_interface(object):

    def __init__(self, node_name):
        """ 
            Construct the CPP-interface and a cpp nodehandle.
        """
        self._kdl_interface_wrapper = kdl_interface_wrapper()

        # init ros
        roscpp_init(node_name, [])

    def _to_cpp(self, msg):
        """
            Return a serialized string from a ROS message
        """
        buf = BytesIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """
            Return a ROS message from a serialized string
        """
        msg = cls()
        return msg.deserialize(str.encode(str_msg))

    def initialize(self, link_base, link_ee):
        """
            Initialize the kdl_interface in cpp
        """
        if not isinstance(link_base, String):
            rospy.ROSException('Argument 1 is not a std_msgs/String')
        if not isinstance(link_ee, String):
            rospy.ROSException('Argument 2 is not a std_msgs/String')
        link_b = self._to_cpp(link_base)
        link_e = self._to_cpp(link_ee)
        msg = self._kdl_interface_wrapper.initialize(link_b, link_e)
        return self._from_cpp(msg, Bool)

    def pose_euler(self, q):
        """
            Return a pose_euler (ZYX) given the joint-angles
        """
        if not isinstance(q, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        q = self._to_cpp(q)
        msg = self._kdl_interface_wrapper.pose_euler(q)
        return msg

    def pose_error_euler(self, q_e, q_d):
        """
            Return a pose_euler error (ZYX) given end_effector and desired joint-angles
        """
        if not isinstance(q_e, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        if not isinstance(q_d, Float64MultiArray):
            rospy.ROSException(
                'Argument 2 is not a std_msgs/Float64MultiArray')
        q_e = self._to_cpp(q_e)
        q_d = self._to_cpp(q_d)
        msg = self._kdl_interface_wrapper.pose_error_euler(q_e, q_d)
        return msg
    
    def pose_quaternion(self, q):
        """
            Return a pose with quaternion [7 x 1]
        """
        if not isinstance(q, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        q = self._to_cpp(q)
        msg = self._kdl_interface_wrapper.pose_quaternion(q)
        return msg

    def pose_error_quaternion(self, q_e, q_d):
        """
            Return a pose_quat error given end_effector and desired joint-angles
        """
        if not isinstance(q_e, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        if not isinstance(q_d, Float64MultiArray):
            rospy.ROSException(
                'Argument 2 is not a std_msgs/Float64MultiArray')
        q_e = self._to_cpp(q_e)
        q_d = self._to_cpp(q_d)
        msg = self._kdl_interface_wrapper.pose_error_quaternion(q_e, q_d)
        return msg

    def jac_geometric(self, q):
        """
            Return the geometric jacobian given joint-angles
        """
        if not isinstance(q, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        q = self._to_cpp(q)
        msg = self._kdl_interface_wrapper.jac_geometric(q)
        return msg

    def jac_analytic(self, q):
        """
            Return the analytical jacobian (euler ZYX) given joint-angles
        """
        if not isinstance(q, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        q = self._to_cpp(q)
        msg = self._kdl_interface_wrapper.jac_analytic(q)
        return msg

    def gravity(self, q):
        """
            Return a gravity vector
        """
        if not isinstance(q, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        q = self._to_cpp(q)
        msg = self._kdl_interface_wrapper.gravity(q)
        return msg
    
    def coriolis(self, q, qd):
        """
            Return a coriolis vector
        """
        if not isinstance(q, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        if not isinstance(qd, Float64MultiArray):
            rospy.ROSException(
                'Argument 2 is not a std_msgs/Float64MultiArray')
        q = self._to_cpp(q)
        qd = self._to_cpp(qd)
        msg = self._kdl_interface_wrapper.coriolis(q, qd)
        return msg
    
    def inertia_matrix(self, q):
        """
            Return the analytical jacobian (euler ZYX) given joint-angles
        """
        if not isinstance(q, Float64MultiArray):
            rospy.ROSException(
                'Argument 1 is not a std_msgs/Float64MultiArray')
        q = self._to_cpp(q)
        msg = self._kdl_interface_wrapper.inertia_matrix(q)
        return msg
