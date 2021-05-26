#include <dynamics_wrapper/kdl_interface.hpp>

#include <ros/ros.h>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace dynamics_wrapper{

    kdl_interface::kdl_interface()
    {

    }

    std_msgs::Bool kdl_interface::initialize(const std_msgs::String & link_start, const std_msgs::String & link_end)
    {
        std_msgs::Bool msg;
        msg.data = 0; _init = false;
        
        if (_tree = std::make_shared<KDL::Tree>(); !kdl_parser::treeFromParam("robot_description", *_tree.get())) 
        {
            ROS_ERROR("Failed to construct kdl tree.");
            return msg;
        }

        if(_chain = std::make_shared<KDL::Chain>(); !_tree->getChain(link_start.data, link_end.data, *_chain.get()))
        {
            ROS_ERROR_STREAM("Failed to parse the chain from " << link_start.data << " to " << link_end.data);
            return msg;
        }

        if(_fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(*_chain.get()); _fk_solver == nullptr)
        {
            ROS_ERROR("Failed to create the fk_solver in KDL.");
            return msg;
        }

        if(_jac_solver = std::make_shared<KDL::ChainJntToJacSolver>(*_chain.get()); _jac_solver == nullptr)
        {
            ROS_ERROR("Failed to create the jac_solver in KDL.");
            return msg;
        }

        _joints = _chain->getNrOfJoints();

        msg.data = 1; _init = true;

        return msg;
    }

    std_msgs::Float64MultiArray kdl_interface::car_euler(std_msgs::Float64MultiArray q_)
    {
        // Msg
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Load data into KDL framework
        if(q_.data.size() != _joints) return msg;

        // Resize to 6 [ x y z | alpha beta gamma ]
        msg.data.resize(6);

        // KDL Jnt_array, Frame
        KDL::JntArray q(_joints);
        KDL::Frame x;
        q.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_.data.data(), q_.data.size());

        // Determine the Cartesian coords
        _fk_solver->JntToCart(q, x);

        // Get Translational part
        msg.data[0] = x.p.data[0]; msg.data[1] = x.p.data[1]; msg.data[2] = x.p.data[2];

        // Get EulerZYX
        x.M.GetEulerZYX(msg.data[3], msg.data[4], msg.data[5]);

        return msg;
    }

    std_msgs::Float64MultiArray kdl_interface::car_error_euler(std_msgs::Float64MultiArray q_e_, std_msgs::Float64MultiArray q_d_)
    {
        // Msg
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Updata
        auto msg_d = car_euler(q_d_);
        auto msg_e = car_euler(q_e_);

        // Resize
        msg.data.resize(6);

        for (int i = 0; i < msg.data.size(); i++)
            msg.data[i] = msg_d.data[i] - msg_e.data[i];

        return msg;
    }

    std_msgs::Float64MultiArray kdl_interface::car_error_quaternion(std_msgs::Float64MultiArray q_e_, std_msgs::Float64MultiArray q_d_)
    {
        // Msg
        std_msgs::Float64MultiArray msg;
        
        if(_init == false) return msg;

        if(q_e_.data.size() != q_d_.data.size() and q_d_.data.size() == _joints) return msg;

        // Resize to 6 [x y z | x y z]
        msg.data.resize(6);

        // KDL Jnt_array, Frame
        KDL::JntArray q_e(_joints), q_d(_joints);
        KDL::Frame x_e, x_d;

        // Update the kdl_jnt arrays
        q_e.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_e_.data.data(), q_e_.data.size());
        q_d.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_d_.data.data(), q_d_.data.size());

        // Determine the Cartesian homogenous transformations for each
        _fk_solver->JntToCart(q_e, x_e);
        _fk_solver->JntToCart(q_d, x_d);

        // Rotation matrices
        Eigen::Matrix3d R_e = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_e.M.data);
        Eigen::Matrix3d R_d =  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x_d.M.data);

        // Define quaternions 
        Eigen::Quaterniond quat_d(R_d);
        Eigen::Quaterniond quat_e(R_e);
        Eigen::Quaterniond quat = quat_d * quat_e.inverse();

        // Compute error_quat
        msg.data[0] = x_d.p.data[0] - x_e.p.data[0];
        msg.data[1] = x_d.p.data[1] - x_e.p.data[1]; 
        msg.data[2] = x_d.p.data[2] - x_e.p.data[2];
        msg.data[3] = quat.x();
        msg.data[4] = quat.y();
        msg.data[5] = quat.z();

        return msg;
    }

    std_msgs::Float64MultiArray kdl_interface::jac_geometric(std_msgs::Float64MultiArray q_)
    {
        // Msg
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Load data into KDL framework
        if(q_.data.size() != _joints) return msg;

        // KDL Jnt_array, Frame
        KDL::JntArray q(_joints);
        KDL::Jacobian jac(_joints);
        KDL::Frame x;

        q.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_.data.data(), q_.data.size());
        
        // Compute jacobian
        _jac_solver->JntToJac(q, jac);

        // Transform from eigen to std_msgs
        tf::matrixEigenToMsg(jac.data, msg);

        return msg;
    }

    std_msgs::Float64MultiArray kdl_interface::jac_analytic(std_msgs::Float64MultiArray q_)
    {
        // Msg
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Load data into KDL framework
        if(q_.data.size() != _joints) return msg;
        
        // Defines
        KDL::Jacobian jac(_joints);
        KDL::JntArray q(_joints);
        KDL::Frame x;

        q.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_.data.data(), q_.data.size());
        
        // Forward kinematic solver
        _fk_solver->JntToCart(q, x);

        // Get the angles to compute the transformation
        double alpha, beta, gamma;
        x.M.GetEulerZYX(alpha, beta, gamma);

        // Transformation for ZYX angles
        Eigen::Matrix3d B;
        B << 0,    -std::sin(alpha),   std::cos(beta)*std::cos(alpha), 
             0,     std::cos(alpha),   std::cos(beta)*std::sin(alpha), 
             1,                   0,                  -std::sin(beta);

        // Put it into the transformation matrix
        Eigen::Matrix<double, 6,  6> T = Eigen::MatrixXd::Identity(6, 6);
        T.bottomRightCorner(3, 3) << B;

        // Invert the transformation
        T = T.inverse();

        // Determine jacobian
        _jac_solver->JntToJac(q, jac);

        // Do Jacobian
        Eigen::Matrix<double, 6, -1> T_jac = T * jac.data;

        // Transfrom message
        tf::matrixEigenToMsg(T_jac, msg);
        
        return msg;
    }

}
