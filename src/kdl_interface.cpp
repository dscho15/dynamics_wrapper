#include <dynamics_wrapper/kdl_interface.hpp>

#include <ros/ros.h>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace dynamics_wrapper{

    kdl_interface::kdl_interface()
    {}

    std_msgs::Bool 
    kdl_interface::initialize(const std_msgs::String & link_start, const std_msgs::String & link_end)
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

        if(_dyn_solver = std::make_shared<KDL::ChainDynParam>(*_chain.get(), KDL::Vector(0, 0, -9.82)); _dyn_solver == nullptr)
        {
            ROS_ERROR("Failed to create the dynamics solver in KDL");
            return msg;
        }

        _joints = _chain->getNrOfJoints();

        msg.data = 1; _init = true;

        return msg;
    }

    std_msgs::Float64MultiArray 
    kdl_interface::pose_euler(std_msgs::Float64MultiArray q_)
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

    std_msgs::Float64MultiArray 
    kdl_interface::pose_error_euler(std_msgs::Float64MultiArray q_e_, std_msgs::Float64MultiArray q_d_)
    {
        // Msg
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Updata
        auto msg_d = pose_euler(q_d_);
        auto msg_e = pose_euler(q_e_);

        // Resize
        msg.data.resize(6);

        for (int i = 0; i < msg.data.size(); i++)
            msg.data[i] = msg_d.data[i] - msg_e.data[i];

        return msg;
    }

    std_msgs::Float64MultiArray 
    kdl_interface::pose_quaternion(std_msgs::Float64MultiArray q_)
    {
        // Msg
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        if(q_.data.size() != _joints) return msg;

        // KDL Jnt_array, Frame
        KDL::JntArray q(_joints);
        KDL::Frame x;

        // Fill JntArray
        q.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_.data.data(), q_.data.size());

        // Compute [4 x 4] homogenous transformatrion matrix
        _fk_solver->JntToCart(q, x);

        // Extract [3 x 3] rotation matrix
        Eigen::Matrix3d R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x.M.data);

        // Extract Quaternion
        Eigen::Quaterniond quat(R);

        msg.data.resize(7);
        msg.data[0] = x.p.data[0];
        msg.data[1] = x.p.data[1]; 
        msg.data[2] = x.p.data[2];
        msg.data[3] = quat.x();
        msg.data[4] = quat.y();
        msg.data[5] = quat.z();
        msg.data[6] = quat.w();

        return msg;
    }

    std_msgs::Float64MultiArray 
    kdl_interface::pose_error_quaternion(std_msgs::Float64MultiArray q_e_, std_msgs::Float64MultiArray q_d_)
    {
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        if(q_e_.data.size() != q_d_.data.size() and q_e_.data.size() != _joints) return msg;

        auto pose_e = pose_quaternion(q_e_);
        auto pose_d = pose_quaternion(q_d_);

        Eigen::Quaterniond q_e(pose_e.data[6], pose_e.data[3], pose_e.data[4], pose_e.data[5]);
        Eigen::Quaterniond q_d(pose_d.data[6], pose_d.data[3], pose_d.data[4], pose_d.data[5]);
        Eigen::Quaterniond q = q_d * q_e.inverse();

        msg.data.resize(6);
        msg.data[0] = pose_d.data[0] - pose_e.data[0];
        msg.data[1] = pose_d.data[1] - pose_e.data[1];
        msg.data[2] = pose_d.data[2] - pose_e.data[2];
        msg.data[3] = q.x();
        msg.data[4] = q.y();
        msg.data[5] = q.z();

        return msg;
    }

    std_msgs::Float64MultiArray 
    kdl_interface::jac_geometric(std_msgs::Float64MultiArray q_)
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

    std_msgs::Float64MultiArray 
    kdl_interface::jac_analytic(std_msgs::Float64MultiArray q_)
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

    std_msgs::Float64MultiArray 
    kdl_interface::gravity(std_msgs::Float64MultiArray q_)
    {
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Load data into KDL framework
        if(q_.data.size() != _joints) return msg;
        
        // Variables
        KDL::JntArray q(_joints);
        KDL::JntArray g(_joints);

        // Parse joint values
        q.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_.data.data(), q_.data.size());

        // Gravity determination
        _dyn_solver->JntToGravity(q, g);

        msg.data = std::vector<double>(&g.data[0], g.data.data()+g.data.size());

        return msg;
    }

    std_msgs::Float64MultiArray 
    kdl_interface::coriolis(std_msgs::Float64MultiArray q_, std_msgs::Float64MultiArray qd_)
    {
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Load data into KDL framework
        if(q_.data.size() != _joints and q_.data.size() == qd_.data.size()) return msg;

        // Variables
        KDL::JntArray q(_joints), qd(_joints), c(_joints);

        // Parse joint values
        q.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_.data.data(), q_.data.size());
        qd.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qd_.data.data(), qd_.data.size());

        // Coriolis determination
        _dyn_solver->JntToCoriolis(q, qd, c);

        msg.data = std::vector<double>(&c.data[0], c.data.data()+c.data.size());

        return msg;
    }

    std_msgs::Float64MultiArray 
    kdl_interface::inertia_matrix(std_msgs::Float64MultiArray q_)
    {
        std_msgs::Float64MultiArray msg;

        if(_init == false) return msg;

        // Load data into KDL framework
        if(q_.data.size() != _joints) return msg;

        // Variables
        KDL::JntArray q(_joints);
        KDL::JntSpaceInertiaMatrix m(_joints);

        // Parse joint values
        q.data = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_.data.data(), q_.data.size());

        // Mass Matrix
        _dyn_solver->JntToMass(q, m);

        // Parse back to message
        msg.data = std::vector<double>(&m.data(0, 0), m.data.data()+m.data.size());

        return msg;
    }

}
