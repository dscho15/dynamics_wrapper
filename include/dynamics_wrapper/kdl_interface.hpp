#ifndef PYTHON_BINDINGS_KDL_INTERFACE
#define PYTHON_BINDINGS_KDL_INTERFACE

#include <dynamics_wrapper/kdl_interface.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <memory>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace dynamics_wrapper{

class kdl_interface
{

    public:

        kdl_interface();

        std_msgs::Bool initialize(const std_msgs::String & link_start, const std_msgs::String & link_end);

        std_msgs::Float64MultiArray car_euler(std_msgs::Float64MultiArray q_cur_);

        std_msgs::Float64MultiArray car_error_euler(std_msgs::Float64MultiArray q_cur_, std_msgs::Float64MultiArray q_d_);

        std_msgs::Float64MultiArray car_error_quaternion(std_msgs::Float64MultiArray q_cur_, std_msgs::Float64MultiArray q_d_);

        std_msgs::Float64MultiArray jac_geometric(std_msgs::Float64MultiArray q_cur_);

        std_msgs::Float64MultiArray jac_analytic(std_msgs::Float64MultiArray q_e_);


    private:

        std::shared_ptr<KDL::Tree> _tree;
        
        std::shared_ptr<KDL::Chain> _chain;

        std::shared_ptr<KDL::ChainFkSolverPos_recursive> _fk_solver;

        std::shared_ptr<KDL::ChainJntToJacSolver> _jac_solver;

        int _joints = 7;

        bool _init = false;

};

}

#endif // PYTHON_BINDINGS_TUTORIAL_ADD_TWO_INTS_H