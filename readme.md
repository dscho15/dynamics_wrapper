# Dynamic Wrapper for KDL

## The purpose

KDL stands for "The Kinematic and Dynamic Library", which takes care of modelling and computation of kinematic chains for serial robots, such as the Franka Emika Panda manipulator.

## The package

The package is a wrapper for the exisiting library *orocos_kdl* to enable for faster development in Python. The code utilizes PyObjects and one should familiarize themself with Numpy's library before expanding the code base.

```
project
│   README.md
│   setup.py
|   package.xml
|   CMakeLists.txt  
│
└───src
│   │   kdl_interface.cpp
|   |   _kdl_interface.cpp
│   └───dynamics_wrapper
│       │   __init__.py
│       │   _kdl_interface_py.py
│   
└───scripts
|   │   
|   └───how_to_use.py
|
└───include/dynamics_wrapper
    |
    └───kdl_interface.hpp
```

## Guide

In the directory ***dynamics_wrapper/scripts/***, a file named "**how_to_use.py**" gives an exemplification on how to make use of the interface. One thing worth noting, *initilization* of the code is necessary before utilization, it requires a **rosparameter** to be loaded onto the ***ROS-server***, which shall contain **URDF** information and has to be labeled as **robot_description** and in addition, it is required to note where the chain **begins** and **ends**, otherwise a runtime-error will occur, followed by termination of the program.

```c++

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

```

The submodule comes with an URDF file for the Franka Emika Panda robot, simply launch **robot_description.launch** and then **how_to_use.py**. A more comprehensive utilization can be found at:

- https://github.com/dscho15/joint_velocity_controller

## Useful links

The following three links gives a better understanding of how to create **.cpp** to **.py** wrappers

- https://github.com/erikfrojdh/python_cpp_example/blob/master/src/my_module.cpp

- https://stackoverflow.com/questions/25494858/creating-numpy-array-in-c-extension-segfaults

- https://github.com/GalacticDynamics-Oxford/Agama/blob/master/src/py_wrapper.cpp
