#include <Python.h>
#include <numpy/arrayobject.h>
#include <boost/python.hpp>

#include <string>
#include <ros/serialization.h>
#include <ros/ros.h>
#include <dynamics_wrapper/kdl_interface.hpp>
#include <std_msgs/Float64MultiArray.h>

/*

    ROS Serialization Interface

*/

template <typename M>
M from_python(const std::string str_msg)
{
    size_t serial_size = str_msg.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    for (size_t i = 0; i < serial_size; ++i)
    {
        buffer[i] = str_msg[i];
    }
    ros::serialization::IStream stream(buffer.get(), serial_size);
    M msg;
    ros::serialization::Serializer<M>::read(stream, msg);
    return msg;
}

// serialization interface
template <typename M>
std::string to_python(const M& msg)
{
    size_t serial_size = ros::serialization::serializationLength(msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, msg);
    std::string str_msg;
    str_msg.reserve(serial_size);
    for (size_t i = 0; i < serial_size; ++i)
    {
        str_msg.push_back(buffer[i]);
    }
    return str_msg;
}

/* 

    Python objects interface

    Some very usefull links:

    - https://github.com/erikfrojdh/python_cpp_example/blob/master/src/my_module.cpp
    - https://stackoverflow.com/questions/25494858/creating-numpy-array-in-c-extension-segfaults
    - https://github.com/GalacticDynamics-Oxford/Agama/blob/master/src/py_wrapper.cpp


*/

template<typename DataType>
inline DataType& pyArrayElem(void* arr, npy_intp index)
{
    return *static_cast<DataType*>(PyArray_GETPTR1(static_cast<PyArrayObject*>(arr), index));
}

template<typename DataType>
inline DataType& pyArrayElem(void* arr, npy_intp index1, npy_intp index2)
{
    return *static_cast<DataType*>(PyArray_GETPTR2(static_cast<PyArrayObject*>(arr), index1, index2));
}

PyObject* to_python_vec(const std::vector<double>& vec)
{
    if(PyArray_API == NULL)
    {
        Py_Initialize;
        import_array();
    }

    npy_intp size = vec.size();
    PyObject *arr = PyArray_SimpleNew(1, &size, NPY_DOUBLE);
    if(!arr)
        return arr;
    for(int i = 0; i < size; i++)
        pyArrayElem<double>(arr, i) = vec[i];
    return arr;
}

PyObject* to_python_mat(const std::vector<double>& mat, const int & cols)
{
    if(PyArray_API == NULL)
    {
        Py_Initialize;
        import_array();
    }
    int rows = static_cast<npy_intp>(mat.size()/cols);
    npy_intp size[] = { static_cast<npy_intp>(rows), static_cast<npy_intp>(cols)};
    PyObject *arr = PyArray_SimpleNew(2, size, NPY_DOUBLE);
    if(!arr)
        return arr;
    int ii = 0;
    for(int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
                pyArrayElem<double>(arr, i, j) = mat[ii++];
    return arr;
}

/*

    KDL Interface Wrapper

*/

class kdl_interface_wrapper : public dynamics_wrapper::kdl_interface
{
  public:

    kdl_interface_wrapper() : kdl_interface()
    {

    }

    std::string initialize(const std::string& link_base_, const std::string& link_end_)
    {
        std_msgs::String link_base = from_python<std_msgs::String>(link_base_);
        std_msgs::String link_end = from_python<std_msgs::String>(link_end_);
        std_msgs::Bool msg = kdl_interface::initialize(link_base, link_end);
        return to_python<std_msgs::Bool>(msg);
    }

    PyObject * car_euler(const std::string& q_)
    {  
        std_msgs::Float64MultiArray q = from_python<std_msgs::Float64MultiArray>(q_);
        std_msgs::Float64MultiArray msg = kdl_interface::car_euler(q);
        PyObject * arr = to_python_vec(msg.data);
        return arr;
    }

    PyObject * car_error_euler(const std::string & q_e_, const std::string & q_d_)
    {
        std_msgs::Float64MultiArray q_e = from_python<std_msgs::Float64MultiArray>(q_e_);
        std_msgs::Float64MultiArray q_d = from_python<std_msgs::Float64MultiArray>(q_d_);
        std_msgs::Float64MultiArray msg = kdl_interface::car_error_euler(q_e, q_d);
        PyObject * arr = to_python_vec(msg.data);
        return arr;
    }

    PyObject * car_error_quaternion(const std::string & q_e_, const std::string & q_d_)
    {
        std_msgs::Float64MultiArray q_e = from_python<std_msgs::Float64MultiArray>(q_e_);
        std_msgs::Float64MultiArray q_d = from_python<std_msgs::Float64MultiArray>(q_d_);
        std_msgs::Float64MultiArray msg = kdl_interface::car_error_quaternion(q_e, q_d);
        PyObject * arr = to_python_vec(msg.data);
        return arr;
    }

    PyObject * jac_geometric(const std::string & q_)
    {
        std_msgs::Float64MultiArray q = from_python<std_msgs::Float64MultiArray>(q_);
        std_msgs::Float64MultiArray msg = kdl_interface::jac_geometric(q);
        PyObject * arr = to_python_mat(msg.data, q.data.size());
        return arr;
    }

    PyObject * jac_analytic(const std::string & q_)
    {
        std_msgs::Float64MultiArray q = from_python<std_msgs::Float64MultiArray>(q_);
        std_msgs::Float64MultiArray msg = kdl_interface::jac_analytic(q);
        PyObject * arr = to_python_mat(msg.data, q.data.size());
        return arr;
    }

};


BOOST_PYTHON_MODULE(_kdl_interface_cpp)
{
    boost::python::class_<kdl_interface_wrapper>("kdl_interface_wrapper", boost::python::init<>())
        .def("initialize", &kdl_interface_wrapper::initialize)
        .def("car_euler", &kdl_interface_wrapper::car_euler)
        .def("car_error_euler", &kdl_interface_wrapper::car_error_euler)
        .def("car_error_quaternion", &kdl_interface_wrapper::car_error_quaternion)
        .def("jac_geometric", &kdl_interface_wrapper::jac_geometric)
        .def("jac_analytic", &kdl_interface_wrapper::jac_analytic);
}