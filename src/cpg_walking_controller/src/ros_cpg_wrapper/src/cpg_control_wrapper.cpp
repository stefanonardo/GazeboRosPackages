#include <boost/python.hpp>

#include <string>

#include <ros/serialization.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

#include <ros_cpg_wrapper/cpg_control.h>

/* Read a ROS message from a serialized string.
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

/* Write a ROS message into a serialized string.
*/
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

class CpgControllerWrapper : public cpg_control::CpgController
{
  public:
    CpgControllerWrapper() : CpgController() {}

    std::string GetNumIterations()
    {
      std_msgs::Int64 numIterations = CpgController::GetNumIterations();

      return to_python(numIterations);
    }
    
    void SetNumIterations(const std::string& msg)
    {
      std_msgs::Int64 iterationsMsg = from_python<std_msgs::Int64>(msg);
      CpgController::SetNumIterations(iterationsMsg);
    }
    
    void SetCpgParameter0(const std::string& msg)
    {
      std_msgs::Float64 parameterMsg = from_python<std_msgs::Float64>(msg);
      CpgController::SetCpgParameter0(parameterMsg);
    }
    
    void SetCpgParameter1(const std::string& msg)
    {
      std_msgs::Float64 parameterMsg = from_python<std_msgs::Float64>(msg);
      CpgController::SetCpgParameter1(parameterMsg);
    }
    
    void SetCpgParameter2(const std::string& msg)
    {
      std_msgs::Float64 parameterMsg = from_python<std_msgs::Float64>(msg);
      CpgController::SetCpgParameter2(parameterMsg);
    }
};

BOOST_PYTHON_MODULE(_cpg_control_wrapper_cpp)
{
  boost::python::class_<CpgControllerWrapper>("CpgControllerWrapper", boost::python::init<>())
    .def("GetNumIterations", &CpgControllerWrapper::GetNumIterations)
    .def("SetNumIterations", &CpgControllerWrapper::SetNumIterations)
    .def("SetCpgParameter0", &CpgControllerWrapper::SetCpgParameter0)
    .def("SetCpgParameter1", &CpgControllerWrapper::SetCpgParameter1)
    .def("SetCpgParameter2", &CpgControllerWrapper::SetCpgParameter2)
    ;
}