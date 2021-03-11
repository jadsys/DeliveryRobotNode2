// Generated by gencpp from file delivery_robot/r_emergency_result.msg
// DO NOT EDIT!


#ifndef DELIVERY_ROBOT_MESSAGE_R_EMERGENCY_RESULT_H
#define DELIVERY_ROBOT_MESSAGE_R_EMERGENCY_RESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace delivery_robot
{
template <class ContainerAllocator>
struct r_emergency_result_
{
  typedef r_emergency_result_<ContainerAllocator> Type;

  r_emergency_result_()
    : id()
    , type()
    , time()
    , received_time()
    , received_emergency_cmd()
    , result()
    , errors()  {
    }
  r_emergency_result_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , type(_alloc)
    , time(_alloc)
    , received_time(_alloc)
    , received_emergency_cmd(_alloc)
    , result(_alloc)
    , errors(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _time_type;
  _time_type time;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _received_time_type;
  _received_time_type received_time;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _received_emergency_cmd_type;
  _received_emergency_cmd_type received_emergency_cmd;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _result_type;
  _result_type result;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _errors_type;
  _errors_type errors;





  typedef boost::shared_ptr< ::delivery_robot::r_emergency_result_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::delivery_robot::r_emergency_result_<ContainerAllocator> const> ConstPtr;

}; // struct r_emergency_result_

typedef ::delivery_robot::r_emergency_result_<std::allocator<void> > r_emergency_result;

typedef boost::shared_ptr< ::delivery_robot::r_emergency_result > r_emergency_resultPtr;
typedef boost::shared_ptr< ::delivery_robot::r_emergency_result const> r_emergency_resultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::delivery_robot::r_emergency_result_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::delivery_robot::r_emergency_result_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace delivery_robot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'delivery_robot': ['/home/ros/catkin_ws/src/delivery_robot/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'move_base_msgs': ['/opt/ros/kinetic/share/move_base_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::delivery_robot::r_emergency_result_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_emergency_result_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_emergency_result_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6387b5dc3e134004d23a488ce743a792";
  }

  static const char* value(const ::delivery_robot::r_emergency_result_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6387b5dc3e134004ULL;
  static const uint64_t static_value2 = 0xd23a488ce743a792ULL;
};

template<class ContainerAllocator>
struct DataType< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "delivery_robot/r_emergency_result";
  }

  static const char* value(const ::delivery_robot::r_emergency_result_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string id\n\
string type\n\
string time\n\
string received_time\n\
string received_emergency_cmd\n\
string result\n\
string[] errors\n\
";
  }

  static const char* value(const ::delivery_robot::r_emergency_result_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.time);
      stream.next(m.received_time);
      stream.next(m.received_emergency_cmd);
      stream.next(m.result);
      stream.next(m.errors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct r_emergency_result_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::delivery_robot::r_emergency_result_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::delivery_robot::r_emergency_result_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "time: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.time);
    s << indent << "received_time: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.received_time);
    s << indent << "received_emergency_cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.received_emergency_cmd);
    s << indent << "result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.result);
    s << indent << "errors[]" << std::endl;
    for (size_t i = 0; i < v.errors.size(); ++i)
    {
      s << indent << "  errors[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.errors[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DELIVERY_ROBOT_MESSAGE_R_EMERGENCY_RESULT_H
