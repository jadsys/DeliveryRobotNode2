// Generated by gencpp from file delivery_robot/r_navi_command.msg
// DO NOT EDIT!


#ifndef DELIVERY_ROBOT_MESSAGE_R_NAVI_COMMAND_H
#define DELIVERY_ROBOT_MESSAGE_R_NAVI_COMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <delivery_robot/r_pose_optional.h>
#include <delivery_robot/r_costmap.h>

namespace delivery_robot
{
template <class ContainerAllocator>
struct r_navi_command_
{
  typedef r_navi_command_<ContainerAllocator> Type;

  r_navi_command_()
    : id()
    , type()
    , time()
    , cmd()
    , destination()
    , costmap()  {
    }
  r_navi_command_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , type(_alloc)
    , time(_alloc)
    , cmd(_alloc)
    , destination(_alloc)
    , costmap(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _time_type;
  _time_type time;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cmd_type;
  _cmd_type cmd;

   typedef  ::delivery_robot::r_pose_optional_<ContainerAllocator>  _destination_type;
  _destination_type destination;

   typedef  ::delivery_robot::r_costmap_<ContainerAllocator>  _costmap_type;
  _costmap_type costmap;





  typedef boost::shared_ptr< ::delivery_robot::r_navi_command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::delivery_robot::r_navi_command_<ContainerAllocator> const> ConstPtr;

}; // struct r_navi_command_

typedef ::delivery_robot::r_navi_command_<std::allocator<void> > r_navi_command;

typedef boost::shared_ptr< ::delivery_robot::r_navi_command > r_navi_commandPtr;
typedef boost::shared_ptr< ::delivery_robot::r_navi_command const> r_navi_commandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::delivery_robot::r_navi_command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::delivery_robot::r_navi_command_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::delivery_robot::r_navi_command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::delivery_robot::r_navi_command_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_navi_command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_navi_command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_navi_command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_navi_command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::delivery_robot::r_navi_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e897c9113f9162ecf1718815f64d8c3d";
  }

  static const char* value(const ::delivery_robot::r_navi_command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe897c9113f9162ecULL;
  static const uint64_t static_value2 = 0xf1718815f64d8c3dULL;
};

template<class ContainerAllocator>
struct DataType< ::delivery_robot::r_navi_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "delivery_robot/r_navi_command";
  }

  static const char* value(const ::delivery_robot::r_navi_command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::delivery_robot::r_navi_command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string id\n\
string type\n\
string time\n\
string cmd\n\
r_pose_optional destination\n\
r_costmap costmap\n\
\n\
================================================================================\n\
MSG: delivery_robot/r_pose_optional\n\
geometry_msgs/Point point\n\
r_angle_optional angle_optional\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: delivery_robot/r_angle_optional\n\
bool valid\n\
r_angle angle\n\
\n\
================================================================================\n\
MSG: delivery_robot/r_angle\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
\n\
================================================================================\n\
MSG: delivery_robot/r_costmap\n\
float64 resolution\n\
uint16 width\n\
uint16 height\n\
r_pose origin\n\
uint8[] cost_value\n\
\n\
================================================================================\n\
MSG: delivery_robot/r_pose\n\
geometry_msgs/Point point\n\
r_angle angle\n\
";
  }

  static const char* value(const ::delivery_robot::r_navi_command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::delivery_robot::r_navi_command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.time);
      stream.next(m.cmd);
      stream.next(m.destination);
      stream.next(m.costmap);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct r_navi_command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::delivery_robot::r_navi_command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::delivery_robot::r_navi_command_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "time: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.time);
    s << indent << "cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cmd);
    s << indent << "destination: ";
    s << std::endl;
    Printer< ::delivery_robot::r_pose_optional_<ContainerAllocator> >::stream(s, indent + "  ", v.destination);
    s << indent << "costmap: ";
    s << std::endl;
    Printer< ::delivery_robot::r_costmap_<ContainerAllocator> >::stream(s, indent + "  ", v.costmap);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DELIVERY_ROBOT_MESSAGE_R_NAVI_COMMAND_H
