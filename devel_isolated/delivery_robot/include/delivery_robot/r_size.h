// Generated by gencpp from file delivery_robot/r_size.msg
// DO NOT EDIT!


#ifndef DELIVERY_ROBOT_MESSAGE_R_SIZE_H
#define DELIVERY_ROBOT_MESSAGE_R_SIZE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <delivery_robot/r_corner.h>

namespace delivery_robot
{
template <class ContainerAllocator>
struct r_size_
{
  typedef r_size_<ContainerAllocator> Type;

  r_size_()
    : robot_radius(0.0)
    , inflation_radius(0.0)
    , footprint()  {
    }
  r_size_(const ContainerAllocator& _alloc)
    : robot_radius(0.0)
    , inflation_radius(0.0)
    , footprint(_alloc)  {
  (void)_alloc;
    }



   typedef double _robot_radius_type;
  _robot_radius_type robot_radius;

   typedef double _inflation_radius_type;
  _inflation_radius_type inflation_radius;

   typedef std::vector< ::delivery_robot::r_corner_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::delivery_robot::r_corner_<ContainerAllocator> >::other >  _footprint_type;
  _footprint_type footprint;





  typedef boost::shared_ptr< ::delivery_robot::r_size_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::delivery_robot::r_size_<ContainerAllocator> const> ConstPtr;

}; // struct r_size_

typedef ::delivery_robot::r_size_<std::allocator<void> > r_size;

typedef boost::shared_ptr< ::delivery_robot::r_size > r_sizePtr;
typedef boost::shared_ptr< ::delivery_robot::r_size const> r_sizeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::delivery_robot::r_size_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::delivery_robot::r_size_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace delivery_robot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'delivery_robot': ['/home/azu001/catkin_ws/src/delivery_robot/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'move_base_msgs': ['/home/azu001/catkin_ws/src/navigation_msgs/move_base_msgs/msg', '/home/azu001/catkin_ws/devel_isolated/move_base_msgs/share/move_base_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::delivery_robot::r_size_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::delivery_robot::r_size_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_size_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_size_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_size_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_size_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::delivery_robot::r_size_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8b9808e3d883fc3fa50abb52807ae2b";
  }

  static const char* value(const ::delivery_robot::r_size_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8b9808e3d883fc3ULL;
  static const uint64_t static_value2 = 0xfa50abb52807ae2bULL;
};

template<class ContainerAllocator>
struct DataType< ::delivery_robot::r_size_<ContainerAllocator> >
{
  static const char* value()
  {
    return "delivery_robot/r_size";
  }

  static const char* value(const ::delivery_robot::r_size_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::delivery_robot::r_size_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 robot_radius\n\
float64 inflation_radius\n\
r_corner[] footprint\n\
\n\
================================================================================\n\
MSG: delivery_robot/r_corner\n\
float64 x\n\
float64 y\n\
";
  }

  static const char* value(const ::delivery_robot::r_size_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::delivery_robot::r_size_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.robot_radius);
      stream.next(m.inflation_radius);
      stream.next(m.footprint);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct r_size_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::delivery_robot::r_size_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::delivery_robot::r_size_<ContainerAllocator>& v)
  {
    s << indent << "robot_radius: ";
    Printer<double>::stream(s, indent + "  ", v.robot_radius);
    s << indent << "inflation_radius: ";
    Printer<double>::stream(s, indent + "  ", v.inflation_radius);
    s << indent << "footprint[]" << std::endl;
    for (size_t i = 0; i < v.footprint.size(); ++i)
    {
      s << indent << "  footprint[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::delivery_robot::r_corner_<ContainerAllocator> >::stream(s, indent + "    ", v.footprint[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DELIVERY_ROBOT_MESSAGE_R_SIZE_H
