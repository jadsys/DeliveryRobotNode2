// Generated by gencpp from file delivery_robot/r_pose.msg
// DO NOT EDIT!


#ifndef DELIVERY_ROBOT_MESSAGE_R_POSE_H
#define DELIVERY_ROBOT_MESSAGE_R_POSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>
#include <delivery_robot/r_angle.h>

namespace delivery_robot
{
template <class ContainerAllocator>
struct r_pose_
{
  typedef r_pose_<ContainerAllocator> Type;

  r_pose_()
    : point()
    , angle()  {
    }
  r_pose_(const ContainerAllocator& _alloc)
    : point(_alloc)
    , angle(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point_type;
  _point_type point;

   typedef  ::delivery_robot::r_angle_<ContainerAllocator>  _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::delivery_robot::r_pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::delivery_robot::r_pose_<ContainerAllocator> const> ConstPtr;

}; // struct r_pose_

typedef ::delivery_robot::r_pose_<std::allocator<void> > r_pose;

typedef boost::shared_ptr< ::delivery_robot::r_pose > r_posePtr;
typedef boost::shared_ptr< ::delivery_robot::r_pose const> r_poseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::delivery_robot::r_pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::delivery_robot::r_pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace delivery_robot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'delivery_robot': ['/home/azu001/catkin_ws/src/delivery_robot/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'move_base_msgs': ['/home/azu001/catkin_ws/src/navigation_msgs/move_base_msgs/msg', '/home/azu001/catkin_ws/devel_isolated/move_base_msgs/share/move_base_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::delivery_robot::r_pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::delivery_robot::r_pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::delivery_robot::r_pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::delivery_robot::r_pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::delivery_robot::r_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "68cc13657fadf0a694f8bfd643d94f73";
  }

  static const char* value(const ::delivery_robot::r_pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x68cc13657fadf0a6ULL;
  static const uint64_t static_value2 = 0x94f8bfd643d94f73ULL;
};

template<class ContainerAllocator>
struct DataType< ::delivery_robot::r_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "delivery_robot/r_pose";
  }

  static const char* value(const ::delivery_robot::r_pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::delivery_robot::r_pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point point\n\
r_angle angle\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: delivery_robot/r_angle\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
";
  }

  static const char* value(const ::delivery_robot::r_pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::delivery_robot::r_pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct r_pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::delivery_robot::r_pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::delivery_robot::r_pose_<ContainerAllocator>& v)
  {
    s << indent << "point: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.point);
    s << indent << "angle: ";
    s << std::endl;
    Printer< ::delivery_robot::r_angle_<ContainerAllocator> >::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DELIVERY_ROBOT_MESSAGE_R_POSE_H
