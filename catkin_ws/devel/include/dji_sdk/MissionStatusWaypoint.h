// Generated by gencpp from file dji_sdk/MissionStatusWaypoint.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONSTATUSWAYPOINT_H
#define DJI_SDK_MESSAGE_MISSIONSTATUSWAYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dji_sdk
{
template <class ContainerAllocator>
struct MissionStatusWaypoint_
{
  typedef MissionStatusWaypoint_<ContainerAllocator> Type;

  MissionStatusWaypoint_()
    : mission_type(0)
    , target_waypoint(0)
    , current_status(0)
    , error_code(0)  {
    }
  MissionStatusWaypoint_(const ContainerAllocator& _alloc)
    : mission_type(0)
    , target_waypoint(0)
    , current_status(0)
    , error_code(0)  {
  (void)_alloc;
    }



   typedef uint8_t _mission_type_type;
  _mission_type_type mission_type;

   typedef uint8_t _target_waypoint_type;
  _target_waypoint_type target_waypoint;

   typedef uint8_t _current_status_type;
  _current_status_type current_status;

   typedef uint8_t _error_code_type;
  _error_code_type error_code;




  typedef boost::shared_ptr< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> const> ConstPtr;

}; // struct MissionStatusWaypoint_

typedef ::dji_sdk::MissionStatusWaypoint_<std::allocator<void> > MissionStatusWaypoint;

typedef boost::shared_ptr< ::dji_sdk::MissionStatusWaypoint > MissionStatusWaypointPtr;
typedef boost::shared_ptr< ::dji_sdk::MissionStatusWaypoint const> MissionStatusWaypointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/home/bobby/fyp/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/msg', '/home/bobby/fyp/catkin_ws/devel/share/dji_sdk/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f2b7b1e7f32be55abc541c1b7552d41";
  }

  static const char* value(const ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f2b7b1e7f32be55ULL;
  static const uint64_t static_value2 = 0xabc541c1b7552d41ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionStatusWaypoint";
  }

  static const char* value(const ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 mission_type\n\
uint8 target_waypoint\n\
uint8 current_status\n\
uint8 error_code\n\
";
  }

  static const char* value(const ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mission_type);
      stream.next(m.target_waypoint);
      stream.next(m.current_status);
      stream.next(m.error_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MissionStatusWaypoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionStatusWaypoint_<ContainerAllocator>& v)
  {
    s << indent << "mission_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mission_type);
    s << indent << "target_waypoint: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.target_waypoint);
    s << indent << "current_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.current_status);
    s << indent << "error_code: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.error_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONSTATUSWAYPOINT_H
