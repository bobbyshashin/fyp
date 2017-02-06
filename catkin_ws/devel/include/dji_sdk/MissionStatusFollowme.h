// Generated by gencpp from file dji_sdk/MissionStatusFollowme.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_MISSIONSTATUSFOLLOWME_H
#define DJI_SDK_MESSAGE_MISSIONSTATUSFOLLOWME_H


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
struct MissionStatusFollowme_
{
  typedef MissionStatusFollowme_<ContainerAllocator> Type;

  MissionStatusFollowme_()
    : mission_type(0)  {
    }
  MissionStatusFollowme_(const ContainerAllocator& _alloc)
    : mission_type(0)  {
  (void)_alloc;
    }



   typedef uint8_t _mission_type_type;
  _mission_type_type mission_type;




  typedef boost::shared_ptr< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> const> ConstPtr;

}; // struct MissionStatusFollowme_

typedef ::dji_sdk::MissionStatusFollowme_<std::allocator<void> > MissionStatusFollowme;

typedef boost::shared_ptr< ::dji_sdk::MissionStatusFollowme > MissionStatusFollowmePtr;
typedef boost::shared_ptr< ::dji_sdk::MissionStatusFollowme const> MissionStatusFollowmeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
{
  static const char* value()
  {
    return "917010b744881889ec912637e401b269";
  }

  static const char* value(const ::dji_sdk::MissionStatusFollowme_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x917010b744881889ULL;
  static const uint64_t static_value2 = 0xec912637e401b269ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/MissionStatusFollowme";
  }

  static const char* value(const ::dji_sdk::MissionStatusFollowme_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 mission_type\n\
";
  }

  static const char* value(const ::dji_sdk::MissionStatusFollowme_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mission_type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MissionStatusFollowme_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::MissionStatusFollowme_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::MissionStatusFollowme_<ContainerAllocator>& v)
  {
    s << indent << "mission_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mission_type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_MISSIONSTATUSFOLLOWME_H
