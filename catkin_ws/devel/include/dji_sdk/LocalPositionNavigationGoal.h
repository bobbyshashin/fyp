// Generated by gencpp from file dji_sdk/LocalPositionNavigationGoal.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_LOCALPOSITIONNAVIGATIONGOAL_H
#define DJI_SDK_MESSAGE_LOCALPOSITIONNAVIGATIONGOAL_H


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
struct LocalPositionNavigationGoal_
{
  typedef LocalPositionNavigationGoal_<ContainerAllocator> Type;

  LocalPositionNavigationGoal_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  LocalPositionNavigationGoal_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;




  typedef boost::shared_ptr< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> const> ConstPtr;

}; // struct LocalPositionNavigationGoal_

typedef ::dji_sdk::LocalPositionNavigationGoal_<std::allocator<void> > LocalPositionNavigationGoal;

typedef boost::shared_ptr< ::dji_sdk::LocalPositionNavigationGoal > LocalPositionNavigationGoalPtr;
typedef boost::shared_ptr< ::dji_sdk::LocalPositionNavigationGoal const> LocalPositionNavigationGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc153912f1453b70ULL;
  static const uint64_t static_value2 = 0x8d221682bc23d9acULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/LocalPositionNavigationGoal";
  }

  static const char* value(const ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#x,y,z are in meters\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocalPositionNavigationGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::LocalPositionNavigationGoal_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_LOCALPOSITIONNAVIGATIONGOAL_H
