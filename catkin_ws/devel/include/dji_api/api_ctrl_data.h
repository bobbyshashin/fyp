// Generated by gencpp from file dji_api/api_ctrl_data.msg
// DO NOT EDIT!


#ifndef DJI_API_MESSAGE_API_CTRL_DATA_H
#define DJI_API_MESSAGE_API_CTRL_DATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/QuaternionStamped.h>

namespace dji_api
{
template <class ContainerAllocator>
struct api_ctrl_data_
{
  typedef api_ctrl_data_<ContainerAllocator> Type;

  api_ctrl_data_()
    : ctrl_flag(0.0)
    , horiz_mode(0)
    , vert_mode(0)
    , yaw_mode(0)
    , level_frame(0)
    , torsion_frame(0)
    , ctrl_data()  {
    }
  api_ctrl_data_(const ContainerAllocator& _alloc)
    : ctrl_flag(0.0)
    , horiz_mode(0)
    , vert_mode(0)
    , yaw_mode(0)
    , level_frame(0)
    , torsion_frame(0)
    , ctrl_data(_alloc)  {
  (void)_alloc;
    }



   typedef float _ctrl_flag_type;
  _ctrl_flag_type ctrl_flag;

   typedef int8_t _horiz_mode_type;
  _horiz_mode_type horiz_mode;

   typedef int8_t _vert_mode_type;
  _vert_mode_type vert_mode;

   typedef int8_t _yaw_mode_type;
  _yaw_mode_type yaw_mode;

   typedef int8_t _level_frame_type;
  _level_frame_type level_frame;

   typedef int8_t _torsion_frame_type;
  _torsion_frame_type torsion_frame;

   typedef  ::geometry_msgs::QuaternionStamped_<ContainerAllocator>  _ctrl_data_type;
  _ctrl_data_type ctrl_data;




  typedef boost::shared_ptr< ::dji_api::api_ctrl_data_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_api::api_ctrl_data_<ContainerAllocator> const> ConstPtr;

}; // struct api_ctrl_data_

typedef ::dji_api::api_ctrl_data_<std::allocator<void> > api_ctrl_data;

typedef boost::shared_ptr< ::dji_api::api_ctrl_data > api_ctrl_dataPtr;
typedef boost::shared_ptr< ::dji_api::api_ctrl_data const> api_ctrl_dataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_api::api_ctrl_data_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_api::api_ctrl_data_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_api

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'dji_api': ['/home/bobby/fyp/catkin_ws/src/dji_api/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_api::api_ctrl_data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_api::api_ctrl_data_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_api::api_ctrl_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_api::api_ctrl_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_api::api_ctrl_data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_api::api_ctrl_data_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_api::api_ctrl_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "17a4210b32e15b6ae5ea3f62362227fa";
  }

  static const char* value(const ::dji_api::api_ctrl_data_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x17a4210b32e15b6aULL;
  static const uint64_t static_value2 = 0xe5ea3f62362227faULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_api::api_ctrl_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_api/api_ctrl_data";
  }

  static const char* value(const ::dji_api::api_ctrl_data_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_api::api_ctrl_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 ctrl_flag\n\
int8    horiz_mode\n\
int8    vert_mode\n\
int8    yaw_mode\n\
int8    level_frame\n\
int8    torsion_frame\n\
geometry_msgs/QuaternionStamped ctrl_data\n\
\n\
\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/QuaternionStamped\n\
# This represents an orientation with reference coordinate frame and timestamp.\n\
\n\
Header header\n\
Quaternion quaternion\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::dji_api::api_ctrl_data_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_api::api_ctrl_data_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ctrl_flag);
      stream.next(m.horiz_mode);
      stream.next(m.vert_mode);
      stream.next(m.yaw_mode);
      stream.next(m.level_frame);
      stream.next(m.torsion_frame);
      stream.next(m.ctrl_data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct api_ctrl_data_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_api::api_ctrl_data_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_api::api_ctrl_data_<ContainerAllocator>& v)
  {
    s << indent << "ctrl_flag: ";
    Printer<float>::stream(s, indent + "  ", v.ctrl_flag);
    s << indent << "horiz_mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.horiz_mode);
    s << indent << "vert_mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.vert_mode);
    s << indent << "yaw_mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.yaw_mode);
    s << indent << "level_frame: ";
    Printer<int8_t>::stream(s, indent + "  ", v.level_frame);
    s << indent << "torsion_frame: ";
    Printer<int8_t>::stream(s, indent + "  ", v.torsion_frame);
    s << indent << "ctrl_data: ";
    s << std::endl;
    Printer< ::geometry_msgs::QuaternionStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.ctrl_data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_API_MESSAGE_API_CTRL_DATA_H
