// Generated by gencpp from file hero_chassis_controller/odom_pub.msg
// DO NOT EDIT!


#ifndef HERO_CHASSIS_CONTROLLER_MESSAGE_ODOM_PUB_H
#define HERO_CHASSIS_CONTROLLER_MESSAGE_ODOM_PUB_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hero_chassis_controller
{
template <class ContainerAllocator>
struct odom_pub_
{
  typedef odom_pub_<ContainerAllocator> Type;

  odom_pub_()
    : front_left_velocity(0)
    , front_right_velocity(0)
    , back_left_velocity(0)
    , back_right_velocity(0)  {
    }
  odom_pub_(const ContainerAllocator& _alloc)
    : front_left_velocity(0)
    , front_right_velocity(0)
    , back_left_velocity(0)
    , back_right_velocity(0)  {
  (void)_alloc;
    }



   typedef uint8_t _front_left_velocity_type;
  _front_left_velocity_type front_left_velocity;

   typedef uint8_t _front_right_velocity_type;
  _front_right_velocity_type front_right_velocity;

   typedef uint8_t _back_left_velocity_type;
  _back_left_velocity_type back_left_velocity;

   typedef uint8_t _back_right_velocity_type;
  _back_right_velocity_type back_right_velocity;





  typedef boost::shared_ptr< ::hero_chassis_controller::odom_pub_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hero_chassis_controller::odom_pub_<ContainerAllocator> const> ConstPtr;

}; // struct odom_pub_

typedef ::hero_chassis_controller::odom_pub_<std::allocator<void> > odom_pub;

typedef boost::shared_ptr< ::hero_chassis_controller::odom_pub > odom_pubPtr;
typedef boost::shared_ptr< ::hero_chassis_controller::odom_pub const> odom_pubConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hero_chassis_controller::odom_pub_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hero_chassis_controller::odom_pub_<ContainerAllocator1> & lhs, const ::hero_chassis_controller::odom_pub_<ContainerAllocator2> & rhs)
{
  return lhs.front_left_velocity == rhs.front_left_velocity &&
    lhs.front_right_velocity == rhs.front_right_velocity &&
    lhs.back_left_velocity == rhs.back_left_velocity &&
    lhs.back_right_velocity == rhs.back_right_velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hero_chassis_controller::odom_pub_<ContainerAllocator1> & lhs, const ::hero_chassis_controller::odom_pub_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hero_chassis_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hero_chassis_controller::odom_pub_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hero_chassis_controller::odom_pub_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hero_chassis_controller::odom_pub_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "04b1aa6fd936e967f094f0aa3dc0515b";
  }

  static const char* value(const ::hero_chassis_controller::odom_pub_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x04b1aa6fd936e967ULL;
  static const uint64_t static_value2 = 0xf094f0aa3dc0515bULL;
};

template<class ContainerAllocator>
struct DataType< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hero_chassis_controller/odom_pub";
  }

  static const char* value(const ::hero_chassis_controller::odom_pub_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint8 front_left_velocity\n"
"uint8 front_right_velocity\n"
"uint8 back_left_velocity\n"
"uint8 back_right_velocity\n"
;
  }

  static const char* value(const ::hero_chassis_controller::odom_pub_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.front_left_velocity);
      stream.next(m.front_right_velocity);
      stream.next(m.back_left_velocity);
      stream.next(m.back_right_velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct odom_pub_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hero_chassis_controller::odom_pub_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hero_chassis_controller::odom_pub_<ContainerAllocator>& v)
  {
    s << indent << "front_left_velocity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.front_left_velocity);
    s << indent << "front_right_velocity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.front_right_velocity);
    s << indent << "back_left_velocity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.back_left_velocity);
    s << indent << "back_right_velocity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.back_right_velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HERO_CHASSIS_CONTROLLER_MESSAGE_ODOM_PUB_H
