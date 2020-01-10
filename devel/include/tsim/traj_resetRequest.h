// Generated by gencpp from file tsim/traj_resetRequest.msg
// DO NOT EDIT!


#ifndef TSIM_MESSAGE_TRAJ_RESETREQUEST_H
#define TSIM_MESSAGE_TRAJ_RESETREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tsim
{
template <class ContainerAllocator>
struct traj_resetRequest_
{
  typedef traj_resetRequest_<ContainerAllocator> Type;

  traj_resetRequest_()
    {
    }
  traj_resetRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::tsim::traj_resetRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tsim::traj_resetRequest_<ContainerAllocator> const> ConstPtr;

}; // struct traj_resetRequest_

typedef ::tsim::traj_resetRequest_<std::allocator<void> > traj_resetRequest;

typedef boost::shared_ptr< ::tsim::traj_resetRequest > traj_resetRequestPtr;
typedef boost::shared_ptr< ::tsim::traj_resetRequest const> traj_resetRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tsim::traj_resetRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tsim::traj_resetRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tsim

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'tsim': ['/home/ricojia/main-assignment-RicoJia/src/tsim/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tsim::traj_resetRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tsim::traj_resetRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tsim::traj_resetRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tsim::traj_resetRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tsim::traj_resetRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tsim::traj_resetRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tsim::traj_resetRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::tsim::traj_resetRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::tsim::traj_resetRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tsim/traj_resetRequest";
  }

  static const char* value(const ::tsim::traj_resetRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tsim::traj_resetRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::tsim::traj_resetRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tsim::traj_resetRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct traj_resetRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tsim::traj_resetRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::tsim::traj_resetRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // TSIM_MESSAGE_TRAJ_RESETREQUEST_H
