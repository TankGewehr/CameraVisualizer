// Generated by gencpp from file ros_interface/Obstacle.msg
// DO NOT EDIT!


#ifndef ROS_INTERFACE_MESSAGE_OBSTACLE_H
#define ROS_INTERFACE_MESSAGE_OBSTACLE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ros_interface/Time.h>
#include <ros_interface/Time.h>
#include <ros_interface/Time.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/ImageKeyPoint.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point2D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/Point3D.h>
#include <ros_interface/BBox2D.h>
#include <ros_interface/BBox2D.h>
#include <ros_interface/SensorCalibrator.h>

namespace ros_interface
{
template <class ContainerAllocator>
struct Obstacle_
{
  typedef Obstacle_<ContainerAllocator> Type;

  Obstacle_()
    : timestamp()
    , id(0)
    , existence_prob(0.0)
    , create_time()
    , last_updated_time()
    , center_pos_vehicle()
    , center_pos_abs()
    , theta_vehicle(0.0)
    , theta_abs(0.0)
    , velocity_vehicle()
    , velocity_abs()
    , length(0.0)
    , width(0.0)
    , height(0.0)
    , image_key_points()
    , polygon_point_abs()
    , polygon_point_vehicle()
    , tracking_time(0.0)
    , type(0)
    , confidence(0.0)
    , confidence_type(0)
    , drops()
    , acceleration_vehicle()
    , acceleration_abs()
    , anchor_point_image()
    , anchor_point_vehicle()
    , anchor_point_abs()
    , bbox2d()
    , bbox2d_rear()
    , sub_type(0)
    , height_above_ground(0.0)
    , position_abs_covariance()
    , velocity_abs_covariance()
    , acceleration_abs_covariance()
    , theta_abs_covariance(0.0)
    , position_vehicle_covariance()
    , velocity_vehicle_covariance()
    , acceleration_vehicle_covariance()
    , theta_vehicle_covariance(0.0)
    , sensor_calibrator()
    , cipv_flag(0)
    , lane_position(0)
    , pihp_percentage(0.0)
    , blinker_flag(0)
    , fusion_type(0)  {
    }
  Obstacle_(const ContainerAllocator& _alloc)
    : timestamp(_alloc)
    , id(0)
    , existence_prob(0.0)
    , create_time(_alloc)
    , last_updated_time(_alloc)
    , center_pos_vehicle(_alloc)
    , center_pos_abs(_alloc)
    , theta_vehicle(0.0)
    , theta_abs(0.0)
    , velocity_vehicle(_alloc)
    , velocity_abs(_alloc)
    , length(0.0)
    , width(0.0)
    , height(0.0)
    , image_key_points(_alloc)
    , polygon_point_abs(_alloc)
    , polygon_point_vehicle(_alloc)
    , tracking_time(0.0)
    , type(0)
    , confidence(0.0)
    , confidence_type(0)
    , drops(_alloc)
    , acceleration_vehicle(_alloc)
    , acceleration_abs(_alloc)
    , anchor_point_image(_alloc)
    , anchor_point_vehicle(_alloc)
    , anchor_point_abs(_alloc)
    , bbox2d(_alloc)
    , bbox2d_rear(_alloc)
    , sub_type(0)
    , height_above_ground(0.0)
    , position_abs_covariance(_alloc)
    , velocity_abs_covariance(_alloc)
    , acceleration_abs_covariance(_alloc)
    , theta_abs_covariance(0.0)
    , position_vehicle_covariance(_alloc)
    , velocity_vehicle_covariance(_alloc)
    , acceleration_vehicle_covariance(_alloc)
    , theta_vehicle_covariance(0.0)
    , sensor_calibrator(_alloc)
    , cipv_flag(0)
    , lane_position(0)
    , pihp_percentage(0.0)
    , blinker_flag(0)
    , fusion_type(0)  {
  (void)_alloc;
    }



   typedef  ::ros_interface::Time_<ContainerAllocator>  _timestamp_type;
  _timestamp_type timestamp;

   typedef int32_t _id_type;
  _id_type id;

   typedef double _existence_prob_type;
  _existence_prob_type existence_prob;

   typedef  ::ros_interface::Time_<ContainerAllocator>  _create_time_type;
  _create_time_type create_time;

   typedef  ::ros_interface::Time_<ContainerAllocator>  _last_updated_time_type;
  _last_updated_time_type last_updated_time;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _center_pos_vehicle_type;
  _center_pos_vehicle_type center_pos_vehicle;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _center_pos_abs_type;
  _center_pos_abs_type center_pos_abs;

   typedef double _theta_vehicle_type;
  _theta_vehicle_type theta_vehicle;

   typedef double _theta_abs_type;
  _theta_abs_type theta_abs;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _velocity_vehicle_type;
  _velocity_vehicle_type velocity_vehicle;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _velocity_abs_type;
  _velocity_abs_type velocity_abs;

   typedef double _length_type;
  _length_type length;

   typedef double _width_type;
  _width_type width;

   typedef double _height_type;
  _height_type height;

   typedef std::vector< ::ros_interface::ImageKeyPoint_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ros_interface::ImageKeyPoint_<ContainerAllocator> >> _image_key_points_type;
  _image_key_points_type image_key_points;

   typedef std::vector< ::ros_interface::Point3D_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ros_interface::Point3D_<ContainerAllocator> >> _polygon_point_abs_type;
  _polygon_point_abs_type polygon_point_abs;

   typedef std::vector< ::ros_interface::Point3D_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ros_interface::Point3D_<ContainerAllocator> >> _polygon_point_vehicle_type;
  _polygon_point_vehicle_type polygon_point_vehicle;

   typedef double _tracking_time_type;
  _tracking_time_type tracking_time;

   typedef int32_t _type_type;
  _type_type type;

   typedef double _confidence_type;
  _confidence_type confidence;

   typedef int32_t _confidence_type_type;
  _confidence_type_type confidence_type;

   typedef std::vector< ::ros_interface::Point3D_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ros_interface::Point3D_<ContainerAllocator> >> _drops_type;
  _drops_type drops;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _acceleration_vehicle_type;
  _acceleration_vehicle_type acceleration_vehicle;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _acceleration_abs_type;
  _acceleration_abs_type acceleration_abs;

   typedef  ::ros_interface::Point2D_<ContainerAllocator>  _anchor_point_image_type;
  _anchor_point_image_type anchor_point_image;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _anchor_point_vehicle_type;
  _anchor_point_vehicle_type anchor_point_vehicle;

   typedef  ::ros_interface::Point3D_<ContainerAllocator>  _anchor_point_abs_type;
  _anchor_point_abs_type anchor_point_abs;

   typedef  ::ros_interface::BBox2D_<ContainerAllocator>  _bbox2d_type;
  _bbox2d_type bbox2d;

   typedef  ::ros_interface::BBox2D_<ContainerAllocator>  _bbox2d_rear_type;
  _bbox2d_rear_type bbox2d_rear;

   typedef int32_t _sub_type_type;
  _sub_type_type sub_type;

   typedef double _height_above_ground_type;
  _height_above_ground_type height_above_ground;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _position_abs_covariance_type;
  _position_abs_covariance_type position_abs_covariance;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _velocity_abs_covariance_type;
  _velocity_abs_covariance_type velocity_abs_covariance;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _acceleration_abs_covariance_type;
  _acceleration_abs_covariance_type acceleration_abs_covariance;

   typedef double _theta_abs_covariance_type;
  _theta_abs_covariance_type theta_abs_covariance;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _position_vehicle_covariance_type;
  _position_vehicle_covariance_type position_vehicle_covariance;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _velocity_vehicle_covariance_type;
  _velocity_vehicle_covariance_type velocity_vehicle_covariance;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _acceleration_vehicle_covariance_type;
  _acceleration_vehicle_covariance_type acceleration_vehicle_covariance;

   typedef double _theta_vehicle_covariance_type;
  _theta_vehicle_covariance_type theta_vehicle_covariance;

   typedef  ::ros_interface::SensorCalibrator_<ContainerAllocator>  _sensor_calibrator_type;
  _sensor_calibrator_type sensor_calibrator;

   typedef uint8_t _cipv_flag_type;
  _cipv_flag_type cipv_flag;

   typedef int32_t _lane_position_type;
  _lane_position_type lane_position;

   typedef double _pihp_percentage_type;
  _pihp_percentage_type pihp_percentage;

   typedef int32_t _blinker_flag_type;
  _blinker_flag_type blinker_flag;

   typedef int32_t _fusion_type_type;
  _fusion_type_type fusion_type;





  typedef boost::shared_ptr< ::ros_interface::Obstacle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_interface::Obstacle_<ContainerAllocator> const> ConstPtr;

}; // struct Obstacle_

typedef ::ros_interface::Obstacle_<std::allocator<void> > Obstacle;

typedef boost::shared_ptr< ::ros_interface::Obstacle > ObstaclePtr;
typedef boost::shared_ptr< ::ros_interface::Obstacle const> ObstacleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_interface::Obstacle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_interface::Obstacle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_interface::Obstacle_<ContainerAllocator1> & lhs, const ::ros_interface::Obstacle_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.id == rhs.id &&
    lhs.existence_prob == rhs.existence_prob &&
    lhs.create_time == rhs.create_time &&
    lhs.last_updated_time == rhs.last_updated_time &&
    lhs.center_pos_vehicle == rhs.center_pos_vehicle &&
    lhs.center_pos_abs == rhs.center_pos_abs &&
    lhs.theta_vehicle == rhs.theta_vehicle &&
    lhs.theta_abs == rhs.theta_abs &&
    lhs.velocity_vehicle == rhs.velocity_vehicle &&
    lhs.velocity_abs == rhs.velocity_abs &&
    lhs.length == rhs.length &&
    lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.image_key_points == rhs.image_key_points &&
    lhs.polygon_point_abs == rhs.polygon_point_abs &&
    lhs.polygon_point_vehicle == rhs.polygon_point_vehicle &&
    lhs.tracking_time == rhs.tracking_time &&
    lhs.type == rhs.type &&
    lhs.confidence == rhs.confidence &&
    lhs.confidence_type == rhs.confidence_type &&
    lhs.drops == rhs.drops &&
    lhs.acceleration_vehicle == rhs.acceleration_vehicle &&
    lhs.acceleration_abs == rhs.acceleration_abs &&
    lhs.anchor_point_image == rhs.anchor_point_image &&
    lhs.anchor_point_vehicle == rhs.anchor_point_vehicle &&
    lhs.anchor_point_abs == rhs.anchor_point_abs &&
    lhs.bbox2d == rhs.bbox2d &&
    lhs.bbox2d_rear == rhs.bbox2d_rear &&
    lhs.sub_type == rhs.sub_type &&
    lhs.height_above_ground == rhs.height_above_ground &&
    lhs.position_abs_covariance == rhs.position_abs_covariance &&
    lhs.velocity_abs_covariance == rhs.velocity_abs_covariance &&
    lhs.acceleration_abs_covariance == rhs.acceleration_abs_covariance &&
    lhs.theta_abs_covariance == rhs.theta_abs_covariance &&
    lhs.position_vehicle_covariance == rhs.position_vehicle_covariance &&
    lhs.velocity_vehicle_covariance == rhs.velocity_vehicle_covariance &&
    lhs.acceleration_vehicle_covariance == rhs.acceleration_vehicle_covariance &&
    lhs.theta_vehicle_covariance == rhs.theta_vehicle_covariance &&
    lhs.sensor_calibrator == rhs.sensor_calibrator &&
    lhs.cipv_flag == rhs.cipv_flag &&
    lhs.lane_position == rhs.lane_position &&
    lhs.pihp_percentage == rhs.pihp_percentage &&
    lhs.blinker_flag == rhs.blinker_flag &&
    lhs.fusion_type == rhs.fusion_type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_interface::Obstacle_<ContainerAllocator1> & lhs, const ::ros_interface::Obstacle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ros_interface::Obstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_interface::Obstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_interface::Obstacle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_interface::Obstacle_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_interface::Obstacle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_interface::Obstacle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_interface::Obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "540c5953bd92bdf84eafae61ed094918";
  }

  static const char* value(const ::ros_interface::Obstacle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x540c5953bd92bdf8ULL;
  static const uint64_t static_value2 = 0x4eafae61ed094918ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_interface::Obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_interface/Obstacle";
  }

  static const char* value(const ::ros_interface::Obstacle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_interface::Obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Time   timestamp     # 时间戳\n"
"int32   id     # 障碍物id\n"
"float64   existence_prob     # 障碍物存在的概率\n"
"Time   create_time     # 障碍物被识别的时间戳\n"
"Time   last_updated_time     # 障碍物上一次更新的时间\n"
"Point3D   center_pos_vehicle     # 车辆坐标系下障碍物的中心位置\n"
"Point3D   center_pos_abs     # 世界坐标系下障碍物的中心位置\n"
"float64   theta_vehicle     # 车辆坐标系下障碍物的朝向\n"
"float64   theta_abs     # 世界坐标系下障碍物的朝向\n"
"Point3D   velocity_vehicle     # 车辆坐标系下障碍物的速度\n"
"Point3D   velocity_abs     # 世界坐标系下障碍物的速度\n"
"float64   length     # 障碍物长度\n"
"float64   width     # 障碍物宽度\n"
"float64   height     # 障碍物高度\n"
"ImageKeyPoint[] image_key_points # 图像坐标系下障碍物多边形\n"
"Point3D[] polygon_point_abs # 世界坐标系下障碍物多边形\n"
"Point3D[] polygon_point_vehicle # 车辆坐标系下障碍物多边形\n"
"float64   tracking_time     # 障碍物被追踪的时间\n"
"int32   type     # 障碍物类别： UNKNOWN = 0; UNKNOWN_MOVABLE = 1; UNKNOWN_UNMOVABLE = 2; PEDESTRIAN = 3; BICYCLE = 4; VEHICLE = 5;\n"
"float64   confidence     # 障碍物类别置信度\n"
"int32   confidence_type     # 障碍物置信度类别： CONFIDENCE_UNKNOWN = 0; CONFIDENCE_CNN = 1; CONFIDENCE_RADAR = 2;\n"
"Point3D[] drops # 障碍物轨迹点\n"
"Point3D   acceleration_vehicle     # 车辆坐标系下障碍物的加速度\n"
"Point3D   acceleration_abs     # 世界坐标系下障碍物的加速度\n"
"Point2D   anchor_point_image     # 障碍物尾框中心点(图像坐标系)\n"
"Point3D   anchor_point_vehicle     # 障碍物尾框中心点(车辆坐标系)\n"
"Point3D   anchor_point_abs     # 障碍物尾框中心点(世界坐标系)\n"
"BBox2D   bbox2d     # 障碍物图像框\n"
"BBox2D   bbox2d_rear     # 障碍物图像尾框\n"
"int32   sub_type     # 障碍物类别： ST_UNKNOWN = 0; ST_UNKNOWN_MOVABLE = 1; ST_UNKNOWN_UNMOVABLE = 2; ST_CAR = 3; ST_VAN = 4; ST_TRUCK = 5; ST_BUS = 6; ST_CYCLIST = 7; ST_MOTORCYCLIST = 8; ST_TRICYCLIST = 9; ST_PEDESTRIAN = 10; ST_TRAFFICCONE = 11; ST_PILLAR = 12; ST_SPEED_BUMP = 13;\n"
"float64   height_above_ground     # 障碍物近地点到地面的高度\n"
"float64[] position_abs_covariance # 世界坐标系下障碍物中心位置的协方差矩阵\n"
"float64[] velocity_abs_covariance # 世界坐标系下障碍物速度的协方差矩阵\n"
"float64[] acceleration_abs_covariance # 世界坐标系下障碍物加速度的协方差矩阵\n"
"float64   theta_abs_covariance     # 世界坐标系下障碍物朝向的协方差矩阵\n"
"float64[] position_vehicle_covariance # 车辆坐标系下障碍物中心位置的协方差矩阵\n"
"float64[] velocity_vehicle_covariance # 车辆坐标系下障碍物速度的协方差矩阵\n"
"float64[] acceleration_vehicle_covariance # 车辆坐标系下障碍物加速度的协方差矩阵\n"
"float64   theta_vehicle_covariance     # 车辆坐标系下障碍物朝向的协方差矩阵\n"
"SensorCalibrator   sensor_calibrator     # 传感器标定参数\n"
"uint8   cipv_flag     # 障碍物状态标志（0-CIPV 1-CIPS 2-LPIHP 3-RPIHP 4-NONE）\n"
"int32   lane_position     # 车道线位置 -2-NEXT_LEFT_LANE -1-LEFT_LANE 0-EGO_LANE 1-RIGHT_LANE 2-NEXT_RIGHT_LANE 3-OTHERS 4-UNKNOWN\n"
"float64   pihp_percentage     # 临车道车辆压线比例\n"
"int32   blinker_flag     # 障碍物车辆信号灯状态： 0-OFF 1-LEFT_TURN_VISIBLE 2-LEFT_TURN_ON 3-RIGHT_TURN_VISIBLE 4-RIGHT_TURN_ON 5-BRAKE_VISIBLE 6-BRAKE_ON 7-UNKNOWN\n"
"int32   fusion_type     # 融合障碍物类型 0-CAMERA 1-RADAR 2-LIDAR 3-ULTRASONIC 4-FUSED 5-UNKNOWN\n"
"\n"
"================================================================================\n"
"MSG: ros_interface/Time\n"
"uint32   sec     # 秒\n"
"uint32   nsec     # 纳秒\n"
"\n"
"================================================================================\n"
"MSG: ros_interface/Point3D\n"
"float64   x     # 位置坐标x\n"
"float64   y     # 位置坐标y\n"
"float64   z     # 位置坐标z\n"
"\n"
"================================================================================\n"
"MSG: ros_interface/ImageKeyPoint\n"
"float64   x     # 车位图像关键点x坐标\n"
"float64   y     # 车位图像关键点y坐标\n"
"float64   confidence     # 置信度\n"
"\n"
"================================================================================\n"
"MSG: ros_interface/Point2D\n"
"float64   x     # 位置坐标x\n"
"float64   y     # 位置坐标y\n"
"\n"
"================================================================================\n"
"MSG: ros_interface/BBox2D\n"
"int16   xmin     # 图像框左上角的x坐标\n"
"int16   ymin     # 图像框左上角的y坐标\n"
"int16   xmax     # 图像框右下角的x坐标\n"
"int16   ymax     # 图像框右下角的y坐标\n"
"\n"
"================================================================================\n"
"MSG: ros_interface/SensorCalibrator\n"
"Point3D   pose     # 传感器安装位置(相对于后轴中心点)\n"
"Point3D   angle     # 传感器安装角度(车体坐标系)\n"
;
  }

  static const char* value(const ::ros_interface::Obstacle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_interface::Obstacle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.id);
      stream.next(m.existence_prob);
      stream.next(m.create_time);
      stream.next(m.last_updated_time);
      stream.next(m.center_pos_vehicle);
      stream.next(m.center_pos_abs);
      stream.next(m.theta_vehicle);
      stream.next(m.theta_abs);
      stream.next(m.velocity_vehicle);
      stream.next(m.velocity_abs);
      stream.next(m.length);
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.image_key_points);
      stream.next(m.polygon_point_abs);
      stream.next(m.polygon_point_vehicle);
      stream.next(m.tracking_time);
      stream.next(m.type);
      stream.next(m.confidence);
      stream.next(m.confidence_type);
      stream.next(m.drops);
      stream.next(m.acceleration_vehicle);
      stream.next(m.acceleration_abs);
      stream.next(m.anchor_point_image);
      stream.next(m.anchor_point_vehicle);
      stream.next(m.anchor_point_abs);
      stream.next(m.bbox2d);
      stream.next(m.bbox2d_rear);
      stream.next(m.sub_type);
      stream.next(m.height_above_ground);
      stream.next(m.position_abs_covariance);
      stream.next(m.velocity_abs_covariance);
      stream.next(m.acceleration_abs_covariance);
      stream.next(m.theta_abs_covariance);
      stream.next(m.position_vehicle_covariance);
      stream.next(m.velocity_vehicle_covariance);
      stream.next(m.acceleration_vehicle_covariance);
      stream.next(m.theta_vehicle_covariance);
      stream.next(m.sensor_calibrator);
      stream.next(m.cipv_flag);
      stream.next(m.lane_position);
      stream.next(m.pihp_percentage);
      stream.next(m.blinker_flag);
      stream.next(m.fusion_type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Obstacle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_interface::Obstacle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_interface::Obstacle_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    s << std::endl;
    Printer< ::ros_interface::Time_<ContainerAllocator> >::stream(s, indent + "  ", v.timestamp);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "existence_prob: ";
    Printer<double>::stream(s, indent + "  ", v.existence_prob);
    s << indent << "create_time: ";
    s << std::endl;
    Printer< ::ros_interface::Time_<ContainerAllocator> >::stream(s, indent + "  ", v.create_time);
    s << indent << "last_updated_time: ";
    s << std::endl;
    Printer< ::ros_interface::Time_<ContainerAllocator> >::stream(s, indent + "  ", v.last_updated_time);
    s << indent << "center_pos_vehicle: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.center_pos_vehicle);
    s << indent << "center_pos_abs: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.center_pos_abs);
    s << indent << "theta_vehicle: ";
    Printer<double>::stream(s, indent + "  ", v.theta_vehicle);
    s << indent << "theta_abs: ";
    Printer<double>::stream(s, indent + "  ", v.theta_abs);
    s << indent << "velocity_vehicle: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity_vehicle);
    s << indent << "velocity_abs: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity_abs);
    s << indent << "length: ";
    Printer<double>::stream(s, indent + "  ", v.length);
    s << indent << "width: ";
    Printer<double>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
    s << indent << "image_key_points[]" << std::endl;
    for (size_t i = 0; i < v.image_key_points.size(); ++i)
    {
      s << indent << "  image_key_points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ros_interface::ImageKeyPoint_<ContainerAllocator> >::stream(s, indent + "    ", v.image_key_points[i]);
    }
    s << indent << "polygon_point_abs[]" << std::endl;
    for (size_t i = 0; i < v.polygon_point_abs.size(); ++i)
    {
      s << indent << "  polygon_point_abs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "    ", v.polygon_point_abs[i]);
    }
    s << indent << "polygon_point_vehicle[]" << std::endl;
    for (size_t i = 0; i < v.polygon_point_vehicle.size(); ++i)
    {
      s << indent << "  polygon_point_vehicle[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "    ", v.polygon_point_vehicle[i]);
    }
    s << indent << "tracking_time: ";
    Printer<double>::stream(s, indent + "  ", v.tracking_time);
    s << indent << "type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.type);
    s << indent << "confidence: ";
    Printer<double>::stream(s, indent + "  ", v.confidence);
    s << indent << "confidence_type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.confidence_type);
    s << indent << "drops[]" << std::endl;
    for (size_t i = 0; i < v.drops.size(); ++i)
    {
      s << indent << "  drops[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "    ", v.drops[i]);
    }
    s << indent << "acceleration_vehicle: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration_vehicle);
    s << indent << "acceleration_abs: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration_abs);
    s << indent << "anchor_point_image: ";
    s << std::endl;
    Printer< ::ros_interface::Point2D_<ContainerAllocator> >::stream(s, indent + "  ", v.anchor_point_image);
    s << indent << "anchor_point_vehicle: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.anchor_point_vehicle);
    s << indent << "anchor_point_abs: ";
    s << std::endl;
    Printer< ::ros_interface::Point3D_<ContainerAllocator> >::stream(s, indent + "  ", v.anchor_point_abs);
    s << indent << "bbox2d: ";
    s << std::endl;
    Printer< ::ros_interface::BBox2D_<ContainerAllocator> >::stream(s, indent + "  ", v.bbox2d);
    s << indent << "bbox2d_rear: ";
    s << std::endl;
    Printer< ::ros_interface::BBox2D_<ContainerAllocator> >::stream(s, indent + "  ", v.bbox2d_rear);
    s << indent << "sub_type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.sub_type);
    s << indent << "height_above_ground: ";
    Printer<double>::stream(s, indent + "  ", v.height_above_ground);
    s << indent << "position_abs_covariance[]" << std::endl;
    for (size_t i = 0; i < v.position_abs_covariance.size(); ++i)
    {
      s << indent << "  position_abs_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position_abs_covariance[i]);
    }
    s << indent << "velocity_abs_covariance[]" << std::endl;
    for (size_t i = 0; i < v.velocity_abs_covariance.size(); ++i)
    {
      s << indent << "  velocity_abs_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocity_abs_covariance[i]);
    }
    s << indent << "acceleration_abs_covariance[]" << std::endl;
    for (size_t i = 0; i < v.acceleration_abs_covariance.size(); ++i)
    {
      s << indent << "  acceleration_abs_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.acceleration_abs_covariance[i]);
    }
    s << indent << "theta_abs_covariance: ";
    Printer<double>::stream(s, indent + "  ", v.theta_abs_covariance);
    s << indent << "position_vehicle_covariance[]" << std::endl;
    for (size_t i = 0; i < v.position_vehicle_covariance.size(); ++i)
    {
      s << indent << "  position_vehicle_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position_vehicle_covariance[i]);
    }
    s << indent << "velocity_vehicle_covariance[]" << std::endl;
    for (size_t i = 0; i < v.velocity_vehicle_covariance.size(); ++i)
    {
      s << indent << "  velocity_vehicle_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.velocity_vehicle_covariance[i]);
    }
    s << indent << "acceleration_vehicle_covariance[]" << std::endl;
    for (size_t i = 0; i < v.acceleration_vehicle_covariance.size(); ++i)
    {
      s << indent << "  acceleration_vehicle_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.acceleration_vehicle_covariance[i]);
    }
    s << indent << "theta_vehicle_covariance: ";
    Printer<double>::stream(s, indent + "  ", v.theta_vehicle_covariance);
    s << indent << "sensor_calibrator: ";
    s << std::endl;
    Printer< ::ros_interface::SensorCalibrator_<ContainerAllocator> >::stream(s, indent + "  ", v.sensor_calibrator);
    s << indent << "cipv_flag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cipv_flag);
    s << indent << "lane_position: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lane_position);
    s << indent << "pihp_percentage: ";
    Printer<double>::stream(s, indent + "  ", v.pihp_percentage);
    s << indent << "blinker_flag: ";
    Printer<int32_t>::stream(s, indent + "  ", v.blinker_flag);
    s << indent << "fusion_type: ";
    Printer<int32_t>::stream(s, indent + "  ", v.fusion_type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_INTERFACE_MESSAGE_OBSTACLE_H
