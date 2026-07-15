#pragma once
#include <cstdint>
namespace livox_ros_driver2 { namespace msg {
struct CustomPoint { float x,y,z,reflectivity; uint32_t tag; float offset_time; };
struct CustomMsg { double timebase; uint32_t point_num; uint8_t lidar_id; std::vector<CustomPoint> points; };
} }
