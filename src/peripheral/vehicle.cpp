#include "vehicle.h"
#include <common/log.h>

fs::Vehicle& fs::Vehicle::instance()
{
  static Vehicle instance;
  return instance;
}

void fs::Vehicle::updateVehicle(const EgoMotion* ego_motion)
{
  if(ego_motion == nullptr)
  {
    return;
  }
  if(ego_motion->timestamp > cur_timestamp_ns_)
  {
    last_timestamp_ns_ = cur_timestamp_ns_;
    cur_timestamp_ns_  = ego_motion->timestamp;

    // -- save last pose
    Rwb_last_ = Rwb_;
    twb_last_ = twb_;

    // -- update current pose
    Rwb_ = Eigen::AngleAxisf(ego_motion->yaw, FSVec3f::UnitZ()) *
           Eigen::AngleAxisf(ego_motion->pitch, FSVec3f::UnitY()) *
           Eigen::AngleAxisf(ego_motion->roll, FSVec3f::UnitX());

    twb_.x() = ego_motion->translation.x();
    twb_.y() = ego_motion->translation.y();
    twb_.z() = ego_motion->translation.z();

    Rcl_ = Rwb_.transpose() * Rwb_last_;
    tcl_ = Rwb_.transpose() * (twb_last_ - twb_);

    trajectory_world_.pushBackForce(FSVec3f{ego_motion->translation.x(),
                                            ego_motion->translation.y(),
                                            ego_motion->translation.z()});
  }
  else
  {
    UERROR << "ego motion timestamp is not increasing. current timestamp: " << cur_timestamp_ns_ << ", "
           << "last timestamp: " << last_timestamp_ns_;
  }
}

void fs::Vehicle::reset()
{
  cur_timestamp_ns_  = 0;
  last_timestamp_ns_ = 0;
  Rwb_               = FSMat3x3::Identity();
  twb_               = FSVec3f::Zero();
  Rcl_               = FSMat3x3::Identity();
  tcl_               = FSVec3f::Zero();
  Rwb_last_          = FSMat3x3::Identity();
  twb_last_          = FSVec3f::Zero();
  trajectory_world_.clear();
}

#if FS_CHECK(CFG_ROS2)
void fs::Vehicle::updateMechanicInfo(const uto::proto::MechanicalInfo& mechanicInfo)
{
  if(!m_isMechanicUpdated) // only process once
  {
    m_isMechanicUpdated    = true;
    const auto vehicleType = mechanicInfo.vehicle_type();
    const int  vehicleId   = mechanicInfo.vehicle_id();
    m_vehicleLength        = mechanicInfo.vehicle_length();
    m_vehicleWidth         = mechanicInfo.vehicle_width();
    m_vehicleHeight        = mechanicInfo.vehicle_height();
    switch(vehicleType)
    {
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_HDT:
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_DT:
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_BT:
      m_distBumper2Ego = mechanicInfo.vehicle_wheel_base() + mechanicInfo.vehicle_front_overhang(); //牵引车轴距 + 牵引车前悬
      m_distRear2Ego   = mechanicInfo.trailer_rear_side_to_saddle();                                //挂车尾部到鞍座的距离
      break;
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_AIV:
      m_distBumper2Ego = 0.5f * m_vehicleLength;
      m_distRear2Ego   = -0.5f * m_vehicleLength;
      break;
    default:
      m_distBumper2Ego = 0.f;
      m_distRear2Ego   = 0.f;
      break;
    }
    UINFO << "[vehicle info][num: " << vehicleId << "]"
          << "[length: " << m_vehicleLength << "]"
          << "[width: " << m_vehicleWidth << "]";
  }
}
#endif