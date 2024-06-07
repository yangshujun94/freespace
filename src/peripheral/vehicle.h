#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "ring_buffer.h"
#include "macro.h"
#include "types.h"
#include <mechanical_info.pb.h>

#if FS_CHECK(CFG_ROS2)
#include <mechanical_info.pb.h>
#endif

namespace fs
{
  class Vehicle
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(Vehicle);

    static Vehicle& instance();

    static const Vehicle& getVehicle() { return instance(); }

    /**
     * @brief compute T last to current
     * @param ego_motion  current pose
     */
    void updateVehicle(const EgoMotion* ego_motion);

    void reset();

#if FS_CHECK(CFG_ROS2)
    void updateMechanicInfo(const uto::proto::MechanicalInfo& mechanicInfo);
#endif

    const FSMat3x3& getRotT1toT0() const
    {
      return Rcl_;
    }
    const FSVec3f&  getVecT1toT0() const { return tcl_; }
    const FSVec3f&  getVecEgo2Wrd() const { return twb_; }
    const FSMat3x3& getRotEgo2Wrd() const { return Rwb_; }

    bool    isValid() const { return 0 != last_timestamp_ns_; }
    int64_t getCurrentTimestamp() const { return cur_timestamp_ns_; }
    int64_t getDeltaTimestampUs() const { return cur_timestamp_ns_ - last_timestamp_ns_; }
    float   getDeltaPsi() const { return std::atan2(Rcl_(1, 0), Rcl_(0, 0)); }
    FSVec3f transformEgo2Wrd(const FSVec3f& point_ego) const { return Rwb_ * point_ego + twb_; }
    FSVec3f transformWrd2Ego(const FSVec3f& point_wrd) const { return Rwb_.transpose() * (point_wrd - twb_); }
    FSVec3f transformT1toT0Ego(const FSVec3f& point_last_ego) const { return Rcl_ * point_last_ego + tcl_; }
    FSVec3f transformT0toT1Ego(const FSVec3f& point_cur_ego) const { return Rcl_.transpose() * (point_cur_ego - tcl_); }

    float getLength() { return m_vehicleLength; }
    float getWidth() { return m_vehicleWidth; }
    float getHalfLength() { return m_vehicleLength * 0.5; }
    float getHalfWidth() { return m_vehicleWidth * 0.5; }

    float getHeight() { return m_vehicleHeight; }

  private:
    Vehicle() = default;

    int64_t  cur_timestamp_ns_  = 0;
    int64_t  last_timestamp_ns_ = 0;
    FSMat3x3 Rwb_{}; // --  Rotation matrix  from ego(base) to world
    FSMat3x3 Rcl_{}; // --  Rotation matrix of the instance from t-1 to t (last frame to current frame)
    FSVec3f  twb_{}; // --  translation matrix  from ego(base) to world
    FSMat3x3 Rwb_last_{};
    FSVec3f  twb_last_{};
    FSVec3f  tcl_{}; // -- translation matrix  from last to current / Ego position of last frame in current ego coordinate system

    bool  m_isMechanicUpdated = false;
    float m_vehicleLength;
    float m_vehicleWidth;
    float m_vehicleHeight;
    float m_distBumper2Ego;
    float m_distRear2Ego;

    fs::RingBuffer<FSVec3f, 64> trajectory_world_;
  };
} // namespace fs

#endif //VEHICLE_H_
