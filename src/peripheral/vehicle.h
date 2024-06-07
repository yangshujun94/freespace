#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <boost/circular_buffer.hpp>
#include <vehicle_config.pb.h>
#include "types.h"
#include "macro.h"
#include "defs.h"

namespace fs
{
  class Vehicle
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(Vehicle);

    enum class EgoStatus : uint8_t
    {
      NO_SMALLER_TIMESTAMP,
      NO_GREATER_TIMESTAMP,
      INTERPOLATED_TIMESTAMP,
      MAX_NUM
    };

    static Vehicle& instance();

    static const Vehicle& getVehicle() { return instance(); }

    void updateMechanicInfo(const uto::proto::MechanicalInfo& mechanicInfo);
    void updateVehicle(const EgoMotion& egoMotion);
    void pushEmoBuffer(const std::unique_ptr<EgoMotion>& egoMotionPtr) { m_emoBuffer.push_back(*egoMotionPtr); }
    void resetEmoBuffer() { m_emoBuffer.clear(); }

    const EMatrix3& getRotT1toT0() const { return m_rotT1toT0; }
    const EVector3& getVecT1toT0() const { return m_vecT1toT0; }
    const EVector3& getVecEgo2Wrd() const { return m_vecEgo2Wrd; }
    const EMatrix3& getRotEgo2Wrd() const { return m_rotEgo2Wrd; }
    const EMatrix3& getRotEgo2Ltm() const { return m_rotEgo2Ltm; }

    const Eigen::Vector3d&                   getVecEgo2Ltm() const { return m_vecEgo2Ltm; }
    const boost::circular_buffer<EgoMotion>& getEmoBuffer() const { return m_emoBuffer; }
    const Rect&                              getDangerZone() const { return m_dangerZone; }
    const Rect&                              getCautionZone() const { return m_cautionZone; }

    bool      isInitialized() const { return 0 != m_lastTimestampUs; }
    EVector3  transformEgo2Wrd(const EVector3& pointEgo) const { return m_rotEgo2Wrd * pointEgo + m_vecEgo2Wrd; }
    EVector3  transformWrd2Ego(const EVector3& pointWrd) const { return m_rotEgo2Wrd.transpose() * (pointWrd - m_vecEgo2Wrd); }
    EVector3  transformLtm2Ego(const Eigen::Vector3d& pointLtm) const { return m_rotEgo2Ltm.transpose() * (pointLtm - m_vecEgo2Ltm).cast<float>(); }
    EVector3  transformT1toT0Ego(const EVector3& pointT1Ego) const { return m_rotT1toT0 * pointT1Ego + m_vecT1toT0; }
    EVector3  transformT0toT1Ego(const EVector3& pointT0Ego) const { return m_rotT1toT0.transpose() * (pointT0Ego - m_vecT1toT0); }
    EVector3  transformTitoT0(const EVector3& pointTiEgo, const int64_t timestampUs) const;
    float     getDeltaPsi() const { return std::atan2(m_rotT1toT0(1, 0), m_rotT1toT0(0, 0)); }
    int64_t   getCurrentTimestamp() const { return m_currentTimestampUs; }
    bool      isMechanicUpdated() const { return m_isMechanicUpdated; }
    float     getDistRear2Ego() const { return m_distRear2Ego; }
    float     getEgoWidth() const { return m_vehicleWidth; }
    EgoStatus getEgoMotion(EgoMotion& egoMotion, const int64_t timestampUs) const;
    bool      isInGeofence(const std::vector<std::vector<Eigen::Vector2d>>& geoFencesLtm) const;

    std::tuple<EMatrix3, EVector3> calcRotAndVecTitoT0(const int64_t timestampUs) const;

    static EgoMotion interpolateEgoMotion(const int64_t timestampUs, const EgoMotion& lhsEgoMotion, const EgoMotion& rhsEgoMotion);

  private:
    Vehicle() = default;

    int64_t  m_currentTimestampUs = 0;
    int64_t  m_lastTimestampUs    = 0;
    EMatrix3 m_rotEgo2Wrd{};
    EMatrix3 m_rotT1toT0{}; ///< Rotation matrix of the instance from t-1 to t (last frame to current frame)
    EVector3 m_vecEgo2Wrd{};
    EMatrix3 m_rotEgo2WrdLast{};
    EVector3 m_vecEgo2WrdLast{};
    EVector3 m_vecT1toT0{}; ///< Ego position of last frame in current ego coordinate system

    Eigen::Vector3d m_vecEgo2Ltm{};
    EMatrix3        m_rotEgo2Ltm{};
    bool            m_isMechanicUpdated = false;
    float           m_vehicleLength     = 0.0f; ///unit: m
    float           m_vehicleWidth      = 0.0f; ///unit: m
    float           m_vehicleHeight     = 0.0f; ///unit: m
    float           m_distBumper2Ego    = 0.0f;
    float           m_distRear2Ego      = 0.0f; ///< 挂车最尾部到鞍座的相对距离

    Rect                              m_dangerZone{};
    Rect                              m_cautionZone{};
    boost::circular_buffer<EgoMotion> m_emoBuffer{EMO_BUFFER_SIZE}; // TODO: change to unique ptr
  };
} // namespace fs

#endif //VEHICLE_H_
