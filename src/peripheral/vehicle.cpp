#include "vehicle.h"
#include "debug/fs_log.h"
#include "fs_math.h"
#include <common/log.h>
#include <numeric>
#include <common/math/rotation_conversion.h>

fs::Vehicle& fs::Vehicle::instance()
{
  static Vehicle instance;
  return instance;
}

void fs::Vehicle::updateVehicle(const EgoMotion& egoMotion)
{
  if(egoMotion.timestampUs > m_currentTimestampUs)
  {
    m_lastTimestampUs    = m_currentTimestampUs;
    m_currentTimestampUs = egoMotion.timestampUs;

    m_rotEgo2WrdLast = m_rotEgo2Wrd;
    m_vecEgo2WrdLast = m_vecEgo2Wrd;

    m_rotEgo2Wrd = egoMotion.quaternion.toRotationMatrix();

    m_vecEgo2Wrd = egoMotion.translation;

    m_vecEgo2Ltm = egoMotion.translationLtm;
    m_rotEgo2Ltm = egoMotion.quaternionLtm.toRotationMatrix();

    m_rotT1toT0 = m_rotEgo2Wrd.transpose() * m_rotEgo2WrdLast;
    m_vecT1toT0 = m_rotEgo2Wrd.transpose() * (m_vecEgo2WrdLast - m_vecEgo2Wrd);
  }
  else
  {
    UERROR << "ego motion timestamp is not increasing. current timestamp: " << m_currentTimestampUs << ", "
           << "ego motion timestamp: " << egoMotion.timestampUs;
  }
}

fs::Vehicle::EgoStatus fs::Vehicle::getEgoMotion(EgoMotion& egoMotion, const int64_t timestampUs) const
{
  EgoStatus status = EgoStatus::MAX_NUM;

  const auto emoRhsIter = std::find_if(m_emoBuffer.begin(), m_emoBuffer.end(), [timestampUs](const EgoMotion& emo) { return emo.timestampUs >= timestampUs; });
  const auto emoLhsIter = std::find_if(m_emoBuffer.rbegin(), m_emoBuffer.rend(), [timestampUs](const EgoMotion& emo) { return emo.timestampUs <= timestampUs; });
  if(emoLhsIter == m_emoBuffer.rend())
  {
    status = EgoStatus::NO_SMALLER_TIMESTAMP;
    UWARN << "unable to get egoMotion information smaller than this timestamp ,tf timestamp: " << timestampUs / 1000 << " ms; "
          << "ego motion timestamp: front " << m_emoBuffer.front().timestampUs / 1000 << " ms | "
          << "back " << m_emoBuffer.back().timestampUs / 1000 << " ms";
  }
  else if(emoRhsIter == m_emoBuffer.end())
  {
    status = EgoStatus::NO_GREATER_TIMESTAMP;
    UWARN << "unable to get egoMotion information greater than this timestamp ,tf timestamp: " << timestampUs / 1000 << " ms; "
          << "ego motion timestamp: front " << m_emoBuffer.front().timestampUs / 1000 << " ms | "
          << "back " << m_emoBuffer.back().timestampUs / 1000 << " ms";
  }
  else
  {
    status    = EgoStatus::INTERPOLATED_TIMESTAMP;
    egoMotion = interpolateEgoMotion(timestampUs, *emoLhsIter, *emoRhsIter);
  }

  return status;
}

fs::EgoMotion fs::Vehicle::interpolateEgoMotion(const int64_t timestampUs, const EgoMotion& lhsEgoMotion, const EgoMotion& rhsEgoMotion)
{
  // TODO: use quaternion to interpolate angles
  const float lhsRatio    = lhsEgoMotion.timestampUs == rhsEgoMotion.timestampUs ? 0.f : static_cast<float>(rhsEgoMotion.timestampUs - timestampUs) / (rhsEgoMotion.timestampUs - lhsEgoMotion.timestampUs);
  const float rhsRatio    = 1.f - lhsRatio;
  auto        interpolate = [lhsRatio, rhsRatio](const auto& lhsValue, const auto& rhsValue) { return lhsValue * lhsRatio + rhsValue * rhsRatio; };

  EgoMotion outputEgoMotion{};

  outputEgoMotion.timestampUs = timestampUs;
  outputEgoMotion.translation = interpolate(lhsEgoMotion.translation, rhsEgoMotion.translation);
  outputEgoMotion.quaternion  = lhsEgoMotion.quaternion.slerp(rhsRatio, rhsEgoMotion.quaternion);

  outputEgoMotion.translationLtm = interpolate(lhsEgoMotion.translationLtm, rhsEgoMotion.translationLtm);
  outputEgoMotion.quaternionLtm  = lhsEgoMotion.quaternionLtm.slerp(rhsRatio, rhsEgoMotion.quaternionLtm);

  return outputEgoMotion;
}

std::tuple<fs::EMatrix3, fs::EVector3> fs::Vehicle::calcRotAndVecTitoT0(const int64_t timestampUs) const
{
  EgoMotion       egoMotion;
  const EgoStatus status = getEgoMotion(egoMotion, timestampUs);
  assert(status == EgoStatus::INTERPOLATED_TIMESTAMP);

  return {m_rotEgo2Wrd.transpose() * egoMotion.quaternion.toRotationMatrix(),
          m_rotEgo2Wrd.transpose() * (egoMotion.translation - m_vecEgo2Wrd)};
}

fs::EVector3 fs::Vehicle::transformTitoT0(const fs::EVector3& pointTiEgo, const int64_t timestampUs) const
{
  const auto [rotTitoT0, vecTitoT0] = calcRotAndVecTitoT0(timestampUs);
  return rotTitoT0 * pointTiEgo + vecTitoT0;
}

bool fs::Vehicle::isInGeofence(const std::vector<std::vector<Eigen::Vector2d>>& geoFencesLtm) const
{
  bool isInFence = false;

  const Eigen::Vector2d vehLtm = Eigen::Vector2d{m_vecEgo2Ltm.x(), m_vecEgo2Ltm.y()};
  for(const auto& fenceLtm : geoFencesLtm)
  {
    if(pointInPolygon(vehLtm, fenceLtm))
    {
      isInFence = true;
      break;
    }
  }
  return isInFence;
}

void fs::Vehicle::updateMechanicInfo(const uto::proto::MechanicalInfo& mechanicInfo)
{
  if(!m_isMechanicUpdated) // only process once
  {
    m_isMechanicUpdated = true;

    const auto vehicleType = mechanicInfo.vehicle_type();
    const int  vehicleId   = mechanicInfo.vehicle_id();

    m_vehicleLength = mechanicInfo.vehicle_length();
    m_vehicleWidth  = mechanicInfo.vehicle_width();
    m_vehicleHeight = mechanicInfo.vehicle_height();

    switch(vehicleType)
    {
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_HDT:
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_DT:
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_BT:
    case uto::proto::SensorTable_VehicleType::SensorTable_VehicleType_HDT_SURROUND_VIEW:
      m_distBumper2Ego = mechanicInfo.vehicle_wheel_base() + mechanicInfo.vehicle_front_overhang(); //牵引车轴距 + 牵引车前悬
      m_distRear2Ego   = -mechanicInfo.trailer_rear_side_to_saddle();                               //挂车尾部到鞍座的距离
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

    m_dangerZone.minX = m_distRear2Ego - DANGER_ZONE_X;
    m_dangerZone.maxX = m_distBumper2Ego + DANGER_ZONE_X;
    m_dangerZone.minY = -0.5f * m_vehicleWidth - DANGER_ZONE_Y;
    m_dangerZone.maxY = 0.5f * m_vehicleWidth + DANGER_ZONE_Y;

    m_cautionZone.minX = m_distRear2Ego - CAUTION_ZONE_X;
    m_cautionZone.maxX = m_distBumper2Ego + CAUTION_ZONE_X;
    m_cautionZone.minY = -0.5f * m_vehicleWidth - CAUTION_ZONE_Y;
    m_cautionZone.maxY = 0.5f * m_vehicleWidth + CAUTION_ZONE_Y;

    LOG_DEBUG("[vehicle info][num: %d][length: %.3f][width: %.3f][front dist: %.3f]", vehicleId, m_vehicleLength, m_vehicleWidth, m_distBumper2Ego);
  }
}
