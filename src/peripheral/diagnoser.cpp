#include "diagnoser.h"

fs::Diagnoser& fs::Diagnoser::instance()
{
  static Diagnoser instance;
  return instance;
}

void fs::Diagnoser::incrementTimeoutCount()
{
  ++m_egoMotionTimeoutCount;
  ++m_odTimeoutCount;
  ++m_processTimeoutCount;

  for(int i = 0; i < static_cast<int>(SensorId::MAX_SENSOR_NUM); ++i)
  {
    ++m_sensorTimeoutCountList[i];
  }
}

#if FS_CHECK(CFG_USE_FISHEYE_FS)
bool fs::Diagnoser::isFisheyeDefect() const
{
  return isSensorDefect(SensorId::FISHEYE_FCF) ||
         isSensorDefect(SensorId::FISHEYE_FLL) ||
         isSensorDefect(SensorId::FISHEYE_FRR) ||
         isSensorDefect(SensorId::FISHEYE_BCB) ||
         isSensorDefect(SensorId::FISHEYE_BLL) ||
         isSensorDefect(SensorId::FISHEYE_BRR);
}

bool fs::Diagnoser::hasFisheyeOutOfRangeError() const
{
  return hasSensorOutOfRangeError(SensorId::FISHEYE_FCF) ||
         hasSensorOutOfRangeError(SensorId::FISHEYE_FLL) ||
         hasSensorOutOfRangeError(SensorId::FISHEYE_FRR) ||
         hasSensorOutOfRangeError(SensorId::FISHEYE_BCB) ||
         hasSensorOutOfRangeError(SensorId::FISHEYE_BLL) ||
         hasSensorOutOfRangeError(SensorId::FISHEYE_BRR);
}
#endif
