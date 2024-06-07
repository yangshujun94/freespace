#ifndef DIAGNOSER_H
#define DIAGNOSER_H

#include "macro.h"
#include "defs.h"

namespace fs
{
  class Diagnoser
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(Diagnoser);

    static Diagnoser& instance();

    static const Diagnoser& getDiagnoser() { return instance(); }

    //Sensor Defect
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    bool isFisheyeDefect() const;
#endif
    bool isSensorDefect(SensorId sensorId) const
    {
      return m_sensorTimeoutCountList[static_cast<int>(sensorId)] > diag::INPUT_TIMEOUT_ERROR_COUNT_THRESH;
    }
    bool isEgoMotionDefect() const { return m_egoMotionTimeoutCount > diag::INPUT_TIMEOUT_ERROR_COUNT_THRESH; }
    bool isOdDefect() const { return m_odTimeoutCount > diag::INPUT_TIMEOUT_ERROR_COUNT_THRESH; }
    void incrementTimeoutCount();
    void healSensorTimeoutCount(SensorId sensorId) { m_sensorTimeoutCountList[static_cast<int>(sensorId)] = 0; }
    void healEgoMotionTimeoutCount() { m_egoMotionTimeoutCount = 0; }
    void healOdTimeoutCount() { m_odTimeoutCount = 0; }

    //sensor timestamp out of range
#if FS_CHECK(CFG_USE_FISHEYE_FS)
    bool hasFisheyeOutOfRangeError() const;
#endif
    bool hasSensorOutOfRangeError(SensorId sensorId) const
    {
      return m_sensorOutOfRangeCountList[static_cast<int>(sensorId)] > diag::INPUT_DELAY_ERROR_COUNT_THRESH;
    }
    void incrementSensorOutOfRangeCount(SensorId sensorId) { ++m_sensorOutOfRangeCountList[static_cast<int>(sensorId)]; }
    void healSensorOutOfRangeCount(SensorId sensorId) { m_sensorOutOfRangeCountList[static_cast<int>(sensorId)] = 0; }

    // od out of range
    void healOdOutOfRangeCount() { m_odOutOfRangeCount = 0; }
    void incrementOdOutOfRangeCount() { ++m_odOutOfRangeCount; }
    bool hasOdOutOfRangeError() const { return m_odOutOfRangeCount > diag::INPUT_DELAY_ERROR_COUNT_THRESH; }

    //ego motion pose
    bool hasPoseInvalid() const { return m_hasPoseInvalid; }
    void healPoseInvalid() { m_hasPoseInvalid = false; }
    void setPoseInvalid() { m_hasPoseInvalid = true; }

    //calib error
    bool hasCalibError() const { return m_calibReadCount > diag::INPUT_TIMEOUT_ERROR_COUNT_THRESH; }
    void healCalibCount() { m_calibReadCount = 0; }
    void incrementCalibCount() { ++m_calibReadCount; }

    //map error
    bool hasMapError() const { return m_hasMapInvalid; }
    void healMapInvalid() { m_hasMapInvalid = false; }
    void setMapInvalid() { m_hasMapInvalid = true; }

    //process error
    void healProcessTimeoutCount() { m_processTimeoutCount = 0; }
    bool hasProcessError() const { return m_processTimeoutCount > diag::PROCESS_TIMEOUT_ERROR_COUNT_THRESH; }

  private:
    Diagnoser()
    {
      m_sensorTimeoutCountList.fill(0);
      m_sensorOutOfRangeCountList.fill(0);
    }

    //timeout count
    int m_egoMotionTimeoutCount = 0;
    int m_odTimeoutCount        = 0;

    std::array<int, static_cast<int>(SensorId::MAX_SENSOR_NUM)> m_sensorTimeoutCountList{};    ///< time out
    std::array<int, static_cast<int>(SensorId::MAX_SENSOR_NUM)> m_sensorOutOfRangeCountList{}; ///< out of range

    int  m_odOutOfRangeCount   = 0;     ///< od out of range count
    int  m_calibReadCount      = 0;     ///< calib count
    int  m_processTimeoutCount = 0;     ///< process count
    bool m_hasPoseInvalid      = false; ///< ego motion pose state
    bool m_hasMapInvalid       = false; ///< mapsdk state
  };

} // namespace fs

#endif
