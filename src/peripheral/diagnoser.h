#pragma once

#include "macro.h"

namespace fs
{
  static constexpr int MONITOR_COUNT_THRESH = 10; ///< Monitor count value

  class Diagnoser
  {
  public:
    DISALLOW_COPY_AND_ASSIGN(Diagnoser);

    static Diagnoser& instance();

    static const Diagnoser& getDiagnoser() { return instance(); }

    bool isEgoMotionInvalid() const { return m_egoMotionCount > MONITOR_COUNT_THRESH; }
    bool isCameraFSInvalid() const { return m_cameraFSCount > MONITOR_COUNT_THRESH; }
    bool isCameraFishInvalid() const
    {
      return m_cameraFishSFWCount > MONITOR_COUNT_THRESH ||
             m_cameraFishSFLCount > MONITOR_COUNT_THRESH ||
             m_cameraFishSFRCount > MONITOR_COUNT_THRESH ||
             m_cameraFishSRLCount > MONITOR_COUNT_THRESH ||
             m_cameraFishSRRCount > MONITOR_COUNT_THRESH ||
             m_cameraFishSRWCount > MONITOR_COUNT_THRESH;
    }
    bool isCameraFclInvalid() const { return m_cameraFclCount > MONITOR_COUNT_THRESH; }
    bool isCameraFlInvalid() const { return m_cameraFlCount > MONITOR_COUNT_THRESH; }
    bool isCameraFrInvalid() const { return m_cameraFrCount > MONITOR_COUNT_THRESH; }
    bool isCameraRcInvalid() const { return m_cameraRcCount > MONITOR_COUNT_THRESH; }
    bool isCameraRclInvalid() const { return m_cameraRclCount > MONITOR_COUNT_THRESH; }
    bool isCameraRlInvalid() const { return m_cameraRlCount > MONITOR_COUNT_THRESH; }
    bool isCameraRrInvalid() const { return m_cameraRrCount > MONITOR_COUNT_THRESH; }
    bool isLidarInvalid() const { return m_lidarCount > MONITOR_COUNT_THRESH; }
    bool isRadarFDAInvalid() const { return m_radarFDACount > MONITOR_COUNT_THRESH; }
    bool isRadarFSDAInvalid() const { return m_radarFSDACount > MONITOR_COUNT_THRESH; }
    bool isRadarRDAInvalid() const { return m_radarRDACount > MONITOR_COUNT_THRESH; }
    bool isTrailerInvalid() const { return m_trailerCount > MONITOR_COUNT_THRESH; }
    bool isCameraTrackedInvalid() const { return m_cameraTracked > MONITOR_COUNT_THRESH; }
    bool isBEVFSInvalid() const { return m_BEVFSCount > MONITOR_COUNT_THRESH; }
    bool isVOTInvalid() const { return m_VOTCount > MONITOR_COUNT_THRESH; }

    bool isEgoMotionReceived() const { return m_egoMotionCount == 0; }
    bool isCameraFcReceived() const { return m_cameraFSCount == 0; }
    bool isCameraFclReceived() const { return m_cameraFclCount == 0; }
    bool isCameraFlReceived() const { return m_cameraFlCount == 0; }
    bool isCameraFrReceived() const { return m_cameraFrCount == 0; }
    bool isCameraRcReceived() const { return m_cameraRcCount == 0; }
    bool isCameraRclReceived() const { return m_cameraRclCount == 0; }
    bool isCameraRlReceived() const { return m_cameraRlCount == 0; }
    bool isCameraRrReceived() const { return m_cameraRrCount == 0; }
    bool isLidarReceived() const { return m_lidarCount == 0; }
    bool isRadarFDAReceived() const { return m_radarFDACount == 0; }
    bool isRadarFSDAReceived() const { return m_radarFSDACount == 0; }
    bool isRadarRDAReceived() const { return m_radarRDACount == 0; }
    bool isTrailerReceived() const { return m_trailerCount == 0; }
    bool isCameraTrackedReceived() const { return m_cameraTracked == 0; }
    bool isBEVCount() const { return m_BEVFSCount == 0; }
    bool isVOTCount() const { return m_VOTCount == 0; }

    void updateCount();

    void updateVision();
    void updateEgoMotion() { m_egoMotionCount = 0; }
    void updateCameraFS() { m_cameraFSCount = 0; }
    void updateCameraFishSFW() { m_cameraFishSFWCount = 0; }
    void updateCameraFishSFL() { m_cameraFishSFLCount = 0; }
    void updateCameraFishSFR() { m_cameraFishSFRCount = 0; }
    void updateCameraFishSRL() { m_cameraFishSRLCount = 0; }
    void updateCameraFishSRR() { m_cameraFishSRRCount = 0; }
    void updateCameraFishSRW() { m_cameraFishSRWCount = 0; }
    void updateCameraFcl() { m_cameraFclCount = 0; }
    void updateCameraFl() { m_cameraFlCount = 0; }
    void updateCameraFr() { m_cameraFrCount = 0; }
    void updateCameraRc() { m_cameraRcCount = 0; }
    void updateCameraRcl() { m_cameraRclCount = 0; }
    void updateCameraRl() { m_cameraRlCount = 0; }
    void updateCameraRr() { m_cameraRrCount = 0; }
    void updateLidar() { m_lidarCount = 0; }
    void updateRadarFDA() { m_radarFDACount = 0; }
    void updateRadarFSDA() { m_radarFSDACount = 0; }
    void updateRadarRDA() { m_radarRDACount = 0; }
    void updateTrailer() { m_trailerCount = 0; }
    void updateCameraTracked() { m_cameraTracked = 0; }
    void updateBEVCount() { m_BEVFSCount = 0; }
    void updateVOTCount() { m_VOTCount = 0; }

  private:
    Diagnoser() = default;

    int m_cameraFSCount      = 0;
    int m_cameraFishSFWCount = 0;
    int m_cameraFishSFLCount = 0;
    int m_cameraFishSFRCount = 0;
    int m_cameraFishSRLCount = 0;
    int m_cameraFishSRRCount = 0;
    int m_cameraFishSRWCount = 0;
    int m_cameraFclCount     = 0;
    int m_cameraFlCount      = 0;
    int m_cameraFrCount      = 0;
    int m_cameraRcCount      = 0;
    int m_cameraRclCount     = 0;
    int m_cameraRlCount      = 0;
    int m_cameraRrCount      = 0;
    int m_lidarCount         = 0;
    int m_radarRDACount      = 0;
    int m_radarFDACount      = 0;
    int m_radarFSDACount     = 0;
    int m_trailerCount       = 0;
    int m_egoMotionCount     = 0;
    int m_cameraTracked      = 0;
    int m_BEVFSCount         = 0;
    int m_VOTCount           = 0;
  };

} // namespace fs
