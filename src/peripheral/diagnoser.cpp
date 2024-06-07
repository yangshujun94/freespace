#include "diagnoser.h"

fs::Diagnoser& fs::Diagnoser::instance()
{
  static Diagnoser instance;
  return instance;
}

void fs::Diagnoser::updateCount()
{
  ++m_egoMotionCount;
  ++m_cameraFSCount;
  ++m_cameraFishSFWCount;
  ++m_cameraFishSFLCount;
  ++m_cameraFishSFRCount;
  ++m_cameraFishSRLCount;
  ++m_cameraFishSRRCount;
  ++m_cameraFishSRWCount;
  ++m_cameraFclCount;
  ++m_cameraFlCount;
  ++m_cameraFrCount;
  ++m_cameraRcCount;
  ++m_cameraRclCount;
  ++m_cameraRlCount;
  ++m_cameraRrCount;
  ++m_lidarCount;
  ++m_radarFDACount;
  ++m_radarFSDACount;
  ++m_radarRDACount;
  ++m_trailerCount;
  ++m_cameraTracked;
  ++m_BEVFSCount;
  ++m_VOTCount;
}

void fs::Diagnoser::updateVision()
{
  m_cameraFSCount  = 0;
  m_cameraFclCount = 0;
  m_cameraFlCount  = 0;
  m_cameraFrCount  = 0;
  m_cameraRcCount  = 0;
  m_cameraRclCount = 0;
  m_cameraRlCount  = 0;
  m_cameraRrCount  = 0;
}
