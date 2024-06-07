#include <common/log.h>
#include "map_provider_default.h"

fs::MapProviderDefault::MapProviderDefault()
{
  // Note: loadMap depends on the config in /opt/uto/pilot/uto_map_sdk/config/utomap_config.prototxt
  m_loadMapStatus = m_hdMap.loadMap();

  if(isMapSdkAvailable())
  {
    UERROR << "load map sdk success! map file is: " << m_hdMap.getUtoMapFilePath();
  }
  else
  {
    UERROR << "load map sdk failed! status is: " << m_loadMapStatus;
    // TODO: add diagnostic, temporary measure
    exit(0);
  }
}

Eigen::Vector3d fs::MapProviderDefault::getLtmOrigin() const
{
  assert(isMapSdkAvailable());
  const auto& ltmOrigin = m_hdMap.getLtmOrigin();
  return Eigen::Vector3d{ltmOrigin.x(), ltmOrigin.y(), 1.0f};
}