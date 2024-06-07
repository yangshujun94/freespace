#include "common/log.h"
#include "map_provider_la.h"

fs::MapProviderLa::MapProviderLa()
{
#if FS_CHECK(CFG_VIS_ENABLE)
  m_loadMapStatus = m_hdMap.loadMapFromProto("/opt/uto/pilot/uto_map_sdk/data/bin/la/la.bin");
#else
  // Note: loadMap depends on the config in /opt/uto/pilot/uto_map_sdk/config/utomap_config.prototxt
  m_loadMapStatus = m_hdMap.loadMap();
#endif

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

Eigen::Vector3d fs::MapProviderLa::getLtmOrigin() const
{
  // LTM_ORIGIN = Eigen::Vector3d{115.97040640463, 32.32834639315, 1.0};
  assert(isMapSdkAvailable());
  const auto& ltmOrigin = m_hdMap.getLtmOrigin();
  return Eigen::Vector3d{ltmOrigin.x(), ltmOrigin.y(), 1.0f};
}

std::vector<std::vector<Eigen::Vector2d>> fs::MapProviderLa::getTunnelAreaLtm() const
{
  return geofence::la::TUNNEL_ELECTRONIC_FENCE;
}

std::vector<std::vector<Eigen::Vector2d>> fs::MapProviderLa::getWeighbridgeAreaLtm() const
{
  return geofence::la::WEIGHBRIDGE_ELECTRONIC_FENCE;
}