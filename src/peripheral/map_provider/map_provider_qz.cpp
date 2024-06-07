#include "map_provider_qz.h"
fs::MapProviderQz::MapProviderQz()
{
  origin_lla_ = Eigen::Vector3d{118.732660, 24.815767, 1.0}; ///(longitude ,latitude ,the scale on the central meridian)
}