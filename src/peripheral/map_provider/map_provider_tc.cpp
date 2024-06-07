#include "map_provider_tc.h"
fs::MapProviderTc::MapProviderTc()
{
  origin_lla_ = Eigen::Vector3d{0.0, 0.0, 0.0}; ///(longitude ,latitude ,the scale on the central meridian)
}