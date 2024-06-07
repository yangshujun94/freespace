#include "map_provider_lj.h"
fs::MapProviderLj::MapProviderLj()
{
  origin_lla_ = Eigen::Vector3d{0., 0., 0.}; ///(longitude ,latitude ,the scale on the central meridian)
}