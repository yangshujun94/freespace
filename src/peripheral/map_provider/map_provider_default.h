#ifndef MAP_PROVIDER_DEFAULT_H
#define MAP_PROVIDER_DEFAULT_H
#include "map_provider.h"

namespace fs
{
  class MapProviderDefault final : public MapProvider
  {
  public:
    friend void MapProvider::create(const std::string& projectLoc);

    Eigen::Vector3d getLtmOrigin() const override;

    std::vector<std::vector<Eigen::Vector2d>> getTunnelAreaLtm() const override { return {}; };
    std::vector<std::vector<Eigen::Vector2d>> getWeighbridgeAreaLtm() const override { return {}; };

    // features
    bool useLockStationPush() const override { return false; };
    bool useLaneRemoveGrass() const override { return false; };

  private:
    MapProviderDefault();
  };

} // namespace fs

#endif //MAP_PROVIDER_DEFAULT_H
