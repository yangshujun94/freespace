#ifndef MAP_PROVIDER_LA_H
#define MAP_PROVIDER_LA_H
#include "map_provider.h"

namespace fs
{
  class MapProviderLa final : public MapProvider
  {
  public:
    friend void MapProvider::create(const std::string& projectLoc);

    Eigen::Vector3d getLtmOrigin() const override;

    std::vector<std::vector<Eigen::Vector2d>> getTunnelAreaLtm() const override;
    std::vector<std::vector<Eigen::Vector2d>> getWeighbridgeAreaLtm() const override;

    // features
    bool useLockStationPush() const override { return true; };
    bool useLaneRemoveGrass() const override { return true; };

  private:
    MapProviderLa();
  };

} // namespace fs

#endif //MAP_PROVIDER_LA_H
