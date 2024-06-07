#ifndef MAP_PROVIDER_DX_H
#define MAP_PROVIDER_DX_H
#include "map_provider.h"

namespace fs
{

  class MapProviderDx final : public MapProvider
  {
  public:
    friend void MapProvider::create(const ProjectLocation projectLoc);

    bool useLockStationPush() const override { return true; };
    bool useLaneRemoveGrass() const override { return true; };

  private:
    MapProviderDx();
  };

} // namespace fs

#endif //MAP_PROVIDER_DX_H
