#ifndef MAP_PROVIDER_TC_H
#define MAP_PROVIDER_TC_H
#include "map_provider.h"

namespace fs
{

  class MapProviderTc final : public MapProvider
  {
  public:
    friend void MapProvider::create(const ProjectLocation projectLoc);

    bool useLockStationPush() const override { return false; };
    bool useLaneRemoveGrass() const override { return false; };

  private:
    MapProviderTc();
  };
} // namespace fs

#endif //MAP_PROVIDER_TC_H
