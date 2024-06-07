#ifndef UTO_PER_VOT_MAP_PROVIDER_LJ_H
#define UTO_PER_VOT_MAP_PROVIDER_LJ_H
#include "map_provider.h"

namespace fs
{

  class MapProviderLj final : public MapProvider
  {
  public:
    friend void MapProvider::create(const ProjectLocation projectLoc);

    bool useLockStationPush() const override { return false; };
    bool useLaneRemoveGrass() const override { return false; };

  private:
    MapProviderLj();
  };

} // namespace fs

#endif //UTO_PER_VOT_MAP_PROVIDER_LJ_H
