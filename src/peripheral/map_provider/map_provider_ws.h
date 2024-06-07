#ifndef MAP_PROVIDER_WS_H
#define MAP_PROVIDER_WS_H
#include "map_provider.h"

namespace fs
{

  class MapProviderWs final : public MapProvider
  {
  public:
    friend void MapProvider::create(const ProjectLocation projectLoc);

    bool useLockStationPush() const override { return false; };
    bool useLaneRemoveGrass() const override { return false; };

  private:
    MapProviderWs();
  };

} // namespace fs

#endif //MAP_PROVIDER_WS_H
