#ifndef MAP_PROVIDER_H_
#define MAP_PROVIDER_H_

#include <memory>
#include <hdmap.h>
#include "peripheral/defs.h"

namespace fs
{
  class MapProvider
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(MapProvider);

    MapProvider() = default;

    virtual ~MapProvider() = default;

    static void create(const std::string& projectLoc);

    static MapProvider& getMapProvider()
    {
      assert(nullptr != m_mapProviderPtr);
      return *m_mapProviderPtr;
    }

    bool isMapSdkAvailable() const { return m_loadMapStatus == uto::hdmap::HDMap::Result::SUCCESS; }

    virtual Eigen::Vector3d getLtmOrigin() const = 0;

    virtual std::vector<std::vector<Eigen::Vector2d>> getTunnelAreaLtm() const      = 0;
    virtual std::vector<std::vector<Eigen::Vector2d>> getWeighbridgeAreaLtm() const = 0;

    // features
    virtual bool useLockStationPush() const = 0;
    virtual bool useLaneRemoveGrass() const = 0;

  protected:
    uto::hdmap::HDMap         m_hdMap{};
    uto::hdmap::HDMap::Result m_loadMapStatus = uto::hdmap::HDMap::Result::FAIL;

  private:
    static std::unique_ptr<MapProvider> m_mapProviderPtr;
  };

} // namespace fs

#endif //MAP_PROVIDER_H_
