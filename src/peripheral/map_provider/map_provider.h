#ifndef MAP_PROVIDER_H_
#define MAP_PROVIDER_H_

#include <memory>
#include "../utils.h"
#include "peripheral/types.h"

namespace fs
{
  class MapProvider
  {
  public:
    DISALLOW_COPY_MOVE_AND_ASSIGN(MapProvider);

    MapProvider() = default;

    virtual ~MapProvider() = default;

    static void create(const ProjectLocation projectLoc);

    static MapProvider& getMapProvider()
    {
      assert(nullptr != m_mapProviderPtr);
      return *m_mapProviderPtr;
    }

    void                                                    transformLLA2LocalXTM(double init_x, double init_y, double init_z, bool init_flag);
    bool                                                    isInElectricFence(const FSVec3f& twb) const;
    inline bool                                             isInit() const { return electronic_fence_init_flag_; }
    inline const std::vector<std::vector<Eigen::Vector2f>>& getElectircFenceXTM() const { return electronic_fence_xtm_; }

    //features
    virtual bool useLockStationPush() const = 0;
    virtual bool useLaneRemoveGrass() const = 0;

  private:
    inline void setInit() { electronic_fence_init_flag_ = true; }

    static std::unique_ptr<MapProvider> m_mapProviderPtr;

  protected:
    Eigen::Vector3d                           origin_lla_{};
    std::vector<std::vector<Eigen::Vector2d>> electronic_fence_lla_{};
    std::vector<std::vector<Eigen::Vector2f>> electronic_fence_xtm_{};
    bool                                      electronic_fence_init_flag_ = false;
  };

} // namespace fs

#endif //MAP_PROVIDER_H_
