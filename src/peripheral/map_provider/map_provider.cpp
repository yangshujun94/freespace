#include "map_provider.h"
#include "map_provider_ws.h"
#include "map_provider_dx.h"
#include "map_provider_qz.h"
#include "map_provider_tc.h"
#include "map_provider_lj.h"

std::unique_ptr<fs::MapProvider> fs::MapProvider::m_mapProviderPtr = nullptr;

void fs::MapProvider::create(const ProjectLocation projectLoc)
{
  switch(projectLoc)
  {
  case ProjectLocation::WS:
    m_mapProviderPtr.reset(new MapProviderWs);
    break;
  case ProjectLocation::DX:
    m_mapProviderPtr.reset(new MapProviderDx);
    break;
  case ProjectLocation::QZ:
    m_mapProviderPtr.reset(new MapProviderQz);
    break;
  case ProjectLocation::TC:
    m_mapProviderPtr.reset(new MapProviderTc);
    break;
  case ProjectLocation::LJ:
    m_mapProviderPtr.reset(new MapProviderLj);
    break;
  case ProjectLocation::UNKNOWN:
    m_mapProviderPtr.reset(new MapProviderWs);
    break;
  default:
    UERROR << "unknown project, will cause SIGSEGV !!!";
    break;
  }
}

void fs::MapProvider::transformLLA2LocalXTM(double init_x, double init_y, double init_z, bool init_flag)
{
  if(!init_flag || fs::MapProvider::getMapProvider().isInit())
  {
    return;
  }
  electronic_fence_xtm_.resize(electronic_fence_lla_.size(), std::vector<Eigen::Vector2f>(4, {0.f, 0.f}));
  for(int i = 0; i < electronic_fence_lla_.size(); ++i)
  {
    for(int j = 0; j < electronic_fence_lla_[i].size(); ++j)
    {
#if FS_CHECK(CFG_ROS2)
      const auto pose = fs::Utils::convertLLA2LTM(electronic_fence_lla_[i][j].x(), electronic_fence_lla_[i][j].y(), origin_lla_);
#else
      const auto pose = fs::Utils::convertLLA2UTM(electronic_fence_lla_[i][j].x(), electronic_fence_lla_[i][j].y());
#endif
      electronic_fence_xtm_[i][j].x() = pose.x() - init_x;
      electronic_fence_xtm_[i][j].y() = pose.y() - init_y;
    }
  }
  setInit();
}

bool fs::MapProvider::isInElectricFence(const FSVec3f& twb) const
{
  for(const auto& box : electronic_fence_xtm_)
  {
    if(Utils::pointInPoly(twb.x(), twb.y(), box))
    {
      UINFO << "vehicle in electric fence";
      return true;
    }
  }
  return false;
}
