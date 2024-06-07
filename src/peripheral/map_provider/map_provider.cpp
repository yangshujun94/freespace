#include <common/log.h>
#include "map_provider.h"
#include "map_provider_default.h"
#include "map_provider_la.h"

std::unique_ptr<fs::MapProvider> fs::MapProvider::m_mapProviderPtr = nullptr;

void fs::MapProvider::create(const std::string& projectLoc)
{
  UERROR << "create map provider: " << projectLoc;

  if(projectLoc == "LA")
  {
    m_mapProviderPtr.reset(new MapProviderLa);
  }
  else
  {
    UERROR << "No matching projectLoc, use default!";
    m_mapProviderPtr.reset(new MapProviderDefault);
  }
}