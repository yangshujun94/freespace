#ifndef SWITCHER_H_
#define SWITCHER_H_

#include "macro.h"
#include "project_switcher.h"

// project independent switchers, applied to all projects
#ifdef LOAD_LOG_ENABLE
#define CFG_LOAD_RECORDING FS_SWITCH(1)
#else
#define CFG_LOAD_RECORDING FS_SWITCH(0)
#endif

#ifdef VIS_ENABLE
#define CFG_VIS_ENABLE FS_SWITCH(1)
#else
#define CFG_VIS_ENABLE FS_SWITCH(0)
#endif

#define CFG_RVIZ_LOGIT_OR_MOTION        FS_SWITCH(1) // -- vis: 0-LOGIT  1-MOTION
#define CFG_USE_FOXGLOV_VIS             FS_SWITCH(0) // Online
#define CFG_DEBUG_PERCEPTION_FREESPACES FS_SWITCH(0)
#define CFG_USE_MCAP_RAW                FS_SWITCH(0) // -- Raw Vis
#define CFG_USE_MCAP_FUSION             FS_SWITCH(1) // -- Local Code Vis

#endif //SWITCHER_H_
