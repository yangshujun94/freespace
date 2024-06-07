#ifndef SWITCHER_H_
#define SWITCHER_H_

#include "peripheral/macro.h"
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

#define CFG_DEBUG_GRID FS_SWITCH(0)

#endif //SWITCHER_H_
