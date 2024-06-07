#ifndef PROJECT_HDT_SWITCHER_H_
#define PROJECT_HDT_SWITCHER_H_

#define CFG_USE_CF_FS               FS_SWITCH(1)
#define CFG_USE_FISHEYE_FS          FS_SWITCH(0)
#define CFG_PUB_BORDER              FS_SWITCH(1)
#define CFG_USE_WEIGHBRIDGE         FS_SWITCH(0) // 控制进入地磅区域后的不同响应;  0: 所有点不输出  1: 只输出属性为ROAD EDGE的地磅边沿点
#define CFG_USE_PASS_THROUGH_REGION FS_SWITCH(0) // 是否使用一定区域内透传的功能;  0: 不使用       1: 使用

#endif //PROJECT_HDT_SWITCHER_H_
