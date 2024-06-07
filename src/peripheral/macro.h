#ifndef MACRO_H_
#define MACRO_H_

#include <common/macros.h>

#define DISALLOW_COPY_MOVE_AND_ASSIGN(ClassName) \
  DISALLOW_COPY_AND_ASSIGN(ClassName);           \
  DISALLOW_MOVE_AND_ASSIGN(ClassName)

#define FS_SWITCH(X) (X
#define FS_CHECK(X)   X)

#endif //MACRO_H_
