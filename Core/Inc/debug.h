#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdio.h>

#define ENABLE_PRINTF   0

#if !ENABLE_PRINTF
    // 若未定义ENABLE_PRINTF，则将printf定义为空操作
    #define printf(fmt, ...) ((void)0)
#endif




#endif

