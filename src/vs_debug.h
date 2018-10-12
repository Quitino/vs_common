#ifndef __VS_DEBUG_H__
#define __VS_DEBUG_H__
#include <stdlib.h>

// #define DEBUG 1

#ifdef DEBUG
    #include <iostream>
    #include "vs_tictoc.h"
    #define VS_WATCH(a) do {std::cout << #a << ":" << a <<std::endl;} while(0)
    #define VS_TRACE() do {printf("Debug: %s() %d\n", __func__, __LINE__);} while(0)
    #define VS_TIC(a) vs::tic(a)
    #define VS_TICTOC(a) vs::tictoc(a)
#else //DEBUG
    #define VS_WATCH(x)
    #define VS_TICTOC(a)
    #define VS_TRACE()
    #define VS_TIC(a)
#endif //DEBUG

namespace vs
{

void openSegFaultDebug(const char* log_file = NULL);

} /* namespace vs */

#endif //__VS_DEBUG_H__