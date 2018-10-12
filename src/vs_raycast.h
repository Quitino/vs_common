#ifndef __VS_RAYCAST_H__
#define __VS_RAYCAST_H__
#include <functional>

namespace vs
{

/** \brief ray casting for 2D line voxel tracersal, using Bresenham's algorithm
    \param[in] start start point
    \param[in] end end point
    \param[in] op operator function which take TWO int input param and
                  return BOOLEAN. raycast will exit when op return false.
                  eg: [](int x, int y){printf("%d %d\n", x, y); return true;}
*/
void raycast2D(const int start[2], const int end[2],
               std::function<bool(int, int)> op);

/** \brief ray casting for 3D line voxel tracersal, using Bresenham's algorithm
    \param[in] start start point
    \param[in] end end point
    \param[in] op operator function which take THREE int input param and
                  return BOOLEAN. raycast will exit when op return false.
                  eg:   [](int x, int y, int z) {
                            printf("%d %d %d\n", x, y, z);
                            return true;
                        }
*/
void raycast3D(const int start[3], const int end[3],
               std::function<bool(int, int, int)> op);

} /* namespace vs */
#endif//__VS_RAYCAST_H__