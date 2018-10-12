#ifndef __VS_MAPPING_H__
#define __VS_MAPPING_H__
#include <stdint.h>
#include <memory.h>
#include <cmath>
#include <algorithm>
#include <functional>

namespace vs
{
/** \brief 2D/3D grid mapping
    ValType: grid map value type
    D: dimension, only 2 or 3 support
*/
template<class ValType, int D>
class GridMapping
{
public:
    enum CheckRes
    {
        OCCUPY = 1,
        FREE = 0,
        UNKNOWN = -1,
    };

    // constructor
    GridMapping(int len_exp = 8, float res = 1.0f,
                ValType val_max = 1, ValType val_inc = 1,
                ValType val_dec = 1, ValType val_thres = 1);

    // deconstructor
    ~GridMapping();

    // update map with a frame of point cloud
    // NOTE: pts are in world frame rather than body frame
    void update(const float origin[D], const float pts[][D], int N);

    int checkPoint(const float pos[D]);

    int checkPoint(const float p[D], float r);

    int checkLine(const float p1[D], const float p2[D]);

    int checkLine(const float p1[D], const float p2[D], float r);

    void clear();

    int dim() {return D;}

    float resolution() {return m_res;}

    float* origin() {return m_origin;}

    int* originIdx() {return m_origin_idx;}

    int size() {return m_size;}

    int len() {return m_len;}

    ValType* map() {return m_map;}

    bool occupyVal(ValType v) {return v >= m_thres;}

    int index(int idx[D])
    {
        int res = idx[0];
        for(int i = 1; i < D; i++)
        {
            res = (res << E) | idx[i];
        }
        return res;
    }

    void pos2idx(const float pos[D], int idx[D])
    {
        for(int i = 0; i < D; i++) {idx[i] = p2i(pos[i]);}
    }

    void idx2pos(const int idx[D], float pos[D])
    {
        for(int i = 0; i < D; i++)
        {
            int k = idx[i] - m_origin_idx[i];
            while(k > UPPER) k -= K;
            while(k < LOWER) k += K;
            pos[i] = (k + 0.5) * m_res + m_origin_base[i];
        }
    }

    ValType& at(const int idx[D])
    {
        int in[D];
        for(int i = 0; i < D; i++) in[i] = idx[i] & M;
        return m_map[index(in)];
    }

    ValType& pos(const float p[D])
    {
        int idx[D];
        pos2idx(p, idx);
        return at(idx);
    }

    bool inMap(const int idx[D])
    {
        for(int i = 0; i < D; i++)
        {
            int delta = idx[i] - m_origin_idx[i];
            if(delta > UPPER || delta < LOWER) return false;
        }
        return true;
    }

    bool inMap(const float pos[D])
    {
        int idx[D];
        pos2idx(pos, idx);
        return inMap(idx);
    }

    int check(const int idx[D])
    {
        if(!inMap(idx)) return UNKNOWN;
        else return occupyVal(at(idx)) ? OCCUPY : FREE;
    }

    const int E,K,M;

private:
    const int UPPER,LOWER;
    const ValType m_max, m_inc, m_dec, m_thres;
    const int m_len, m_size, m_pos_size, m_idx_size;
    int m_dim_size[D];
    ValType* m_map; //gridmap rolling buffer
    float m_origin[D];      //origin
    float m_res;            //resolution
    int m_origin_idx[D];    //origin index
    float m_origin_base[D]; // base = int(origin/res) * res

    int m_check_res;
    std::function<bool(int[D])> m_f_update_free;
    std::function<bool(int[D])> m_f_check;
    std::function<bool(int, int, int)> m_f_update_free_3d;
    std::function<bool(int, int, int)> m_f_check_3d;
    std::function<bool(int, int)> m_f_update_free_2d;
    std::function<bool(int, int)> m_f_check_2d;

    int p2i(float p) {return int(std::floor(p / m_res));}

    void updateRay(const float p[D]);

    void moveMap(const float new_origin[3]);

    void clearRow(int r, int dim);

    int checkLineTravel(const float p1[D], const float p2[D],
                        const float step[D], int n);
};

typedef GridMapping<uint8_t, 2> GridMapping2D;
typedef GridMapping<uint8_t, 3> GridMapping3D;

} /* namespace vs */
#endif//__VS_MAPPING_H__