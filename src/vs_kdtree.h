#ifndef __VS_KDTREE_H__
#define __VS_KDTREE_H__
#include <memory>
#include <vector>

namespace vs
{

class KDTree
{
public:
    typedef std::vector<float> Data;
    typedef std::vector<Data> DataArray;

    virtual ~KDTree(){}

    virtual bool build(const DataArray& data) = 0;

    virtual int nearest(const Data& query, DataArray& res, int k, float r = 0) = 0;

    virtual int k_nearest(const Data& query, DataArray& res, int k)
    {
        return nearest(query, res, k, 0);
    }

    virtual int r_nearest(const Data& query, DataArray& res, float r)
    {
        return nearest(query, res, 0, r);
    }
};

std::shared_ptr<KDTree> createKDTree(int method = 0);

} /* namespace vs */

#endif//__VS_KDTREE_H__