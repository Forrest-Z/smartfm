/**
 *  @file
 *
 *  @date   Mar 26, 2011
 *  @author Josh Bialkowski (jbialk@mit.edu)
 *  @brief
 */

#ifndef MPBLOCKS_KD_TREE_HYPERRECT_HPP_
#define MPBLOCKS_KD_TREE_HYPERRECT_HPP_

#include <limits>

namespace mpblocks {
namespace  kd_tree {





template < class Traits >
void HyperRect<Traits>::makeInfinite(const int ndim)
{
    if( Traits::NDim == Eigen::Dynamic )
    {
        minExt.resize(ndim,1);
        maxExt.resize(ndim,1);
    }

    for( int i=0; i < ndim; i++)
    {
        minExt[i] = -std::numeric_limits<Format_t>::max();
        maxExt[i] = std::numeric_limits<Format_t>::max();
    }
}





template < class Traits >
typename Traits::Format_t
    HyperRect<Traits>::dist2(const Point_t& point) const
{
    Format_t dist2 = 0;
    Format_t dist2i = 0;

    for ( int i=0; i < point.rows(); i++)
    {
        if (point[i] < minExt[i])
            dist2i = minExt[i] - point[i];
        else if(point[i] > maxExt[i])
            dist2i = maxExt[i] - point[i];
        else
            dist2i = 0;

        dist2i *= dist2i;
        dist2 += dist2i;
    }

    return dist2;
}





template < class Traits >
void HyperRect<Traits>::copyTo(HyperRect<Traits>& other) const
{
    other.minExt = minExt;
    other.maxExt = maxExt;
}


template < class Traits >
typename Traits::Format_t
    HyperRect<Traits>::measure() const
{
    Format_t s = 1.0;
    for( int i=0; i < minExt.rows(); i++)
        s *= maxExt[i] - minExt[i];

    return s;
}




} // namespace kd_tree
} // namespace mpblocks

#endif
