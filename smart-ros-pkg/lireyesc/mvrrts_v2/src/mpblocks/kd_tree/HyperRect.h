/**
 *  @file
 *
 *  @date   Mar 26, 2011
 *  @author Josh Bialkowski (jbialk@mit.edu)
 *  @brief
 */

#ifndef MPBLOCKS_KD_TREE_HYPERRECT_H_
#define MPBLOCKS_KD_TREE_HYPERRECT_H_


namespace mpblocks {
namespace  kd_tree {


/// an NDim dimensional hyperrectangle, represented as a min and max extent
/**
 *  implemented by storing the minimum and maximum extent of the hyper-rectangle
 *  in all dimensions
 */
template <class Traits>
struct HyperRect
{
    typedef typename Traits::Format_t    Format_t;
    typedef Eigen::Matrix<Format_t,Traits::NDim,1>  Vector_t;
    typedef Vector_t                                Point_t;

    Point_t     minExt;     ///< minimum extent of hyper-rectangle
    Point_t     maxExt;     ///< maximum extent of hyper-rectangle

    /// reset the hyperrectangle to be infinite in all extents
    void makeInfinite(const int ndim=Traits::NDim);

    /// find the nearest point in the hyper-rectangle to the query point and
    /// return it's distance (squared)
    Format_t dist2(const Point_t& point) const;

    /// copy the contents of this hyper rectangle to another
    void copyTo(HyperRect& other) const;

    /// return the measure of the hypercube
    Format_t measure() const;
};



} // namespace kd_tree
} // namespace mpblocks

#endif
