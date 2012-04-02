#include "polygon.h"

#include <fmutil/fm_math.h>

using namespace std;

string Polygon::to_str() const
{
    stringstream ss;
    ss << '[';
    for( unsigned i=0; i<size(); i++ )
    {
        if( i!=0 ) ss <<", ";
        ss <<'[' <<at(i).x <<',' <<at(i).y <<']';
    }
    ss <<']';
    return ss.str();
}

XmlRpc::XmlRpcValue Polygon::to_XmlRpc() const
{
    XmlRpc::XmlRpcValue xmlrpc;
    xmlrpc.setSize( size() );
    for( unsigned i=0; i<size(); i++ )
    {
        xmlrpc[i].setSize(2);
        xmlrpc[i][0] = at(i).x;
        xmlrpc[i][1] = at(i).y;
    }
    return xmlrpc;
}

void Polygon::from_XmlRpc(XmlRpc::XmlRpcValue xmlrpc)
{
    if( xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray )
        throw runtime_error("XmlRpc array wrong type");

    std::vector<cv::Point> poly;
    poly.resize( xmlrpc.size() );

    for (int32_t i = 0; i < xmlrpc.size(); ++i)
    {
        if( xmlrpc[i].getType() != XmlRpc::XmlRpcValue::TypeArray )
          throw runtime_error("XmlRpc array wrong type");

        if( xmlrpc[i].size()!=2 )
          throw runtime_error("XmlRpc array wrong size");

        poly[i].x = xmlrpc[i][0];
        poly[i].y = xmlrpc[i][1];
    }

    assign(poly.begin(), poly.end());
}

void Polygon::draw(cv::Mat & frame, cv::Scalar color) const
{
    vector< vector<cv::Point> > v;
    v.push_back( *this );
    cv::drawContours( frame, v, -1, color, 2, 8 );
    for( unsigned i=0; i<size(); i++ )
        cv::circle(frame, at(i), 5, color, 2);
}

bool Polygon::is_inside(cv::Point p) const
{
    bool inside = false;
    cv::Point p1 = at(0);
    for( unsigned i=0; i<=size(); i++ )
    {
        cv::Point p2 = at(i % size());
        if( p.y > p1.y || p.y > p2.y ) { // p.y > min(p1.y,p2.y)
            if( p.y <= p1.y || p.y <= p2.y ) { // p.y <= max(p1.y,p2.y)
                if( p.x <= p1.x || p.x <= p2.x ) { // p.x <= max(p1.x,p2.x)
                    double xinters = 0;
                    if( p1.y != p2.y )
                        xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y) + p1.x;
                    if( p1.x == p2.x || p.x <= xinters )
                        inside = ! inside;
                }
            }
        }
        p1 = p2;
    }
    return inside;
}

Polygon::iterator Polygon::find_closest_point(int x, int y)
{
    iterator closest = end();
    float dmin = 10000;
    for( iterator it = begin(); it!=end(); ++it )
    {
        float d = fmutil::distance(it->x, it->y, x, y);
        if( d<dmin && d<30 )
        {
            closest = it;
            dmin = d;
        }
    }
    return closest;
}


/** Check all lines of the polygon for the one closest to (x,y)
 *
 * Returns an iterator to the point
*/
float Polygon::dist_to_segment(int x, int y, const Polygon::iterator & it, const Polygon::iterator & jt)
{
    float aa = fmutil::angle(it->x, it->y, jt->x, jt->y);
    float ab = fmutil::angle(it->x, it->y, x, y);
    float ad = fmutil::angDist(aa,ab);
    if( ad>M_PI_2 ) return -1;

    float da = fmutil::distance(it->x, it->y, jt->x, jt->y);
    float db = fmutil::distance(it->x, it->y, x, y);
    if( db>da ) return -1;

    return db * sin(ad);
}

Polygon::iterator Polygon::add_to_countour(int x, int y)
{
    iterator closest = end();
    float dmin = 10000;
    for( iterator it = begin(); it!=end(); ++it )
    {
        iterator jt;
        if( it!=end()-1 )
            jt = it+1;
        else
            jt = begin();

        float d = dist_to_segment(x,y,it,jt);
        if( d>=0 && d<dmin )
        {
            dmin = d;
            closest = jt;
        }
    }

    if( closest==begin() ) closest = end();
    return insert(closest, cv::Point(x,y));
}

