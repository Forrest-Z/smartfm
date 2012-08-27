#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include "SvgBoundary.h"

using namespace std;

const unsigned VERBOSITY = 0;

SvgBoundary::SvgBoundary()
{

}

SvgBoundary::SvgBoundary(string pFilename, double res)
{
    loadFile(pFilename, res);
}

#include <wordexp.h>
string tilde_expand(string path)
{
    wordexp_t p;
    wordexp(path.c_str(), &p, 0);
    char **w = p.we_wordv;
    assert(p.we_wordc==1);
    string res = w[0];
    wordfree(&p);
    return res;
}

void SvgBoundary::loadFile(string pFilename, double res)
{
    TiXmlElement* svgElement;
    string f = tilde_expand(pFilename);
    ROS_INFO("Reading crossing boundary from %s", f.c_str());
    bool loadOkay = svgDoc_.LoadFile(f);
    if (loadOkay)
    {
        svgElement = svgDoc_.FirstChildElement();
        const char * s = svgElement->Value();
        if( strcmp(s,"svg")==0 )
        {
            if(VERBOSITY>=1) cout <<"Svg file " <<pFilename <<" loaded!" <<endl;
            getSize();
            resolution_ = res;
        }
        else
            throw runtime_error(string("Is SVG file loaded? The value found is ")+s);
    }
    else
    {
        throw runtime_error(string("Failed to load file ")+pFilename);
    }
}

void SvgBoundary::convert_to_meter(vector<geometry_msgs::Point32>& pose)
{
    vector<geometry_msgs::Point32>::iterator it;
    for(it=pose.begin(); it!=pose.end(); ++it)
    {
        it->x = it->x * resolution_;
        it->y = (height_ - it->y) * resolution_;
    }
}

void SvgBoundary::getSize()
{
    TiXmlElement* svgElement;
    svgElement = svgDoc_.FirstChildElement();

    double width, height;
    if( svgElement->QueryDoubleAttribute("width", &width)==TIXML_NO_ATTRIBUTE
            || svgElement->QueryDoubleAttribute("height", &height)==TIXML_NO_ATTRIBUTE )
    {
        throw runtime_error("Height or width information not found");
    }
    else
    {
        if(VERBOSITY>=1)
        {
            cout <<"Width found " <<width <<endl;
            cout <<"Height found " <<height <<endl;
        }
    }
    width_ = width;
    height_ = height;
}

vector<geometry_msgs::Point32> SvgBoundary::getPath(string id)
{
    vector<geometry_msgs::Point32> path;

    TiXmlNode* pChild = svgDoc_.FirstChildElement()->FirstChild();
    for ( ; pChild != 0; pChild = pChild->NextSibling() )
    {
        TiXmlElement* childElement= pChild->ToElement();
        if(VERBOSITY>=1) cout <<childElement->Value() <<endl;
        if( strcmp(childElement->Value(),"path")==0 )
        {
            const char * value = childElement->Attribute("id");
            assert( value!=NULL );
            if( strcmp(id.c_str(),value)==0 )
            {
                value = childElement->Attribute("d");
                assert( value!=NULL );
                path = StringToPath(value);
                convert_to_meter(path);
                return path;
            }
        }
    }
    throw runtime_error("Path id not found");
}

vector<geometry_msgs::Point32> SvgBoundary::StringToPath(string data)
{
    unsigned int found_points=0;

    //split the data assuming the delimiter is either space or comma
    if(VERBOSITY>=1) cout <<"Data received: " <<data <<endl;
    vector<string> data_s;
    boost::split(data_s, data, boost::is_any_of(" ,"));

    bool abs_rel;
    //a path must start with m or M
    if(data_s[0].find_first_of("Mm")==string::npos)
    {
        throw runtime_error(string("Unexpected data start character, expected M or m but received ")+data_s[0]);
    }
    else
    {
        //need to differentiate if it is abs or rel
        if(data_s[0].find_first_of("M")!=string::npos) abs_rel = true;
        else abs_rel = false;
    }

    vector<geometry_msgs::Point32> positions;
    geometry_msgs::Point32 pos;
    pos.x = atof(data_s[1].c_str());
    pos.y = atof(data_s[2].c_str());
    found_points++;
    positions.push_back(pos);
    for(unsigned int i=3; i<data_s.size(); i++)
    {
        if(data_s[i].find_first_of("HhVvSsQqTtAa")!=string::npos)
        {
            throw runtime_error(string("Only line path is supported, given ")+data_s[i]);
        }
        else if(data_s[i].find_first_of("Mm")!=string::npos)
        {
            throw runtime_error("Unexpected end of line.\n"
                    "Please check if the SVG path is drawn without new path.");

        }
        else if(data_s[i].find_first_of("Cc")!=string::npos)
        {
            if(data_s[i].find_first_of("C")!=string::npos) abs_rel=true;
            else if(data_s[i].find_first_of("c")!=string::npos) abs_rel=false;
            //only take the third point
            i+=4;
            double offsetx=0, offsety=0;
            if(!abs_rel)
            {
                offsetx = positions[positions.size()-1].x;
                offsety = positions[positions.size()-1].y;
            }
            pos.x = atof(data_s[++i].c_str())+offsetx;
            pos.y = atof(data_s[++i].c_str())+offsety;
            found_points++;
            positions.push_back(pos);

        }
        else if(data_s[i].find_first_of("Zz")==string::npos)
        {
            if(data_s[i].find_first_of("L")!=string::npos) abs_rel=true;
            else if(data_s[i].find_first_of("l")!=string::npos) abs_rel=false;
            else i--;

            double offsetx=0, offsety=0;
            if(!abs_rel)
            {
                offsetx = positions[positions.size()-1].x;
                offsety = positions[positions.size()-1].y;
            }
            pos.x = atof(data_s[++i].c_str())+offsetx;
            pos.y = atof(data_s[++i].c_str())+offsety;
            found_points++;
            positions.push_back(pos);
        }
        else
        {
            if(VERBOSITY>=1) cout<<"Closepath command ignored."<<endl;
        }


    }
    if(VERBOSITY>=1)
    {
        cout<<"Found points: "<< found_points<<" . Size of path: "<< positions.size()<<endl;
        cout<<"The above should match for a successful parse"<<endl;
    }
    if(found_points!=positions.size()) throw (string)"Size not match";
    return positions;
}
