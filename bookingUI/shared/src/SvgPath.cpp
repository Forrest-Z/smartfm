#include <iostream>
#include <sstream>
#include <stdexcept>

#include <tinyxml.h>

#include "StationPath.h"
#include "SvgPath.h"

using namespace std;


// There was an API change in tinyxml 2.6. Installed version is detected by CMAKE.
// The following defines are for API version <2.6
#ifdef TINYXML_API_PRE26
#define TINYXML_ELEMENT ELEMENT
#define TINYXML_TEXT TEXT
#endif

// Whether or not to print some debugging info about paths.
#define VERBOSE 0

SvgPath::SvgPath(){}

void SvgPath::loadFile(const char* pFilename, double res)
{
    TiXmlElement* svgElement;
    bool loadOkay = svgDoc_.LoadFile(pFilename);
    if (loadOkay)
    {
        svgElement = svgDoc_.FirstChildElement();
        const char * s = svgElement->Value();
        if( strcmp(s,"svg")==0 )
        {
            if(VERBOSE) cout <<"Svg file " <<pFilename <<" loaded!" <<endl;
            size_ = getSize();
            res_ = res;
        }
        else
            throw runtime_error(string("Is SVG file loaded? The value found is ")+s);
    }
    else
    {
        throw runtime_error(string("Failed to load file ")+pFilename);
    }
}

SvgPath::SvgPath(const char* pFilename, double res)
{
    loadFile(pFilename, res);
}

vector<SlowZone> SvgPath::getSlowZone()
{
    TiXmlElement* svgElement;
    svgElement = svgDoc_.FirstChildElement();
    TiXmlNode* pChild;
    vector<SlowZone> szs;
    for ( pChild = svgElement->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
        TiXmlElement* childElement= pChild->ToElement();
        if((string)childElement->Value()=="path")
        {
            // Note: QueryStringAttribute was introduced in 2.6. Using Attribute
            // instead for compatibility with older API versions.
            const char * value;
            value = childElement->Attribute("sodipodi:type");

            if(value!=NULL && strcmp(value,"arc")==0)
            {
                double cx,cy,rx,ry;
                GetDoubleAttribute(childElement, "sodipodi:cx",&cx);
                GetDoubleAttribute(childElement, "sodipodi:cy",&cy);
                GetDoubleAttribute(childElement, "sodipodi:rx",&rx);
                GetDoubleAttribute(childElement, "sodipodi:ry",&ry);
                SlowZone sz;
                if(rx==ry)
                {
                    PathPoint tf=GetTransform(childElement);
                    sz.r_ = rx;
                    sz.x_ = cx+tf.x_;
                    sz.y_ = cy+tf.y_;
                    szs.push_back(sz);
                }
                else
                {
                    cout<<"Only circle is supported"<<endl;
                    return szs;
                }
            }
        }
    }
    for(unsigned int i=0;i<szs.size();i++)
    {

        szs[i].x_=szs[i].x_*res_;
        szs[i].y_=(size_.y_-szs[i].y_)*res_;
        szs[i].r_=szs[i].r_*res_;
        if(VERBOSE) cout<<"SlowZone"<<i<<" x y r: "<< szs[i].x_<<' '<<szs[i].y_<<' '<<szs[i].r_<<endl;
    }

    return szs;
}

void SvgPath::GetDoubleAttribute(TiXmlElement* childElement, const char* attributeName, double* value)
{
    if(childElement->QueryDoubleAttribute(attributeName,value)==TIXML_NO_ATTRIBUTE)
        throw runtime_error(string("Attribute not found, requested ")+attributeName );
    if(VERBOSE) cout <<"Attribute " <<attributeName <<" Value " << *value <<endl;
}

vector<string> SplitString(const char *data, const char* delimiter)
{
    char *str = strdup(data);
    char *pch = strtok(str, delimiter);
    vector<string> data_s;
    while (pch != NULL)
    {
        data_s.push_back(string(pch));
        pch = strtok(NULL, delimiter);
    }
    free(str);
    return data_s;
}

PathPoint SvgPath::GetTransform(TiXmlElement* childElement)
{
    PathPoint tf;
    const char * value = childElement->Attribute("transform");
    if( value==NULL )
    {
        tf.x_=0; tf.y_=0;
    }
    else
    {
        vector<string> data = SplitString(value, "(,)");
        if(data.size()==3 && data[0]=="translate")
        {
            tf.x_ = atof(data[1].c_str());
            tf.y_ = atof(data[2].c_str());
            
        }
        else
            throw runtime_error("Unexpected transform data format");
    }
    if(VERBOSE) cout <<"Transform x: " <<tf.x_ <<" y: " <<tf.y_ <<endl;
    return tf;
}

PathPoint SvgPath::getSize()//(TiXmlDocument* svgDoc)//PathPoint* size)
{
    TiXmlElement* svgElement;
    svgElement = svgDoc_.FirstChildElement();

    double width,height;
    if( svgElement->QueryDoubleAttribute("width", &width)==TIXML_NO_ATTRIBUTE
            || svgElement->QueryDoubleAttribute("height", &height)==TIXML_NO_ATTRIBUTE )
    {
        throw runtime_error("Height or width information not found");
    }
    else
    {
        if(VERBOSE)
        {
            cout <<"Width found " <<width <<endl;
            cout <<"Height found " <<height <<endl;
        }
    }
    PathPoint pp;
    pp.x_ = width;
    pp.y_ = height;
    return pp;
}

StationPath SvgPath::getPath(string id)
{
    TiXmlElement* svgElement;
    svgElement = svgDoc_.FirstChildElement();
    TiXmlNode* pChild;
    for ( pChild = svgElement->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
        TiXmlElement* childElement= pChild->ToElement();
        if( strcmp(childElement->Value(),"path")==0 )
        {
            const char * value = childElement->Attribute("id");
            assert( value!=NULL );
            if( strcmp(id.c_str(),value)==0 )
            {
                value = childElement->Attribute("d");
                assert( value!=NULL );
                StationPath path = StringToPath(value);
                convert_to_meter(&path);
                return path;
            }
        }
    }
    throw runtime_error("Path id not found");
}

void SvgPath::convert_to_meter(StationPath* pose)
{
    for(unsigned i=0; i<pose->size(); i++)
    {
        (*pose)[i].x_ = (*pose)[i].x_*res_;
        (*pose)[i].y_ = (size_.y_ - (*pose)[i].y_)*res_;
    }
}

StationPath SvgPath::StringToPath(string data)
{
    unsigned int found_points=0;
    //split the data assuming the delimiter is either space or comma
    if(VERBOSE) cout <<"Data received: " <<data <<endl;
    vector<string> data_s = SplitString(data.c_str(), " ,");
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

    StationPath positions;
    PathPoint pos;
    pos.x_ = atof(data_s[1].c_str());
    pos.y_ = atof(data_s[2].c_str());
    found_points++;
    positions.push_back(pos);
    for(unsigned int i=3; i<data_s.size(); i++)
    {
        if(data_s[i].find_first_of("MmHhVvSsQqTtAa")!=string::npos)
        {
            throw runtime_error(string("Only line path is supported, given ")+data_s[i]);
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
                offsetx = positions[positions.size()-1].x_;
                offsety = positions[positions.size()-1].y_;
            }
            pos.x_ = atof(data_s[++i].c_str())+offsetx;
            pos.y_ = atof(data_s[++i].c_str())+offsety;
            found_points++;
            positions.push_back(pos);

            found_points++;
            //The stop point
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
                offsetx = positions[positions.size()-1].x_;
                offsety = positions[positions.size()-1].y_;
            }
            pos.x_ = atof(data_s[++i].c_str())+offsetx;
            pos.y_ = atof(data_s[++i].c_str())+offsety;
            found_points++;
            positions.push_back(pos);
        }
        else
        {
            if(VERBOSE) cout<<"Closepath command ignored."<<endl;
        }


    }
    if(VERBOSE)
    {
        cout<<"Found points: "<< found_points<<" . Size of path: "<< positions.size()<<endl;
        cout<<"The above should match for a successful parse"<<endl;
    }
    if(found_points!=positions.size()) throw (string)"Size not match";
    return positions;
}
