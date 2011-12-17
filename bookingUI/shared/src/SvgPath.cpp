#include <iostream>
#include <sstream>

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


// Whether or not to pring some debugging info about paths.
#define VERBOSE 0




SvgPath::SvgPath(const char* pFilename, StationPath* pose, PathPoint *size, const char* id)
{
    TiXmlDocument doc(pFilename);
    foundPath=0;
    bool loadOkay = doc.LoadFile();
    if (loadOkay)
    {
        
        unsigned int npoints=0;
        SvgPath::findPathElements( pose, size, &npoints, &doc, id);
        if(VERBOSE)
        {
            cout<<pFilename<<endl;
            cout<<id<<" path found at "<<foundPath<<"th path"<<endl;
        }
        if(pose->size()==npoints) 
        {
            if(VERBOSE) cout<<"Path with "<<npoints<<" points successfully loaded"<<endl;
        }
    }
    else
    {
        throw string_error("Failed to load file", pFilename);
    }
}


void SvgPath::findPathElements(StationPath* pose, PathPoint* size, unsigned int *npoints, TiXmlNode* pParent, const char* id)
{
    if ( !pParent || *npoints>0) return;

    TiXmlNode* pChild;

    stringstream ss;
    //only look for elements
    int t = pParent->Type();
    switch (t)
    {
    case TiXmlNode::TINYXML_ELEMENT:
        ss<<pParent->Value();
        if(ss.str()=="svg")
        {
            PathPoint width_height;
            if(find_size_attributes(pParent->ToElement(),&width_height)==0)
            {
                throw "No width height information found, is correct svg file used?";
            }
            else
            {
                *size = width_height;
                if(VERBOSE) cout<<"Width: "<<size->x_<<" Height: "<<size->y_<<endl;
            }
        }
        else if(ss.str()=="path")
        {
            unsigned int num = find_path_attributes(pParent->ToElement(),pose, id);
            //cout<<"Found points "<<num<<endl;
            foundPath++;
            *npoints=num;

        }
        break;
    case TiXmlNode::TINYXML_TEXT:
        break;
    }

    for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
        findPathElements( pose,size,npoints,pChild, id);
    }

    //return;
}
string SvgPath::string_error(string description, string details)
{
    stringstream s;
    s<<description<<' '<< details;
    return s.str();
}

int SvgPath::find_size_attributes(TiXmlElement* pElement, PathPoint *width_height)
{
    if ( !pElement ) return 0;
    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    unsigned int width=0, height=0;
    while (pAttrib)
    {
        stringstream ss;
        ss<<pAttrib->Name();
        string data_str=pAttrib->Value();
        const char cset[] = "1234567890.";
        if(ss.str()=="width")
        {
            if(strspn (data_str.c_str(),cset)==data_str.length() || data_str.find("px")!=string::npos)
            {
                width_height->x_ = atof(data_str.c_str());
                width++;
            }
            else
            {
                throw string_error("Width unit not supported, please use px, given",data_str);
            }
        }
        if(ss.str()=="height")
        {
            if(strspn (data_str.c_str(),cset)==data_str.length() || data_str.find("px")!=string::npos)
            {
                width_height->y_ = atof(data_str.c_str());
                height++;
            }
            else
            {
                throw string_error("Height unit not supported, please use px, given",data_str);
            }
        }
        if(height==1&&width==1) return 1;
        pAttrib=pAttrib->Next();
    }
    return 0;
}

void SvgPath::convert_to_meter(StationPath* pose, PathPoint &size, double res)
{
    for(unsigned i=0; i<pose->size(); i++)
    {
        (*pose)[i].x_ = (*pose)[i].x_*res;
        (*pose)[i].y_ = (size.y_ - (*pose)[i].y_)*res;
    }
}

int SvgPath::find_path_attributes(TiXmlElement* pElement, StationPath* pose, const char* id)
{
    if ( !pElement ) return 0;

    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    int found_points=0;

    while (pAttrib)
    {
        stringstream ss;
        ss<<pAttrib->Name();

        if(ss.str()=="d")
        {
            string data_str=pAttrib->Value();
            //split the data assuming the delimiter is either space or comma
            char *str, *pch;
            str = new char[data_str.size()+1];
            strcpy(str,data_str.c_str());
            pch = strtok(str, " ,");
            vector<string> data_s;
            while (pch != NULL)
            {
                stringstream pchss;
                pchss<<pch;
                data_s.push_back(pchss.str());
                pch = strtok(NULL, ", ");
            }
            bool abs_rel;
            //a path must start with m or M
            if(data_s[0].find_first_of("Mm")==string::npos)
            {
                throw string_error("Unexpected data start character, expected M or m but received",data_s[0]);
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
                if(data_s[i].find_first_of("MmHhVvCcSsQqTtAa")!=string::npos)
                {
                    throw string_error("Only line path is supported, given",data_s[i]);
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
            *pose = positions;
        }
        if(ss.str()=="id")
        {
            stringstream s; s<<pAttrib->Value();
            if(s.str().compare(id)!=0) found_points=0;
             return found_points;
        }

        pAttrib=pAttrib->Next();
    }
    return found_points;
}


