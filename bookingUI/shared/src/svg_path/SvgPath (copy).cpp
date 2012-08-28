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

SvgPath::SvgPath(const char* pFilename)
{
    bool loadOkay = svgFile_.LoadFile(pFilename);
    if (loadOkay)
    {
        //TiXmlElement* pParent=svgFile_.ToElement();//.ToElement();
        TiXmlDocument svgFile_;
        TiXmlElement* pChild = svgFile_.FirstChildElement();
        TiXmlNode * pNode;
        cout<<"Svg file? "<<pChild->Value()<<endl;
        double width,height;
        pChild->QueryDoubleAttribute("width", &width);
        pChild->QueryDoubleAttribute("height", &height);
        cout<<"Size= "<<width<<' '<<height<<' '<<endl;
        for(pNode = pChild->FirstChild(); pNode != 0; pNode =pNode->NextSibling())
        {
            cout<<pNode->Value()<< ' '<<pNode->Type()<<endl;//pParent->FirstChild()->Value()<<endl;
            TiXmlElement * ppChild=pNode->ToElement();
            TiXmlAttribute * ppNode;
            string attribute_value;
            ppChild->QueryStringAttribute("id", &attribute_value);//(attribute_value,"id");//attributeSet.Find("id");
            cout<<"\t"<< attribute_value<<endl;
            ppChild->QueryStringAttribute("d", &attribute_value);
            //cout<<"\t"<< attribute_value<<endl;
            if(ppChild->QueryStringAttribute("sodipodi:type", &attribute_value)!=TIXML_NO_ATTRIBUTE)
                cout<<"\t"<< attribute_value<<endl;
            //for(ppNode = ppChild->FirstAttribute();ppNode!=0;ppNode=ppNode->Next())
              //  cout<<"\t"<<ppNode->Value()<<endl;
        }
    }
    else
    {
        throw string_error("Failed to load file", pFilename);
    }
}

SvgPath::SvgPath(const char* pFilename, StationPath* pose, PathPoint *size, const char* id)
{
    foundPath=0;
    bool loadOkay = svgFile_.LoadFile(pFilename);
    if (loadOkay)
    {

        unsigned int npoints=0;
        SvgPath::findSize(&svgFile_, size);
        SvgPath::findPathElements( &svgFile_, pose, &npoints, id);

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

void SvgPath::getSlowZones(std::vector<SlowZone>* slowZones)
{
    TiXmlNode* pParent = &svgFile_;
    if ( !pParent) return;
    TiXmlNode* pChild;
        int t = pParent->Type();
        stringstream ss;
        if(t==TiXmlNode::TINYXML_ELEMENT)
        {
            ss<<pParent->Value();
            if(ss.str()=="path")
            {
                unsigned int num=0;
                try
                {
                    //num = find_sz_attributes(pParent->ToElement(),slowZones);
                }
                catch (string error)
                {
                    throw error;
                }

                foundSlowZone++;
                //*npoints=num;

            }
        }

}
/*
void SvgPath::findSlowZone(TiXmlNode* pParent, std::vector<SlowZone>* slowZones)
{
    //todo: implement the about the same method of searching fo path.
    //Can this 2 slightly different method be componentize in some way?
    if ( !pParent) return;
    TiXmlNode* pChild;
    int t = pParent->Type();
    stringstream ss;
    if(t==TiXmlNode::TINYXML_ELEMENT)
    {
        ss<<pParent->Value();
        if(ss.str()=="path")
        {

        }
    }
}*/

//void SvgPath::find_sz_


void SvgPath::findSize(TiXmlNode* pParent, PathPoint* size)
{
    if ( !pParent) return;
    TiXmlNode* pChild;
    int t = pParent->Type();
    stringstream ss;
    if(t==TiXmlNode::TINYXML_ELEMENT)
    {
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
                return;
            }
        }
    }
    for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
        findSize(pChild, size);
    }
}

void SvgPath::findPathElements(TiXmlNode* pParent, StationPath* pose, unsigned int *npoints, const char* id)
{
    if ( !pParent || *npoints>0) return;

    TiXmlNode* pChild;

    stringstream ss;
    //only look for elements
    int t = pParent->Type();
    if(t==TiXmlNode::TINYXML_ELEMENT)
    {
        ss<<pParent->Value();
        if(ss.str()=="path")
        {
            unsigned int num=0;
            try
            {
                num = find_path_attributes(pParent->ToElement(),pose, id);
            }
            catch (string error)
            {
                throw error;
            }

            foundPath++;
            *npoints=num;

        }
    }

    for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
        findPathElements( pChild, pose, npoints, id);
    }
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

bool SvgPath::match_path_id(TiXmlElement* pElement, const char* id)
{
    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    if ( !pElement ) return 0;
    while (pAttrib)
    {
        stringstream ss;
        ss<<pAttrib->Name();
        if(ss.str()=="id")
        {
            stringstream s; s<<pAttrib->Value();

            if(s.str().compare(id)!=0)
            {
                //just skip the rest of the attributes, it is not the path we want to find
                return false;
            }
            else
            {
                if(VERBOSE) cout<<"ID found: "<<s.str()<<endl;
                return true;
            }
        }
        pAttrib=pAttrib->Next();
    }
}

int SvgPath::find_path_details(TiXmlElement* pElement, StationPath* pose)
{
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
                if(data_s[i].find_first_of("MmHhVvSsQqTtAa")!=string::npos)
                {
                    throw string_error("Only line path is supported, given ",data_s[i]);
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
            *pose = positions;
        }


        pAttrib=pAttrib->Next();
    }

    return found_points;
}
int SvgPath::find_path_attributes(TiXmlElement* pElement, StationPath* pose, const char* id)
{
    int match = SvgPath::match_path_id(pElement, id);
    if(!match) return 0;
    return SvgPath::find_path_details(pElement, pose);
}

int main(int argc, char* argcv[])
{
    SvgPath sp(argcv[1]);
}
