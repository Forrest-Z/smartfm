#ifdef TINYXML_API_PRE26
#define TINYXML_ELEMENT ELEMENT
#define TINYXML_TEXT TEXT
#endif

#include <stdexcept>

#include <tinyxml.h>
#define VERBOSE 1

// this reads a path from the SVG file.
// It's actually quite similar to the SvgPath class, but Demian can't recall
// why we re-implemented from scratch...
using namespace std;
class svg_boundary
{
public:
	TiXmlDocument svgDoc_;
	geometry_msgs::Point32 size_;
	double res_;

	void loadFile(const char* pFilename, double res)
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

	void convert_to_meter(vector<geometry_msgs::Point32>& pose)
	{
	    for(unsigned i=0; i<pose.size(); i++)
	    {
	        (pose)[i].x = (pose)[i].x*res_;
	        (pose)[i].y = (size_.y - (pose)[i].y)*res_;
	    }
	}
	geometry_msgs::Point32 getSize()//(TiXmlDocument* svgDoc)//PathPoint* size)
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
	    geometry_msgs::Point32 pp;
	    pp.x = width;
	    pp.y = height;
	    return pp;
	}
	svg_boundary(const char* pFilename, double res)
	{
	    loadFile(pFilename, res);
	}

	vector<geometry_msgs::Point32> getPath(string id)
	{
	    TiXmlElement* svgElement;
	    svgElement = svgDoc_.FirstChildElement();
	    TiXmlNode* pChild;
	    for ( pChild = svgElement->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
	    {
	        TiXmlElement* childElement= pChild->ToElement();
	        cout<<childElement->Value()<<endl;
	        if( strcmp(childElement->Value(),"path")==0 )
	        {
	            const char * value = childElement->Attribute("id");
	            assert( value!=NULL );
	            if( strcmp(id.c_str(),value)==0 )
	            {
	                value = childElement->Attribute("d");
	                assert( value!=NULL );
	                vector<geometry_msgs::Point32> path = StringToPath(value);
	                convert_to_meter(path);
	                return path;
	            }
	        }
	    }
	    throw runtime_error("Path id not found");
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

	vector<geometry_msgs::Point32> StringToPath(string data)
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
};
