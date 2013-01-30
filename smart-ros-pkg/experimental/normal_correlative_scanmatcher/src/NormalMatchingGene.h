#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "RasterMapPCL.h"
#include <fstream>
#include <boost/thread/thread.hpp>

#include "NormalsCorrelativeMatchingProblem.h"
#include "problems.h"
using std::cout;
using std::endl;
class ValueAndRange
{
	//an interface to convert real value to integers and to gray value
public:

	ValueAndRange(double vmin, double vmax, double vres):
		min(vmin), max(vmax), res(vres)
	{
		//these are the assumptions made to ensure randomization works
		//also the gene encoding
		nbit=0;
		while(pow(2., ++nbit)<((vmax-vmin)/res));
		//assert(max>0);
		assert(max>min);
	};

	void setRealValue(double real_v)
	{
		value = ((real_v- min)/res );
		real_value = real_v;
	}

	void setValue(int v)
	{
		value = v;
		real_value = v*res + min;
	}

	int getValue()
	{
		return value;
	}

	double getRealValue()
	{
		return real_value;
	}

	int getNBit()
	{
		return nbit;
	}

	double getRes()
	{
		return res;
	}

	double getMin()
	{
		return min;
	}

	double getMax()
	{
		return max;
	}
private:
	double real_value;
	int value;
	double min;
	double max;
	double res;

	int nbit;
};

uint32_t gray_encode(uint32_t b)
{
	return b ^ (b >> 1);
}

uint32_t gray_decode(uint32_t g)
{
	for (uint32_t bit = 1U << 31; bit > 1; bit >>= 1)
	{
		if (g & bit) g ^= bit >> 1;
	}
	return g;
}

std::string to_binary(int value, int bitn) // utility function
{
	const std::bitset<32> bs(value);
	const std::string str(bs.to_string());
	return str.substr(32-bitn);
}

uint32_t BinaryStrToInt(string grayStr)
{
	char * ptr;
	return strtol(grayStr.c_str(), & ptr, 2);
}
//a 3 variables gene
class Positional2dGene
{
public:
	string data;
	vector<ValueAndRange> pose;
	double score;
	Positional2dGene(ValueAndRange x, ValueAndRange y, ValueAndRange r)
	{
		pose.push_back(x);
		pose.push_back(y);
		pose.push_back(r);
		data = VNRtoGray(pose);

	}
	Positional2dGene()
	{
		score = 1e-999;
	}
	vector<string> getIndiviualStr()
	{
		vector<string> strs;
		int getnbit = pose[0].getNBit();
		strs.push_back( data.substr(0, pose[0].getNBit()));
		strs.push_back( data.substr(pose[0].getNBit(), pose[1].getNBit()));
		strs.push_back( data.substr(pose[0].getNBit()+pose[1].getNBit()));
		return strs;
	}

	vector<double> GraytoPose()
	{//0-8 9-16 17-25
		vector<string> strs = getIndiviualStr();
		string pose_x = strs[0], pose_y = strs[1], rotation = strs[2];
		vector<double> pose_temp;
		pose_temp.push_back(gray_decode(BinaryStrToInt(pose_x))*pose[0].getRes()+pose[0].getMin());
		pose_temp.push_back(gray_decode(BinaryStrToInt(pose_y))*pose[1].getRes()+pose[1].getMin());
		pose_temp.push_back(gray_decode(BinaryStrToInt(rotation))*pose[2].getRes()+pose[2].getMin());
		return pose_temp;
	}


	string VNRtoGray(vector<ValueAndRange> in)
	{

		stringstream ss;
		int total_bit = 0;
		for(size_t i=0; i<in.size(); i++)
		{
			uint32_t gray_code = gray_encode(in[i].getValue());
			ss<<to_binary(gray_code, in[i].getNBit());
			total_bit += in[i].getNBit();
		}
		assert(total_bit<32);
		return ss.str();
	}

	void updateScore(NormalsCorrelativeMatchingProblem<3> &ncmp)
	{
		vector<double> poses = GraytoPose();
		if(poses[2]>179 && poses[2]<200) poses[2]=poses[2]-360;
    if(poses[2]>=200) poses[2] -= 220;
		double manual_pose[] = {poses[0], poses[1], poses[2]/180.*M_PI};
		score = ncmp.evaluate(manual_pose);
	}
};
