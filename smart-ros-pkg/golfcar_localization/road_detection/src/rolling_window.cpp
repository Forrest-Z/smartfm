#include "rolling_window.h"

namespace estimation{
	//*************************class of "infoPt"*****************
	infoPt::infoPt():firstPt_(false){}

	//*************************class of "rolling_window"*******************
	rolling_window::rolling_window(): 
	window_size_(3.0),count_size_(40),countTresh_(3),covxTresh_(0.6),covyTresh_(0.6), //these 4 parameters to be determined/trained further; 
	accumDis_(0.0),reinitCount_(0),meanCovx_(0.0),meanCovy_(0.0)
	{}
	
	void rolling_window::rollingProcess(const infoPt& input_infoPt, bool& output_credflag)
	{
		float accumdis = input_infoPt.distoFormer_+accumDis_;
		unsigned int temp_count = infoPts_.size();
		infoPts_.push_back(input_infoPt);
		
		//must meet both requirements; 
		while (accumdis > window_size_&&temp_count>count_size_)
		{
			accumdis = accumdis-infoPts_.front().distoFormer_;
			infoPts_.erase(infoPts_.begin());
			temp_count = infoPts_.size();
		}
		accumDis_ = accumdis;
		rolling_window::updateValues();
		if(reinitCount_>=countTresh_||meanCovx_>covxTresh_||meanCovx_>covyTresh_){output_credflag=false;}
	}
	
	void rolling_window::updateValues()
	{
		unsigned int countnum=0;
		float covxsum =0;
		float covysum =0;
		for(unsigned int i=0; i<infoPts_.size(); i++)
		{
			if(infoPts_[i].firstPt_==true){countnum=countnum+1;}
			covxsum = covxsum + infoPts_[i].covX_;
			covysum = covysum + infoPts_[i].covY_;
		}
		reinitCount_ = countnum;
		float vecsize = infoPts_.size();
		meanCovx_ = covxsum/vecsize;
		meanCovy_ = covysum/vecsize;
		ROS_INFO("reinitCount: %d; meanCovx_,meanCovy_: (%f,%f)", reinitCount_, meanCovx_, meanCovy_);
	}
}
