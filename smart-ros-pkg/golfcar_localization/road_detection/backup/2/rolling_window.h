#ifndef __ROLLING_WINDOW__
#define __ROLLING_WINDOW__

#include <cmath>
#include <vector>
#include "road_detection/vec_point.h"

namespace estimation{	
	//designed for rolling_window;
	class infoPt
   {
		public: 
		infoPt();

		road_detection::vec_point 	vecPt_;
		float 							covX_;
		float								covY_;
		bool								firstPt_;
      float                      distoFormer_;
	};

	class rolling_window
	{
		public: 
 		rolling_window();

		void rollingProcess(const infoPt &input_infoPt, bool& output_credflag);		
		
		private:
		void updateValues();

		float 							window_size_;
		unsigned int					count_size_;
		unsigned int 					countTresh_;
		float								covxTresh_;
		float								covyTresh_;
		
		float                      accumDis_;
		unsigned int 					reinitCount_;
		float								meanCovx_;
		float								meanCovy_;
		
		std::vector<infoPt> infoPts_;
 	};
};
#endif
