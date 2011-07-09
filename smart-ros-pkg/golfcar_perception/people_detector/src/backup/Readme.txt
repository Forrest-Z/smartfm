1. The 1st part: (a)laser_projection,(b)naive_laser_projection, and (c)naive_based_projection are used to transform laserscan into pointcloud that we need. 
   Recent Update(2011_01-21):already integrated into people_detector.cpp, according to laser_pipeline tutorial.

2. The 2nd part: people_detector.cpp is the main structure dealing with people detection. Currently, it realize background initialization.

3. THe 3rd part: it should be integrated into the 2nd part. 
   Firstly, substraction; 
   Secondly, clustering;
   Thirdly, classification;
   Forthly, labeling. 
