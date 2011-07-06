NUS Golf Car	2011/06/01

1. There 2 nodes involved.
 	a. "road_detect" detect curbs from each individual laser scan; 
	b. "curb_track"  use EKF to get better performance, based on the output of "road_detect";

2. Inside the folder of "pics" are 3 pictures of results, with data "3_2011-05-12-15-07-01.bag" we collected.
	"Raw_curb"  shows the result only using "road_detect"; 
	"EKF_curb"  shows the result using both "road_detect" and "curb_track";
	"Cred_curb" shows the credible however conservative result that we can believe and use in localization;

3. There are also two *.vcg files included here, to facilitate you for visualization. 
