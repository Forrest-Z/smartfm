# Readme text file for vision_opticalflow/src/lib

    [FeatureExtractor_bkup_10082012.cpp]
    Note: Backup code of first working optical flow project

    [FeatureExtractor_bkup_10092012.cpp]
    Note: Backup code after do some clean-up, remove unnesscary variable and comments

[09/10/2012]
define new message type called Feature.msg -> including two vector of type float (contain feature points of prev_feature and found_feature) and header (contain header of current frame).
    for reference :
        header -> http://ros.org/doc/api/std_msgs/html/msg/Header.html
        geometry_msgs/Point32 -> http://ros.org/doc/api/geometry_msgs/html/msg/Point32.html
        common meassage -> http://www.ros.org/wiki/common_msgs

    mask_roi for goodFeatureToTrack is added - roi is a simple rectangle shape (will chage to rotated-rectangle or polygon later).
    
    [FeatureExtractor_bkup_10102012.cpp] backup code : now with ROI and publish message containing feature point (using Feature.msg).

    now have faeture_display node to display data from feature_extractor node.

[10/10/2012]
 

    