#include <ros/ros.h>
#include <image_transport/image_transport.h>
namespace golfcar_vision{

    class addCamInfo {

    public:
        addCamInfo(ros::NodeHandle&);
        ~addCamInfo();
        void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);

    private:
        ros::NodeHandle n_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Publisher CamInfo_pub_;
        sensor_msgs::CameraInfo CameraStaticInfo_;
    };

    addCamInfo::addCamInfo(ros::NodeHandle &n):
    n_(n), it_(n_)
    {
        ros::NodeHandle nh;
        CamInfo_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera_front/camera_info", 2);
        image_sub_ = it_.subscribe("/camera_front/image_raw", 1, &addCamInfo::imageCallback, this);

        CameraStaticInfo_.width = 640;
        CameraStaticInfo_.height = 360;
        CameraStaticInfo_.distortion_model = "plumb_bob";

        CameraStaticInfo_.D.push_back(-0.00791);
        CameraStaticInfo_.D.push_back(-0.05311);
        CameraStaticInfo_.D.push_back(0.00462);
        CameraStaticInfo_.D.push_back(0.00101);
        CameraStaticInfo_.D.push_back(0.00000);

        CameraStaticInfo_.K[0]= 462.55911;
        CameraStaticInfo_.K[1]= 0.0;
        CameraStaticInfo_.K[2]= 326.21463;
        CameraStaticInfo_.K[3]= 0.0;
        CameraStaticInfo_.K[4]= 472.34580;
        CameraStaticInfo_.K[5]= 188.72264;
        CameraStaticInfo_.K[6]= 0.0;
        CameraStaticInfo_.K[7]= 0.0;
        CameraStaticInfo_.K[8]= 1.00;

        CameraStaticInfo_.R[0]= 1.0;
        CameraStaticInfo_.R[1]= 0.0;
        CameraStaticInfo_.R[2]= 0.0;
        CameraStaticInfo_.R[3]= 1.0;
        CameraStaticInfo_.R[4]= 0.0;
        CameraStaticInfo_.R[5]= 0.0;
        CameraStaticInfo_.R[7]= 0.0;
        CameraStaticInfo_.R[8]= 1.0;

        CameraStaticInfo_.P[0]= 454.09927;
        CameraStaticInfo_.P[1]= 0.0;
        CameraStaticInfo_.P[2]= 326.85492;
        CameraStaticInfo_.P[3]= 0.0;
        CameraStaticInfo_.P[4]= 0.0;
        CameraStaticInfo_.P[5]= 469.67014;
        CameraStaticInfo_.P[6]= 189.22427;
        CameraStaticInfo_.P[7]= 0.0;
        CameraStaticInfo_.P[8]= 0.0;
        CameraStaticInfo_.P[9]= 0.0;
        CameraStaticInfo_.P[10]= 1.0;
        CameraStaticInfo_.P[11]= 0.0;
    }

    addCamInfo::~addCamInfo(){}

    void addCamInfo::imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        CameraStaticInfo_.header = msg_ptr->header;
        CameraStaticInfo_.header.frame_id = "camera_front_img";
        CamInfo_pub_.publish(CameraStaticInfo_);
        return;
    }

};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "addCamInfo");
	 ros::NodeHandle n;
	 golfcar_vision::addCamInfo addCamInfo_node(n);
     ros::spin();
     return 0;
}

