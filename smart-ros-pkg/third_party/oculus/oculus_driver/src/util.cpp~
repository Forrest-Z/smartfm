#include <oculus_driver/util.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;
namespace oculus_driver {

void convertHMDInfoToMsg(const OVR::HMDInfo& info, oculus_msgs::HMDInfo& msg) {
  msg.display_device_name = info.DisplayDeviceName;
  msg.product_name = info.ProductName;
  msg.manufacturer = info.Manufacturer;
  msg.version = info.Version;
  msg.horizontal_resolution = info.HResolution;
  msg.vertical_resolution = info.VResolution;
  msg.horizontal_screen_size = info.HScreenSize;
  msg.vertical_screen_size = info.VScreenSize;
  msg.vertical_screen_center = info.VScreenCenter;
  msg.eye_to_screen_distance = info.EyeToScreenDistance;
  msg.lens_separation_distance = info.LensSeparationDistance;
  msg.interpupillary_distance = info.InterpupillaryDistance;
  msg.distortion_K.push_back(info.DistortionK[0]);
  msg.distortion_K.push_back(info.DistortionK[1]);
  msg.distortion_K.push_back(info.DistortionK[2]);
  msg.desktop_x = info.DesktopX;
  msg.desktop_y = info.DesktopY;
  msg.display_id = info.DisplayId;
}

void convertYPRToMsg(float yaw, float pitch, float roll,
		     geometry_msgs::Quaternion& msg){
  btMatrix3x3 btm;
  btm.setEulerYPR(yaw, pitch, roll);
  btQuaternion btqt_temp;
  btm.getRotation(btqt_temp);
  msg.x = btqt_temp.x();
  msg.y = btqt_temp.y();
  msg.z = btqt_temp.z();
  msg.w = btqt_temp.w();
}
void convertQuaternionToMsg(const OVR::Quatf& quaternion,
			    geometry_msgs::Quaternion& msg) {
  msg.x = quaternion.x;
  msg.y = quaternion.y;
  msg.z = quaternion.z;
  msg.w = quaternion.w;
  /*btScalar pitch, roll, yaw;
  btScalar r_pitch, r_roll, r_yaw;
  btQuaternion btq(msg.w, msg.x, msg.y, msg.z);
  btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);
  r_pitch = -roll;
  r_roll = -yaw;
  r_yaw = pitch;
  std::cout.setf(std::ios::fixed);
    std::cout.precision(2);
  cout<<roll/M_PI*180<<"\t"<<pitch/M_PI*180<<"\t"<<yaw/M_PI*180<<endl;
  cout<<r_roll/M_PI*180<<"\t"<<r_pitch/M_PI*180<<"\t"<<r_yaw/M_PI*180<<endl;
  
  btMatrix3x3 btm;
  btm.setEulerYPR(r_yaw, r_pitch, r_roll);
  btQuaternion btqt_temp;
  btm.getRotation(btqt_temp);
  msg.x = btqt_temp.x();
  msg.y = btqt_temp.y();
  msg.z = btqt_temp.z();
  msg.w = btqt_temp.w();*/
}

}  // namespace oculus_ros
