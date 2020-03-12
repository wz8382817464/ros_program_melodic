
#include "microstrain_3dmgx2_imu/imu_ros_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "microstrain_3dmgx2_node");

  ros::NodeHandle nh;

  ImuNode in(nh);
  in.spin();

  return(0);
}
