
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <unitree_legged_msgs/HighCmd.h>

// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// float32[] axes
// int32[] buttons

unitree_legged_msgs::HighCmd SendHighROS;
auto SCALE_GAIN = 0.3f;

void sub_callback(const sensor_msgs::Joy &data) {
  std::cout << "1X : " << data.axes[0] << std::endl;
  std::cout << "1Y : " << data.axes[1] << std::endl;
  std::cout << "2X : " << data.axes[3] << std::endl;
  std::cout << "2Y : " << data.axes[4] << std::endl;

  std::cout << std::endl;

  SendHighROS.mode = 1;
  SendHighROS.roll = data.axes[3] * SCALE_GAIN;
  SendHighROS.pitch = data.axes[4] * SCALE_GAIN;
  SendHighROS.yaw = data.axes[0] * SCALE_GAIN;

  //   SendHighROS.forwardSpeed = data.axes[1] * SCALE_GAIN;
  SendHighROS.forwardSpeed = 0.0f;
  SendHighROS.sideSpeed = 0.0f;
  SendHighROS.rotateSpeed = 0.0f;
  SendHighROS.bodyHeight = 0.0f;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "joy_sub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/joy", 1, sub_callback);

  ros::spin();

  return 0;
}