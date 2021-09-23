#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <unitree_legged_msgs/HighCmd.h>

// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// float32[] axes
// int32[] buttons

class JoySubA1CmdPub {
private:
  std::string m_name;

  unitree_legged_msgs::HighCmd SendHighROS;
  float SCALE_GAIN = 0.3f;

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;

  int control_mode = 1;

  /*
  mode 1
    roll, pitch, yaw, bodyHeight
  mode 2
    forwardSpeed, rotateSpeed
  mode switch button
    buttons[4]
  */

public:
  JoySubA1CmdPub(const std::string &name_in) : m_name(name_in) {
    ROS_INFO("Publisher and Subscriber initialized");

    m_pub = m_nh.advertise<unitree_legged_msgs::HighCmd>("/a1_high_cmd", 5);
    m_sub = m_nh.subscribe("/joy", 1, &JoySubA1CmdPub::subCallback, this);
  }

  ~JoySubA1CmdPub() {}

  void subCallback(const sensor_msgs::Joy &data) {
    std::cout << "1X : " << data.axes[0] << std::endl;
    std::cout << "1Y : " << data.axes[1] << std::endl;
    std::cout << "2X : " << data.axes[3] << std::endl;
    std::cout << "2Y : " << data.axes[4] << std::endl;

    if (data.buttons[4] == 1)
      control_mode = 2;
    else
      control_mode = 1;

    if (control_mode == 1 || control_mode == 0) {
      SendHighROS.mode = control_mode;
      // TODO: Check value orientation
      SendHighROS.roll = data.axes[0] * 0.3;
      SendHighROS.pitch = data.axes[1] * 0.3;
      SendHighROS.yaw = data.axes[3] * 0.3;

      //   SendHighROS.forwardSpeed = data.axes[1] * SCALE_GAIN;
      SendHighROS.forwardSpeed = 0.0f;
      SendHighROS.sideSpeed = 0.0f;
      SendHighROS.rotateSpeed = 0.0f;
      SendHighROS.bodyHeight = 0.0f;
    } else {
      SendHighROS.mode = control_mode;
      // TODO: Check value orientation
      SendHighROS.roll = 0.0f;
      SendHighROS.pitch = 0.0f;
      SendHighROS.yaw = 0.0f;

      //   SendHighROS.forwardSpeed = data.axes[1] * SCALE_GAIN;
      SendHighROS.forwardSpeed = data.axes[1] * 0.3;
      if (data.axes[0] < 0)
        SendHighROS.sideSpeed = data.axes[0] * 0.4;

      SendHighROS.sideSpeed = data.axes[0] * 0.3;
      SendHighROS.rotateSpeed = data.axes[3] * 0.3;
      SendHighROS.bodyHeight = 0.0f;
    }

    m_pub.publish(SendHighROS);

    // TODO: SendHighROS << operator
    std::cout << std::endl;
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "joy_sub_node");
  JoySubA1CmdPub joy_sub_a1_cmd("a1_controll_with_joy");

  ros::spin();

  return 0;
}