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
    else if (data.buttons[5] == 1)
      control_mode = 3;
    else if (data.buttons[6] == 1)
      control_mode = 4;
    else
      control_mode = 1;

    if (control_mode == 1 || control_mode == 0) {
      SendHighROS.mode = control_mode;
      // TODO: Check value orientation
      SendHighROS.euler[0] = data.axes[0] * 0.3;
      SendHighROS.euler[1] = data.axes[1] * 0.3;
      SendHighROS.euler[2] = data.axes[3] * 0.3;
    } else if (control_mode == 2) {
      // basic trot mode
      SendHighROS.mode = control_mode;
      SendHighROS.gaitType = 1;

      SendHighROS.velocity[0] = data.axes[0] * 0.3; // -1  ~ +1
      SendHighROS.velocity[1] = data.axes[1] * 0.3; // -1  ~ +1
      // SendHighROS.yawSpeed = data.axes[2] * 0.3;
      SendHighROS.bodyHeight = 0.1;

      // if (data.axes[0] < 0)
      //   SendHighROS.sideSpeed = data.axes[0] * 0.4;
      // SendHighROS.sideSpeed = data.axes[0] * 0.3;
      // SendHighROS.rotateSpeed = data.axes[3] * 0.3;
    } else if (control_mode == 3) {
      // trot running mode
      SendHighROS.mode = 2;
      SendHighROS.gaitType = 2;

      SendHighROS.velocity[0] = data.axes[0] * 0.3; // -1  ~ +1
      SendHighROS.velocity[1] = data.axes[1] * 0.3; // -1  ~ +1
      SendHighROS.yawSpeed = data.axes[2] * 0.3;
      // SendHighROS.bodyHeight = 0.1;
      SendHighROS.footRaiseHeight = 0.1;
    } else if (control_mode == 4) {
      // stairs climbing mode
      SendHighROS.mode = 2;
      SendHighROS.gaitType = 3;

      SendHighROS.velocity[0] = data.axes[0] * 0.3; // -1  ~ +1
      SendHighROS.velocity[1] = data.axes[1] * 0.3; // -1  ~ +1

      SendHighROS.bodyHeight = 0.1;
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