/*
 * basic topic subscriber example
 *
 * referenced from wiki.ros.org :
 * http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */

#include "convert.h"
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

class A1Test {
private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
  ros::Rate loop_rate{500};

  std::string m_name;
  unitree_legged_msgs::HighCmd m_cmd_vel;

  UNITREE_LEGGED_SDK::HighCmd SendHighLCM = {0};
  UNITREE_LEGGED_SDK::HighState RecvHighLCM = {0};
  UNITREE_LEGGED_SDK::LCM roslcm{UNITREE_LEGGED_SDK::HIGHLEVEL};

  unitree_legged_msgs::HighCmd SendHighROS;
  unitree_legged_msgs::HighState RecvHighROS;

public:
  A1Test(const std::string &name_in = "my_tiny") : m_name(name_in) {
    roslcm = UNITREE_LEGGED_SDK::LCM(UNITREE_LEGGED_SDK::HIGHLEVEL);
    m_sub = m_nh.subscribe("/a1_high_cmd", 1, &A1Test::subCallback, this);
  }

  void subCallback(const unitree_legged_msgs::HighCmd &data) {
    std::cout << data.roll << std::endl;
    std::cout << data.pitch << std::endl;
    std::cout << data.yaw << std::endl;

    SendHighROS.mode = 1;
    SendHighROS.roll = data.roll;
    SendHighROS.pitch = data.pitch;
    SendHighROS.yaw = data.yaw;

    SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
    roslcm.Send(SendHighLCM);
    loop_rate.sleep();
  }

  void mainHelper() {
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "basic_cmd_sub_node");

  A1Test my_a1("my_a1");

  my_a1.mainHelper();

  ros::spin();

  return 0;
}
