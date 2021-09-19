/*
 * Basic Publisher for A1 walking example
 *
 * created by kimsooyoung : https://github.com/kimsooyoung
 */

#include <chrono>
#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

class MyA1 {
private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;

  std::string m_name;
  unitree_legged_msgs::HighCmd m_high_cmd;
  std::chrono::steady_clock::time_point m_start_time;

public:
  MyA1(const std::string &name_in = "my_a1") : m_name(name_in) {
    ROS_INFO("Publisher and Subscriber initialized");
    m_pub = m_nh.advertise<unitree_legged_msgs::HighCmd>("/a1_high_cmd", 5);

    m_start_time = std::chrono::steady_clock::now();
  }

  void sendInitialCmd() {
    m_high_cmd.forwardSpeed = 0.0f;
    m_high_cmd.sideSpeed = 0.0f;
    m_high_cmd.rotateSpeed = 0.0f;
    m_high_cmd.bodyHeight = 0.0f;

    m_high_cmd.mode = 0;
    m_high_cmd.roll = 0;
    m_high_cmd.pitch = 0;
    m_high_cmd.yaw = 0;

    m_pub.publish(m_high_cmd);
  }

  void rollCtrl(float roll_in = 0.3f) {
    ROS_INFO("rollCtrl");
    this->sendInitialCmd();

    m_high_cmd.mode = 1;
    m_high_cmd.roll = roll_in;
    m_pub.publish(m_high_cmd);
  }

  void pitchCtrl(float pitch_in = 0.3f) {
    ROS_INFO("pitchCtrl");
    this->sendInitialCmd();

    m_high_cmd.mode = 1;
    m_high_cmd.pitch = pitch_in;
    m_pub.publish(m_high_cmd);
  }

  void yawCtrl(float yaw_in = 0.3f) {
    ROS_INFO("yawCtrl");
    this->sendInitialCmd();

    m_high_cmd.mode = 1;
    m_high_cmd.yaw = yaw_in;
    m_pub.publish(m_high_cmd);
  }

  void bodyHeightCtrl(float bodyHeight_in = 0.3f) {
    m_high_cmd.mode = 1;
    m_high_cmd.bodyHeight = bodyHeight_in;
    m_pub.publish(m_high_cmd);
  }

  void forwardSpeedCtrl(float forwardSpeed_in = 0.1f) {
    m_high_cmd.mode = 2;
    m_high_cmd.forwardSpeed = forwardSpeed_in;
    m_pub.publish(m_high_cmd);
  }

  void rotateSpeedCtrl(float rotateSpeed_in = 0.1f) {
    m_high_cmd.mode = 2;
    m_high_cmd.rotateSpeed = rotateSpeed_in;
    m_pub.publish(m_high_cmd);
  }
};

int main(int argv, char **argc) {

  ros::init(argv, argc, "basic_walk_controller");

  MyA1 my_a1("my_a1");
  my_a1.sendInitialCmd();
  ROS_INFO("Initialized A1 Robot");

  auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();

  std::chrono::duration<double> time_duration = now - start;

  while (ros::ok()) {
    if (time_duration.count() < 5.0)
      my_a1.rollCtrl();

    if (time_duration.count() > 5.0 && time_duration.count() < 10.0)
      my_a1.pitchCtrl();

    if (time_duration.count() > 10.0 && time_duration.count() < 15.0)
      my_a1.yawCtrl();

    now = std::chrono::steady_clock::now();
    time_duration = now - start;

    ros::spinOnce();
  }

  my_a1.sendInitialCmd();

  ros::spinOnce();
  ROS_INFO("Node Destroyed");

  ros::shutdown();

  return 0;
}