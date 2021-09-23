/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "convert.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <string>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);

template <typename TLCM> void *update_loop(void *param) {
  TLCM *data = (TLCM *)param;
  while (ros::ok) {
    data->Recv();
    usleep(2000);
  }
}

class A1LCMInterface {
private:
  long motiontime = 0;
  std::string m_name;

  // send to robot lcm
  UNITREE_LEGGED_SDK::HighCmd SendHighLCM = {0};
  unitree_legged_msgs::HighCmd SendHighROS;

  // receive from robot lcm
  UNITREE_LEGGED_SDK::HighState RecvHighLCM = {0};
  unitree_legged_msgs::HighState RecvHighROS;

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;

public:
  A1LCMInterface(const std::string &name_in) : m_name(name_in) {
    ROS_INFO("Publisher and Subscriber initialized");

    m_pub = m_nh.advertise<unitree_legged_msgs::HighState>("/a1_high_state", 5);
    m_sub =
        m_nh.subscribe("/a1_high_cmd", 1, &A1LCMInterface::subCallback, this);
  }

  ~A1LCMInterface() {}

  void subCallback(const unitree_legged_msgs::HighCmd &data) {
    std::cout << data.roll << std::endl;
    std::cout << data.pitch << std::endl;
    std::cout << data.yaw << std::endl;

    SendHighROS.mode = data.mode;
    SendHighROS.roll = data.roll;
    SendHighROS.pitch = data.pitch;
    SendHighROS.yaw = data.yaw;

    // if (data.mode == 2)
    //   SendHighROS.mode = 2;

    SendHighROS.forwardSpeed = data.forwardSpeed;
    SendHighROS.sideSpeed = data.sideSpeed;
    SendHighROS.rotateSpeed = data.rotateSpeed;
    SendHighROS.bodyHeight = data.bodyHeight;

    // SendHighROS.forwardSpeed = data.forwardSpeed;
    // SendHighROS.sideSpeed = data.sideSpeed;
    // SendHighROS.rotateSpeed = data.rotateSpeed;
    // SendHighROS.bodyHeight = data.bodyHeight;

    printf("==========================\n");
    printf("%f\n", SendHighROS.forwardSpeed);
    printf("%f\n", SendHighROS.rotateSpeed);

    SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
    roslcm.Send(SendHighLCM);

    // Pub to LCM Done
    // Next is Sub from LCM and pub to other Node
    roslcm.Get(RecvHighLCM);
    RecvHighROS = ToRos(RecvHighLCM);
    // printf("%f\n", RecvHighROS.forwardSpeed);

    m_pub.publish(RecvHighROS);
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "walk_ros_mode");
  std::string firmwork;
  ros::param::get("/firmwork", firmwork);

  std::string robot_name;
  UNITREE_LEGGED_SDK::LeggedType rname;
  ros::param::get("/robot_name", robot_name);
  if (strcasecmp(robot_name.c_str(), "A1") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
  else if (strcasecmp(robot_name.c_str(), "Aliengo") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

  A1LCMInterface a1_lcm_interface("rb_a1");

  // UNITREE_LEGGED_SDK::InitEnvironment();
  // UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
  std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
            << "Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  roslcm.SubscribeState();

  pthread_t tid;
  pthread_create(&tid, NULL, update_loop<UNITREE_LEGGED_SDK::LCM>, &roslcm);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}