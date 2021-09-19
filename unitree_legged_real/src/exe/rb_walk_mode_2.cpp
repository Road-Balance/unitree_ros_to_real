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

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

long motiontime = 0;
UNITREE_LEGGED_SDK::HighCmd SendHighLCM = {0};
UNITREE_LEGGED_SDK::HighState RecvHighLCM = {0};
unitree_legged_msgs::HighCmd SendHighROS;
unitree_legged_msgs::HighState RecvHighROS;

template <typename TLCM> void *update_loop(void *param) {
  TLCM *data = (TLCM *)param;
  while (ros::ok) {
    data->Recv();
    usleep(2000);
  }
}

void subCallback(const unitree_legged_msgs::HighCmd &data) {
  std::cout << data.roll << std::endl;
  std::cout << data.pitch << std::endl;
  std::cout << data.yaw << std::endl;

  SendHighROS.mode = 1;
  SendHighROS.roll = data.roll;
  SendHighROS.pitch = data.pitch;
  SendHighROS.yaw = data.yaw;

  SendHighROS.forwardSpeed = 0.0f;
  SendHighROS.sideSpeed = 0.0f;
  SendHighROS.rotateSpeed = 0.0f;
  SendHighROS.bodyHeight = 0.0f;

  SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
  roslcm.Send(SendHighLCM);
  loop_rate.sleep();
}

// UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState,
//     UNITREE_LEGGED_SDK::LCM;

int mainHelper(int argc, char *argv[], UNITREE_LEGGED_SDK::LCM &roslcm) {
  std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
            << "Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/a1_high_cmd", 1, subCallback);

  ros::Rate loop_rate(500);

  // SetLevel(HIGHLEVEL);

  roslcm.SubscribeState();

  pthread_t tid;
  pthread_create(&tid, NULL, update_loop<UNITREE_LEGGED_SDK::LCM>, &roslcm);

  while (ros::ok()) {
    roslcm.Get(RecvHighLCM);
    RecvHighROS = ToRos(RecvHighLCM);
    printf("%f\n", RecvHighROS.forwardSpeed);

    // std::cout << "================" << std::endl;
    // std::cout << "SendHighROS.mode : " << SendHighROS.mode << std::endl;
    // std::cout << "SendHighROS.roll : " << SendHighROS.roll << std::endl;
    // std::cout << "SendHighROS.pitch : " << SendHighROS.pitch << std::endl;
    // std::cout << "SendHighROS.yaw : " << SendHighROS.yaw << std::endl;
    // std::cout << "SendHighROS.forwardSpeed : " << SendHighROS.forwardSpeed
    //           << std::endl;
    // std::cout << "SendHighROS.rotateSpeed : " << SendHighROS.rotateSpeed
    //           << std::endl;
    // std::cout << std::endl;

    // SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
    // roslcm.Send(SendHighLCM);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "walk_ros_mode");
  std::string firmwork;
  ros::param::get("/firmwork", firmwork);

  // #ifdef SDK3_1
  //   aliengo::Control control(aliengo::HIGHLEVEL);
  //   aliengo::LCM roslcm;
  //   mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc,
  //   argv,
  //                                                                  roslcm);
  // #endif

  // #ifdef SDK3_2
  std::string robot_name;
  UNITREE_LEGGED_SDK::LeggedType rname;
  ros::param::get("/robot_name", robot_name);
  if (strcasecmp(robot_name.c_str(), "A1") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
  else if (strcasecmp(robot_name.c_str(), "Aliengo") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

  // UNITREE_LEGGED_SDK::InitEnvironment();
  UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
  mainHelper(argc, argv, roslcm);

  // #endif
}