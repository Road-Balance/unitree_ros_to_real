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
#include <geometry_msgs/Twist.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

float linear_x = 0.0;
float linear_y = 0.0;
float angular_z = 0.0;
uint8_t mode = 1;
bool can_move = false;
bool is_build_map = false;
float yaw = 0;

void cmd_vel_cb(const geometry_msgs::Twist& msg){
    linear_x = msg.linear.x;
    linear_y = msg.linear.y;
    angular_z = msg.angular.z;
}

void mode_cb(const std_msgs::UInt8& msg){
    mode = msg.data;
}

UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

// main 함수 template
template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // ros 구문 자루와 인자 설정
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1, cmd_vel_cb);
    ros::Subscriber mode_sub = n.subscribe("/dog_mode", 1, mode_cb);
    ros::Publisher dog_imu_pub = n.advertise<sensor_msgs::Imu>("/imu_raw", 1000);
    ros::Publisher dog_can_move_pub = n.advertise<std_msgs::Bool>("/dog_can_move", 1000);

    // 与狗通信的一些设置
    // SetLevel(HIGHLEVEL);
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    while (ros::ok()){
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);

        // printf("%f\n",  RecvHighROS.forwardSpeed);
        // for(int i=0; i < 40; i++) {
        //     std::cout << i << "  RECV: " << int(RecvHighROS.wirelessRemote[i]) << std::endl;
        // }
        // 리모컨의 start 키를 통해 개의 능동 여부를 제어합니다.
        if (RecvHighROS.wirelessRemote[2]-last_start_cmd==4) {
            can_move = !can_move;
            if (can_move) {
                mode = 2;
            }
            std::cout << "!!!!! can move: " << bool(can_move) << std::endl;
        }

        // 构建并发布狗是否能动的信息到 ros topic 中
        dog_can_move.data = can_move;
        dog_can_move_pub.publish(dog_can_move);
        // std::cout << int(RecvHighROS.wirelessRemote[2])  << " mode: " << int(mode) << std::endl;
        last_start_cmd = RecvHighROS.wirelessRemote[2];
        
        // 构建机器狗 imu 的数据
        dogImu.header.frame_id = "imu_link";
        dogImu.header.stamp = ros::Time::now();
        // dog's orientation 狗的方向信息
        dogImu.orientation.w = RecvHighROS.imu.quaternion[0];
        dogImu.orientation.x = -RecvHighROS.imu.quaternion[1];
        dogImu.orientation.y = -RecvHighROS.imu.quaternion[2];
        dogImu.orientation.z = -RecvHighROS.imu.quaternion[3];
        // dogImu.orientation_covariance[0] = -1;
        // dog's angular_velocity 狗的角速度信息
        dogImu.angular_velocity.x = RecvHighROS.imu.gyroscope[0];
        dogImu.angular_velocity.y = RecvHighROS.imu.gyroscope[1];
        dogImu.angular_velocity.z = RecvHighROS.imu.gyroscope[2];
        // dog's linear_acceleration 狗的线性加速度信息
        dogImu.linear_acceleration.x = RecvHighROS.imu.accelerometer[0];
        dogImu.linear_acceleration.y = RecvHighROS.imu.accelerometer[1];
        dogImu.linear_acceleration.z = RecvHighROS.imu.accelerometer[2];
        // publish dog's imu message
        // 发布狗的 imu 信息到 ros 中，供别的节点使用
        dog_imu_pub.publish(dogImu);

        // 만약 렌더링 과정이라면, 순찰점을 저장하는 프로그램을 수행합니다.
        // 여기 삭제

        // 리모컨 상태를 판단하기 위해 이전 키 값을 저장합니다.
        last_set_cmd = RecvHighROS.wirelessRemote[3];
        // 만약 리모컨이 로봇 개가 운동할 수 있음을 나타낸다면, 개의 운동 상태를 알린다.
        if (can_move) {
            SendHighROS.mode = mode;
        } else {
            SendHighROS.mode = 1;
        }

        // 개의 운동 정보 설정
        // A1
        // SendHighROS.forwardSpeed = linear_x;
        // SendHighROS.sideSpeed = linear_y;
        // SendHighROS.rotateSpeed = angular_z;
        // SendHighROS.roll  = 0;
        // SendHighROS.pitch = 0;
        // SendHighROS.yaw = 0;

        // Go1
        SendHighROS.velocity[0] = linear_x;
        SendHighROS.velocity[1] = linear_y;
        SendHighROS.yawSpeed = angular_z;
        SendHighROS.euler[0]  = 0;
        SendHighROS.euler[1] = 0;
        SendHighROS.euler[2] = 0;


        // ros의 움직임 정보를 하위 단계까지 인코딩하는 데 필요한 형식
        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        // ros 리플레이 함수 진입
        ros::spinOnce();
        // 설정된 순환 빈도에 도달하기 위해 일정한 시간으로 일시 정지한다.
        loop_rate.sleep(); 
    }

    return 0;
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