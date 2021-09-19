#include <ros/ros.h>
#include <string>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

class TestClass {
  std::string name{"test class"};

public:
  void callFloat(const float &sth) {
    std::cout << "called : " << sth << std::endl;
  }

  TestClass(const std::string &str_in = "default name ") : name(str_in) {}
  ~TestClass() {}
};

TestClass my_class("test");
int a = 1;

void sub_callback(const unitree_legged_msgs::HighCmd &data) {
  std::cout << "Front Lidar Value : " << data.pitch << std::endl;
  my_class.callFloat(data.roll);
  std::cout << "a : " << a << std::endl;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "sub_test_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/a1_high_cmd", 1, sub_callback);

  ros::spin();

  return 0;
}