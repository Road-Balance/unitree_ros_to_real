#include <ros/ros.h>
#include <string.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

inline bool isTrue(const int &val_in)
{
  return (val_in == 1) ? true : false;
}

class JoyToCmd
{
public:
  struct XMode
  {
    float left_updown;
    float left_leftright;

    float right_updown;
    float right_leftright;

    bool btn_a;
    bool btn_b;
    bool btn_x;
    bool btn_y;

    bool btn_LB;
    bool btn_RB;

    bool btn_back;
    bool btn_start;
  };

private:
  ros::Publisher cmd_vel_pub;
  ros::Publisher a1_mode_pub;
  ros::Subscriber joy_sub;

  geometry_msgs::Twist twist;
  std_msgs::UInt8 a1_mode;

  XMode joy_keys;

public:
  JoyToCmd(ros::NodeHandle *nh)
  {
    cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    a1_mode_pub = nh->advertise<std_msgs::UInt8>("/a1_mode", 5);
    joy_sub = nh->subscribe("joy", 1, &JoyToCmd::subCallback, this);

    a1_mode.data = 1;
  }

  void subCallback(const sensor_msgs::Joy &data)
  {
    // Assume JoyStick is on "X" mode
    joy_keys.left_updown = data.axes[1];
    joy_keys.left_leftright = data.axes[0];
    joy_keys.right_updown = data.axes[4];
    joy_keys.right_leftright = data.axes[3];

    joy_keys.btn_a = isTrue(data.buttons[0]);
    joy_keys.btn_b = isTrue(data.buttons[1]);
    joy_keys.btn_x = isTrue(data.buttons[2]);
    joy_keys.btn_y = isTrue(data.buttons[3]);

    joy_keys.btn_LB = isTrue(data.buttons[4]);
    joy_keys.btn_RB = isTrue(data.buttons[5]);

    joy_keys.btn_back = isTrue(data.buttons[6]);
    joy_keys.btn_start = isTrue(data.buttons[7]);

    calc_cmd_vel();
    pub_mode();
  }

  void calc_cmd_vel()
  {
    twist.linear.x = joy_keys.left_updown;
    twist.linear.y = joy_keys.left_leftright;
    twist.linear.z = 0.0;

    // TODO: Orientation
    twist.angular.z = joy_keys.right_leftright * 2;

    cmd_vel_pub.publish(twist);
  }
  
  void pub_mode(){
    if (joy_keys.btn_x)
      a1_mode.data = 1;
    if (joy_keys.btn_a)
      a1_mode.data = 2;
    if (joy_keys.btn_b)
      a1_mode.data = 3;
    if (joy_keys.btn_y)
      a1_mode.data = 4;

    a1_mode_pub.publish(a1_mode);
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joy_to_cmd_vel");

  ros::NodeHandle nh;
  JoyToCmd joy_to_cmd_node{&nh};

  ros::spin();
}