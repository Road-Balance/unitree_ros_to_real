#include <ros/ros.h>
#include <string.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

inline bool isTrue(const int& val_in){
	return (val_in == 1) ? true:false;
}

class JoyToCmd {
public:
	struct XMode{
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
  ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub;
  ros::Subscriber joy_sub;

	geometry_msgs::Twist twist;
	XMode joy_keys;
public:
	JoyToCmd(ros::NodeHandle *nh){
		cmd_vel_pub	= nh->advertise<geometry_msgs::Twist>("/a1_high_cmd", 5);
		joy_sub = nh->subscribe("joy", 1, &JoyToCmd::subCallback, this);
	}

  void subCallback(const sensor_msgs::Joy &data) {
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
	}

	void calc_cmd_vel(){
		twist.linear.x = joy_keys.left_updown;
		twist.linear.y = joy_keys.left_leftright;
		twist.linear.z = 0.0;

		// TODO: Orientation
		twist.angular.z = joy_keys.right_leftright * 2;

		cmd_vel_pub.publish(twist);
	}
};

int main(int argc, char** argv){
	
	ros::init(argc, argv, "joy_to_cmd_vel");

	ros::NodeHandle nh;
	JoyToCmd joy_to_cmd_node {&nh};

	ros::spin();
}