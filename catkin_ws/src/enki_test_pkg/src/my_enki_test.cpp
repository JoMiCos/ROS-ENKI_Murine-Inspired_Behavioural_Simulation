
//#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"

static int sensor_values[9];
static int sensor_values_stuck;
static int count; 
static bool flag = false;


void callback(const sensor_msgs::Image::ConstPtr& msg, int index){
	sensor_values[index] = msg->data[(81*1)+index*9+5];
	//ROS_INFO("%s", "-----------");
	//ROS_INFO("%d", index);
	//ROS_INFO("%d", sensor_values[index]);
		//81*1 shift to red data (from rgba)
		//index*9 to get desired row
		//+5 to get to the middle of the row
	count++;
	if(count==9){
		count=0;
		flag = true;
	}
}
void callback0(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 0);}
void callback1(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 1);}
void callback2(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 2);}
void callback3(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 3);}
void callback4(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 4);}
void callback5(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 5);}
void callback6(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 6);}
void callback7(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 7);}
void callback8(const sensor_msgs::Image::ConstPtr& msg){callback(msg, 8);}

void callbackstuck(const sensor_msgs::Image::ConstPtr& msg){
	sensor_values_stuck = msg->data[18];
		//18 is one of the first data points that change from gray to white on rqt when near walls, so we use it to check if the robot stuck at the walls
	count++;
	if(count==1){
		count=0;
		flag = true;
	}
}


void calculateMotorSpeeds(geometry_msgs::Twist& msg){
	
	if((sensor_values[8]/255.0) - (sensor_values[2]/255.0) != 0||(sensor_values[7]/255.0) -(sensor_values[3]/255.0) != 0||(sensor_values[6]/255.0) - (sensor_values[4]/255.0) != 0){
		double error = (sensor_values[6]/255.0) + 2*(sensor_values[7]/255.0) + 3*(sensor_values[8]/255.0) - 3*(sensor_values[2]/255.0) - 2*(sensor_values[3]/255.0) - (sensor_values[4]/255.0);
		msg.angular.z = -error*0.05;
	} //compares 3rgb values from left sensor with 3rgb values from right sensor - doesnt take into account colour - wouldnt work if multiple objects introduced. (values 1 and 5 are not rgb values (should probably be 0 and 4 but counting starts in the wrong place. fix later))

	else if(sensor_values_stuck/255.0 > 0.9) { // if it is almost white (0.0-1.0 is black-white)
		msg.angular.z = 0.8;	//positive is clockwise
	}

}
/*===============================================================================================*/

int main(int argc, char **argv){

	ros::init(argc, argv, "enki_btb_react_control_node");
	ros::NodeHandle nh;
	ros::Subscriber eyes[9];
	ros::Subscriber stuck;
	
	eyes[0] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback0);
	eyes[1] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback1);
	eyes[2] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback2);
	eyes[3] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback3);
	eyes[4] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback4);
	eyes[5] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback5);
	eyes[6] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback6);
	eyes[7] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback7);
	eyes[8] = nh.subscribe("mybot/colour_camera/image_raw", 1, callback8);
	stuck = nh.subscribe("mybot/colour_camera/image_raw", 1, callbackstuck);

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("mybot/cmd_vel", 1);

	ros::Rate loop_rate(60);

	while(ros::ok()){
	
		if(flag){
			flag = false;
			geometry_msgs::Twist vel;
			calculateMotorSpeeds(vel);
			vel_pub.publish(vel);
		}
	
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}