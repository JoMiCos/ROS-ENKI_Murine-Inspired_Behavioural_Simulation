//instead of passing reversal-5ht variables to this file, create appropriate subscribers in those files?... (can maybe still run in main here?)
//Next, make sure all subsribed values are coming out (ros_info) and plug into doStep

#include "rat_move.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "robot.h"
#include "world.h"
#include "std_msgs/Float32.h"
//#include "callback.h"
#include <sys/types.h>
#include <sys/stat.h>
//#include "rat_move_pkg/Sight.h"
#include "enki_ros_pck/Sight.h"

//#include <reversal-5ht/limbic-system-model/filter.h>
#include "Brain/limbic-system-model.h"
#include "Brain/robot.h"
static int blue_sensor_values[9];
static int green_sensor_values[9];
static int reward_sensor_values[9];

static int sensor_values_stuck;
static int count; 
static bool flag = false;

static float reward;
static float blue_placefield;
static float green_placefield;
static float blue_sight;
static float blue_distance;
static float green_sight;
static float green_distance;
static float reward_distance;
static float reward_sight;
static float on_contact_blue;
static float on_contact_green;
static float explore_left;
static float explore_right;
static float Greensw;
static float Bluesw;

static double rm = RAND_MAX;

Limbic_system limbic_system;

void rewardSignalCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float reward = msg->data;
}

void bluePlaceCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float blue_placefield = msg->data;
}

void greenPlaceCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float green_placefield = msg->data;
}

void seeBlueLandmarkCallback(const enki_ros_pck::Sight::ConstPtr& msg)
{
	float blue_sight = msg->sight;
	float blue_distance = msg->distance; 
}

void seeGreenLandmarkCallback(const enki_ros_pck::Sight::ConstPtr& msg)
{
	float green_sight = msg->sight;
	float green_distance = msg->distance; 
}

void rewardDistanceCallback(const enki_ros_pck::Sight::ConstPtr& msg)
{
	float reward_distance = msg->distance;
	float reward_seen = msg->sight;
}

void onContactBlue(const std_msgs::Float32::ConstPtr& msg)
{
	float on_contact_blue = msg->data;
}

void onContactGreen(const std_msgs::Float32::ConstPtr& msg)
{
	float on_contact_green = msg->data;
}

void greenCallback(const sensor_msgs::Image::ConstPtr& msg, int index){
	green_sensor_values[index] = msg->data[(80*1)+index*24+1];
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

void blueCallback(const sensor_msgs::Image::ConstPtr& msg, int index){
	blue_sensor_values[index] = msg->data[(80*1)+index*24+2];
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

void rewardCallback(const sensor_msgs::Image::ConstPtr& msg, int index){
	reward_sensor_values[index] = msg->data[(80*1)+index*24+0];
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

void limbicCallback(const geometry_msgs::Twist limbic)
{
	explore_left = limbic.angular.x;
	explore_right = limbic.angular.y;

	Greensw = limbic.linear.x;
	Bluesw = limbic.linear.y;

}

int colourCheck()//const sensor_msgs::Image::ConstPtr& msg)
{	
	int colour_check;
	int green_sum{};
	int blue_sum{};
	int reward_sum{};
	for (int i=0;i<9;i++)
	{
		green_sum = green_sum+green_sensor_values[i]; 
		
	}
	for (int i=0;i<9;i++)
	{
		blue_sum = blue_sum+blue_sensor_values[i]; 
	}
	for (int i=0;i<9;i++)
	{
		reward_sum = reward_sum+reward_sensor_values[i]; 
	}
	if (reward_sum/9 >30)
	{
		colour_check = 3;
		return colour_check;
	}	
	else if (green_sum/9 > 30) //225 -> 8/9*255 to allow for errors
	{
		colour_check = 1;
		return colour_check;
	}
	else if (blue_sum/9 >30)
	{
		colour_check = 2;
		return colour_check;
	}
	else 
	{
		colour_check = 0;
		return colour_check;
	}
}



void green_callback0(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 0);}
void green_callback1(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 1);}
void green_callback2(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 2);}
void green_callback3(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 3);}
void green_callback4(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 4);}
void green_callback5(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 5);}
void green_callback6(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 6);}
void green_callback7(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 7);}
void green_callback8(const sensor_msgs::Image::ConstPtr& msg){greenCallback(msg, 8);}

void blue_callback0(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 0);}
void blue_callback1(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 1);}
void blue_callback2(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 2);}
void blue_callback3(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 3);}
void blue_callback4(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 4);}
void blue_callback5(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 5);}
void blue_callback6(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 6);}
void blue_callback7(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 7);}
void blue_callback8(const sensor_msgs::Image::ConstPtr& msg){blueCallback(msg, 8);}

void reward_callback0(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 0);}
void reward_callback1(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 1);}
void reward_callback2(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 2);}
void reward_callback3(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 3);}
void reward_callback4(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 4);}
void reward_callback5(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 5);}
void reward_callback6(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 6);}
void reward_callback7(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 7);}
void reward_callback8(const sensor_msgs::Image::ConstPtr& msg){rewardCallback(msg, 8);}


void callbackstuck(const sensor_msgs::Image::ConstPtr& msg){
	sensor_values_stuck = msg->data[18];
		//18 is one of the first data points that change from gray to white on rqt when near walls, so we use it to check if the robot stuck at the walls
	count++;
	if(count==1){
		count=0;
		flag = true;
	}
}


//if doesnt see pellet, should be in expore state, using getExploreLeft() and getExploreRight()
void calculateMotorSpeedBlue(geometry_msgs::Twist& msg){
		double error = (blue_sensor_values[6]/255.0) + 2*(blue_sensor_values[7]/255.0) + 3*(blue_sensor_values[8]/255.0) - 3*(blue_sensor_values[0]/255.0) - 2*(blue_sensor_values[1]/255.0) - (blue_sensor_values[2]/255.0);
		
		//ROS_INFO("%f", error);
		msg.angular.z = -error*0.05; //if -ve turn left when see red, if +ve turn right when see red
		msg.linear.y = 1;
	 //compares 3rgb values from left sensor with 3rgb values from right sensor - doesnt take into account colour - wouldnt work if multiple objects introduced. (values 1 and 5 are not rgb values (should probably be 0 and 4 but counting starts in the wrong place. fix later))


}

void calculateMotorSpeedGreen(geometry_msgs::Twist& msg){
	
	double error = (green_sensor_values[6]/255.0) + 2*(green_sensor_values[7]/255.0) + 3*(green_sensor_values[8]/255.0) - 3*(green_sensor_values[0]/255.0) - 2*(green_sensor_values[1]/255.0) - (green_sensor_values[2]/255.0);
		msg.angular.z = -error*0.05;
}

void calculateMotorSpeedReward(geometry_msgs::Twist& msg){
	
		//double error = (reward_sensor_values[6]/255.0) + 2*(reward_sensor_values[7]/255.0) + 3*(reward_sensor_values[8]/255.0) - 3*(reward_sensor_values[2]/255.0) - 2*(reward_sensor_values[3]/255.0) - (reward_sensor_values[4]/255.0);
		double error = (reward_sensor_values[6]/255.0) + 2*(reward_sensor_values[7]/255.0) + 3*(reward_sensor_values[8]/255.0) - 3*(reward_sensor_values[0]/255.0) - 2*(reward_sensor_values[1]/255.0) - (reward_sensor_values[2]/255.0);
		
		msg.angular.z = -error*0.05; //if -ve turn left when see red, if +ve turn right when see red
}

void ratExplore(geometry_msgs::Twist& vel)
{
	
	ROS_INFO("%s", "-------------");
	//ROS_INFO("%f", explore_left);
	//ROS_INFO("%f", explore_right);
	//ROS_INFO("%f", Bluesw);
	//ROS_INFO("%f", Greensw);
	//ROS_INFO("%f", reward_sight);
	ROS_INFO("%d", blue_sensor_values[0]);
	ROS_INFO("%d", blue_sensor_values[1]);
	ROS_INFO("%d", blue_sensor_values[2]);
	ROS_INFO("%d", blue_sensor_values[3]);
	ROS_INFO("%d", blue_sensor_values[4]);
	ROS_INFO("%d", blue_sensor_values[5]);
	ROS_INFO("%d", blue_sensor_values[6]);
	ROS_INFO("%d", blue_sensor_values[7]);
	ROS_INFO("%d", blue_sensor_values[8]);

	
	if(sensor_values_stuck/255.0 > 0.9) 
		{ // if it is almost white (0.0-1.0 is black-white)
			vel.angular.z = 0.7;
			vel.linear.y = 1;		//positive is clockwise
		}

	else if (green_sight == 0 && blue_sight == 0 && reward_sight ==0) //dont see landmark
	{
		//ROS_INFO("%s", "dont see anything");
		vel.angular.z = 10*(explore_right-explore_left);//can make random amount of turn by multiplying both my rand.
		
		if (explore_left > 0 || explore_right >0)
		{
			vel.linear.y = 1;
		}
		else
		{
			vel.linear.y = 1; //should be 0? want to stop until reward appears
		}
	}

	else //see landmark
	{
		if (Greensw > 1) {
			Greensw=1;
		}
		if (Bluesw > 1) {
			Bluesw=1;
		}
		//green/blue output increases as tendency to move to that colour does, so code that...(up to 1)
		
		if(reward_sight != 0)
		{
			//ROS_INFO("%s", "see reward");
			calculateMotorSpeedReward(vel);
			
		}
		else if (blue_sight != 0)
		{	
			if (Bluesw>=rand()/rm)
			{
				//ROS_INFO("%s", "see blue");
				calculateMotorSpeedBlue(vel);	
			}
		
			else
			{
				vel.angular.z = 10*(explore_right-explore_left);
				vel.linear.y = 1;
			}
			
		}
		else if(green_sight != 0)
		{
			//ROS_INFO("%s", "see green");
			if (Greensw>=rand()/rm)
			{
				calculateMotorSpeedGreen(vel);	
			}
		
			else
			{
				vel.angular.z = 10*(explore_right-explore_left);
			}
		}
		
	}
}

/*===============================================================================================*/

int main(int argc, char **argv){

	ros::init(argc, argv, "enki_btb_react_control_node");
	ros::NodeHandle nh;
	ros::Subscriber eyes[27];
	ros::Subscriber stuck;
	ros::Subscriber limbic_signals[9];
	ros::Subscriber reversal5ht;
	
	eyes[0] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback0);
	eyes[1] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback1);
	eyes[2] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback2);
	eyes[3] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback3);
	eyes[4] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback4);
	eyes[5] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback5);
	eyes[6] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback6);
	eyes[7] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback7);
	eyes[8] = nh.subscribe("mybot/colour_camera/image_raw", 1, green_callback8);
	stuck = nh.subscribe("mybot/colour_camera/image_raw", 1, callbackstuck);

	eyes[9] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback0);
	eyes[10] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback1);
	eyes[11] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback2);
	eyes[12] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback3);
	eyes[13] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback4);
	eyes[14] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback5);
	eyes[15] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback6);
	eyes[16] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback7);
	eyes[17] = nh.subscribe("mybot/colour_camera/image_raw", 1, blue_callback8);
	
	eyes[18] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback0);
	eyes[19] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback1);
	eyes[20] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback2);
	eyes[21] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback3);
	eyes[22] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback4);
	eyes[23] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback5);
	eyes[24] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback6);
	eyes[25] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback7);
	eyes[26] = nh.subscribe("mybot/colour_camera/image_raw", 1, reward_callback8);
	
	limbic_signals[0] = nh.subscribe("mybot/isRewarded", 1, rewardSignalCallback);
	limbic_signals[1] = nh.subscribe("mybot/seeBlue", 1, seeBlueLandmarkCallback);
	limbic_signals[2] = nh.subscribe("mybot/seeGreen",1, seeGreenLandmarkCallback);
	limbic_signals[3] = nh.subscribe("mybot/seeReward", 1, rewardDistanceCallback);
	limbic_signals[4] = nh.subscribe("mybot/inPlaceBlue",1, bluePlaceCallback);
	limbic_signals[5] = nh.subscribe("mybot/inPlaceGreen", 1, greenPlaceCallback);
	limbic_signals[6] = nh.subscribe("mybot/contactBlue", 1 , onContactBlue);
	limbic_signals[7] = nh.subscribe("mybot/contactGreen", 1, onContactGreen);
	
	reversal5ht = nh.subscribe("mybot/limbic",1,limbicCallback);
	/*
	ROS_INFO("%s", "reward: ");
	ROS_INFO("%f", reward);
	ROS_INFO("%s", "blue_placefield: ");
	ROS_INFO("%f", blue_placefield);
	ROS_INFO("%s", "green_placefield");
	ROS_INFO("%f", green_placefield);
	ROS_INFO("%s", "blue_sight");
	ROS_INFO("%f", blue_sight);
	ROS_INFO("%s", "blue_distance");
	ROS_INFO("%f", blue_distance);
	ROS_INFO("%s", "green_sight");
	ROS_INFO("%f", green_sight);
	ROS_INFO("%s", "green_distance");
	ROS_INFO("%f", green_distance);
	ROS_INFO("%s", "reward_distance");
	ROS_INFO("%f", reward_distance);
	ROS_INFO("%s", "reward_seen");
	ROS_INFO("%f", reward_seen);
	ROS_INFO("%s", "on_contact_blue");
	ROS_INFO("%f", on_contact_blue);
	ROS_INFO("%s", "on_contact_green");
	ROS_INFO("%f", on_contact_green);
	*/
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("mybot/cmd_vel", 1);

	ros::Rate loop_rate(60);

	geometry_msgs::Twist vel;
	const sensor_msgs::Image bmsg;
	const sensor_msgs::Image gmsg;
	while(ros::ok()){
		if(flag){
			flag = false;
			
			int colour_seen=colourCheck();
			//ROS_INFO("%d", colour_seen);
			switch(colour_seen){
			case 1: //green
				green_sight=1;
				blue_sight=0;
				reward_sight=0;
				break;
			case 2: //blue
				blue_sight=1;
				green_sight=0;
				reward_sight=0;
				break;
			case 3: //reward
				blue_sight=0;
				green_sight=0;
				reward_sight=1;
				break;	
			case 0: //exlpore
				blue_sight=0;
				green_sight=0;
				reward_sight=0;
				break;
			}
			//ROS_INFO("%f", green_sight);
			ratExplore(vel);
				
			vel_pub.publish(vel);
		}
	
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}