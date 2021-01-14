//INITIALISATION=============================================================================

#include "rat_move.h"
#include <unistd.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "robot.h"
#include "std_msgs/Float32.h"
#include <sys/types.h>
#include <sys/stat.h>

static int blue_sensor_values[9];
static int green_sensor_values[9];
static int reward_sensor_values[9];

static int sensor_values_stuck;
static int count; 
static bool flag = false;
static double rm = RAND_MAX;

//static float reward;
static float blue_placefield; //=1 if in blue placefield
static float green_placefield; //=1 if in green placefield
static float blue_sight; //=1 if sees blue landmark
//static float blue_distance; //normalised distance to blue landmark
static float green_sight; //=1 if sees green landmark
//static float green_distance; //normalised distance to green landmark
//static float reward_distance; 
static float reward_sight; //=1 if see reward
//static float on_contact_blue; //=1 if touch blue landmark
//static float on_contact_green; //=1 if touch green landmark
static float explore_left; //determines explore pattern of rat
static float explore_right; //determines explore pattern of rat
static float Greensw; //weight of attraction to green landmark (0-1)
static float Bluesw; //weight of attracttion to blue landmark (0-1)


//LIMBIC CALLBACKS==========================================================================

/* NOT CURRENTLY USED
void rewardSignalCallback(const std_msgs::Float32::ConstPtr& msg)
{
	float reward = msg->data;
}
*/

void bluePlaceCallback(const std_msgs::Float32::ConstPtr& msg)
{
	blue_placefield = msg->data;
}

void greenPlaceCallback(const std_msgs::Float32::ConstPtr& msg)
{
	green_placefield = msg->data;
	
}
/*NOT CURRENTLY USED BUT IS FUNCTIONAL
void seeBlueLandmarkCallback(const std_msgs::Float32::ConstPtr& msg)
{
	blue_distance = msg->data;
}

void seeGreenLandmarkCallback(const std_msgs::Float32::ConstPtr& msg) //works can be used
{
	green_distance = msg->data; 
}
*/
/*NOT USED. WOULD NEED TO BE ADAPTED TO A BLUE AND A GREEN VERSION
void rewardDistanceCallback(const std_msgs::Float32::ConstPtr& msg) //not functional. necesarry? 
{	
	
	float reward_distance = msg->data;
	
}
*/
/*NOT CURRENTLY USED BUT IS FUNCTIONAL
void onContactBlue(const std_msgs::Float32::ConstPtr& msg) //works (not used)
{
	float on_contact_blue = msg->data;
	
}

void onContactGreen(const std_msgs::Float32::ConstPtr& msg) //works (not used)
{
	float on_contact_green = msg->data;
}
*/

void greenCallback(const sensor_msgs::Image::ConstPtr& msg, int index){ //callback for green camera data
	green_sensor_values[index] = msg->data[(80*1)+index*24+1];
		//80+24n+1: colour data begins from pos 80
		// +1 shift moves to green data column 
		// +24n samples green evenly to the end of the data 
		
	count++;
	if(count==9){
		count=0;
		flag = true;
	}
}

void blueCallback(const sensor_msgs::Image::ConstPtr& msg, int index){ //callback for blue camera data
	blue_sensor_values[index] = msg->data[(80*1)+index*24+2];
	//80+24n+1: colour data begins from pos 80
	// +2 shift moves to blue data column 
	// +24n samples green evenly to the end of the data 
	count++;
	if(count==9){
		count=0;
		flag = true;
	}
}

void rewardCallback(const sensor_msgs::Image::ConstPtr& msg, int index){ //callback for red camera data
	reward_sensor_values[index] = msg->data[(80*1)+index*24+0];
	//80+24n+1: colour data begins from pos 80
	// +0 shift moves to red data column (+0 obviously arbitrary)
	// +24n samples green evenly to the end of the data 
	count++;
	if(count==9){
		count=0;
		flag = true;
	}
}

void limbicCallback(const geometry_msgs::Twist limbic) //callback for data retrieved from limbic-system model in run.cpp
{
	explore_left = limbic.angular.x;
	explore_right = limbic.angular.y;

	Greensw = limbic.linear.x;
	Bluesw = limbic.linear.y;

}

int colourCheck()//checks the colour at which the rat is looking when called
{	
	int colour_check;
	int green_sum{};
	int blue_sum{};
	int reward_sum{};
	for (int i=0;i<9;i++) //sum green visual data
	{
		green_sum = green_sum+green_sensor_values[i]; 
		
	}
	for (int i=0;i<9;i++) //sum blue visual data
	{
		blue_sum = blue_sum+blue_sensor_values[i]; 
	}
	for (int i=0;i<9;i++) //sum red visual data
	{
		reward_sum = reward_sum+reward_sensor_values[i]; 
	}
	if (reward_sum/9 >30) //if reward seen at all, reward_sum>30 as background 'grey' = (24,24,24)
	// ((24*8)+255)/9 = 49.6. Therefore reward_sum > 30 only if reward is seen (even if only seen at one datapoint)
	//checks reward first as it should overrule seeing a non-reward landmark
	{
		colour_check = 3;
		return colour_check;
	}	
	else if (green_sum/9 > 30) //functions same as reward_sum but for green landmark
	{
		colour_check = 1;
		return colour_check;
	}
	else if (blue_sum/9 >30) //same as above but for blue
	{
		colour_check = 2;
		return colour_check;
	}
	else //nothing seen
	{
		colour_check = 0;
		return colour_check;
	}
}


//CAMERA CALLBACKS ==================================================================
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

void callbackstuck(const sensor_msgs::Image::ConstPtr& msg){ //if too close to a wall, this turns the rat
	sensor_values_stuck = msg->data[18];
		//18 is one of the first data points that change from gray to white on rqt when near walls, so we use it to check if the robot stuck at the walls
	count++;
	if(count==1){
		count=0;
		flag = true;
	}
}

//MOTOR SIGNALS=========================================================================================================================================

void calculateMotorSpeedBlue(geometry_msgs::Twist& msg) //Braitenberg steering if rat decides to move to blue pellet
{ 
	double error = (blue_sensor_values[6]/255.0) + 2*(blue_sensor_values[7]/255.0) + 3*(blue_sensor_values[8]/255.0) - 3*(blue_sensor_values[0]/255.0) - 2*(blue_sensor_values[1]/255.0) - (blue_sensor_values[2]/255.0);	
	msg.angular.z = -error*0.05; //if -ve turn left, if +ve turn right 
	msg.linear.y = 1; //if non-zero, rat moves
}

void calculateMotorSpeedGreen(geometry_msgs::Twist& msg)//Braitenberg steering if rat decides to move to green pellet
{	
	double error = (green_sensor_values[6]/255.0) + 2*(green_sensor_values[7]/255.0) + 3*(green_sensor_values[8]/255.0) - 3*(green_sensor_values[0]/255.0) - 2*(green_sensor_values[1]/255.0) - (green_sensor_values[2]/255.0);
	msg.angular.z = -error*0.05; //if -ve turn left, if +ve turn right 
	msg.linear.y =1; //if non-zero, rat moves
}

void calculateMotorSpeedReward(geometry_msgs::Twist& msg)//Braitenberg steering if rat decides to move to reward (red) pellet
{	
		double error = (reward_sensor_values[6]/255.0) + 2*(reward_sensor_values[7]/255.0) + 3*(reward_sensor_values[8]/255.0) - 3*(reward_sensor_values[0]/255.0) - 2*(reward_sensor_values[1]/255.0) - (reward_sensor_values[2]/255.0);
		msg.angular.z = -error*0.05; //if -ve turn left when see red, if +ve turn right when see red
		msg.linear.y = 1; //if non-zero, rat moves
}

void ratExplore(geometry_msgs::Twist& vel)
{	
	//ROS_INFO("%f", blue_sight);
	if(sensor_values_stuck/255.0 > 0.9) 
	{ // if it is almost white (the colour of the walls) (0.0-1.0 is black-white)
		vel.angular.z = 0.7; //turn could be randomised to make look more realistic but this is functional
		vel.linear.y = 1;	
		//ROS_INFO("%s", "Wall");	
	}

	else if (green_sight == 0 && blue_sight == 0 && reward_sight ==0) //dont see landmark
	{	
		//ROS_INFO("%s", "explore");
		vel.angular.z = 0.75*(explore_right-explore_left);
		
		if (explore_left > 0 || explore_right >0)
		{
			vel.linear.y = 1;
		}
		else
		{	
			if ((blue_placefield != 0) || (green_placefield != 0))
			{
				//vel.angular.z = -0.5;
				vel.linear.y = 1;
			}
			else
			{
				//ROS_INFO("%s", "stopped");
				vel.linear.y = 0; //stop moving
			}
			
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
		
		if(reward_sight != 0) //see reward
		{
			//ROS_INFO("%s", "reward");
			vel.linear.y=1;
			calculateMotorSpeedReward(vel);
			
		}
		else if ((blue_sight != 0)) //&& (blue_distance >0.65)) <-- optional to restrict how far away rat can see blue landmark from
		{	//see blue
			//ROS_INFO("%s", "blue");
			if (Bluesw>=rand()/rm) //rm == RAND_MAX, but using RAND_MAX directly caused errors
			{
				vel.linear.y=1;
				calculateMotorSpeedBlue(vel);	
			}
			else
			{
				vel.angular.z = 0.75*(explore_right-explore_left);
				vel.linear.y = 1;
			}
			
		}
		else if((green_sight != 0)) //&& (green_distance >0.65)) <-- optional to restrict how far away rat can see green landmark from
		{	//see green
			//ROS_INFO("%s", "green");
			if (Greensw>=rand()/rm) //rm == RAND_MAX but using RAND_MAX directly caused errors
			{
				vel.linear.y=1;
				calculateMotorSpeedGreen(vel);	
			}
			else
			{
				vel.linear.y = 1;
				vel.angular.z = 0.75*(explore_right-explore_left);
			}
			
			
		}
		else //fail-safe - return to explore
		{
			//ROS_INFO("%s", "else");
			vel.angular.z = 0.75*(explore_right-explore_left);
			vel.linear.y = 1;
		}
		
	}
}
/*MAIN LOOP===============================================================================================*/

int main(int argc, char **argv){

	ros::init(argc, argv, "enki_btb_react_control_node");
	ros::NodeHandle nh;
	ros::Subscriber eyes[27];
	ros::Subscriber stuck;
	ros::Subscriber limbic_signals[2];
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
	
	
	limbic_signals[0] = nh.subscribe("mybot/inPlaceBlue",1, bluePlaceCallback);
	limbic_signals[1] = nh.subscribe("mybot/inPlaceGreen", 1, greenPlaceCallback);
	//limbic_signals[2] = nh.subscribe("mybot/isRewarded", 1, rewardSignalCallback);
	//limbic_signals[3] = nh.subscribe("mybot/distBlue", 1, seeBlueLandmarkCallback);
	//limbic_signals[4] = nh.subscribe("mybot/distGreen",1, seeGreenLandmarkCallback);
	//limbic_signals[5] = nh.subscribe("mybot/seeReward", 1, rewardDistanceCallback);
	//limbic_signals[6] = nh.subscribe("mybot/contactBlue", 1 , onContactBlue);
	//limbic_signals[7] = nh.subscribe("mybot/contactGreen", 1, onContactGreen);
	
	//ROS SUBSCRIBE
	reversal5ht = nh.subscribe("mybot/limbic",1,limbicCallback);

	//ROS PUBLISH
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("mybot/cmd_vel", 1);
	ros::Rate loop_rate(60); //twice refresh rate of ENKI loop as if ROS loop fails to update before ENKI it causes as crash
	geometry_msgs::Twist vel;
	
	while(ros::ok()){
		if(flag){
			flag = false;
			
			int colour_seen=colourCheck();
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
			case 0: //explore
				blue_sight=0;
				green_sight=0;
				reward_sight=0;
				break;
			}
		
			ratExplore(vel);	
			vel_pub.publish(vel);
		}
	
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}