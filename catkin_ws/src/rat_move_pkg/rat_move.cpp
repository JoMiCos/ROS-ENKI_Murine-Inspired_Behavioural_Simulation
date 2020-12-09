//instead of passing reversal-5ht variables to this file, create appropriate subscribers in those files?... (can maybe still run in main here?)


#include "rat_move.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"

//#include <reversal-5ht/limbic-system-model/filter.h>
#include "Brain/limbic-system-model.h"
#include "Brain/robot.h"
static int sensor_values[9];
static int sensor_values_stuck;
static int count; 
static bool flag = false;
//CtxNeuron* OFCNeuron;


void callback(const sensor_msgs::Image::ConstPtr& msg, int index){
	sensor_values[index] = msg->data[(81*1)+index*9+7];//changed 7 to 5
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

//must have explore state like bernds... just copy bernds as much as possible.
//OFCNeuron = new CtxNeuron(learning_rate_OFC, learning_rate_OFC * 0.1);
//OFCNeuron->addInput(visual_direction_Green_trace);
//OFCNeuron->addInput(visual_direction_Blue_trace);
/*
Greensw=limbic_system->getGreenOutput() * 2;
// Returns CoreGreenOut from Limbic-system-model.cpp. CoreGreenOut = (mPFC_Green *core_weight_lg2lg)
	// mpfc_green = mpfcneuron->doStep(visual_reward_green + OFC, serotoninConcentration) dostep is ctxneuron dostep. visual_reward_green = _visual_reward_green = 8th input to Limbic_system::doStep in robot.cpp = visual_reward_green = bool value determined by rewardDelayGreen == REWARD_DELAY = int 10 in robot.cpp but = 1 in reversal-5ht, so =1.
		//OFC = OFCNeuron->doStep(reward, serotoninConcentration+0.1)
			//reward = reward_filter->filter(_reward)
				//_reward = 1st input to Limbic_system::doStep = reward in robot.cpp
					//reward = set to either 0 or 1 by if(world->isReward(step,xp,yp)){reward=1}. if (REWARD_DELAY > 0){if(fabs(placefield1)>0){if(rewardDelayGreen>0){visual_reward_Green=0, rewardDelayGreen--,reward = 0}}} IS THIS WHAT SWITCHES WHEN BOT HITS REWARD?
						//REWARD_DELAY = 1
						//fabs() = std::fabs - absolute value of a float
						//placefield1 = 2nd input to doStep. placefield1 = isPlacefield(0)
							//isPlacefield() < if index=1 return 1, else return 0. So, placefield1 = 0;
						//rewardDelayGreen = REWARD_DELAY = 1
						//visual_reward_green = 8th inputto doStep (limbic). 0 if REWARD_DELAY >0, if fabs(placefield1)>0, rewardDelayGreen >0. 1 if rewardDelayGreen==0(rewardDelayGreen -- happens when visual_reward_Green is set to 0 meaning it is subsequently)
		//serotoninConcentration = DRNto5HTrelease->filter(DRN);
			//DRN = (LH + OFC * 4) / (1+RMTg * shunting_inhibition_factor + DRN_SUPPRESSION) + DRN_OFFSET
				//LH = OFC
				//DRN_SUPPRESSION = 0
				//DRN_OFFSET = 0
	//core_weight_lg2g set to 1 in limbic-system-model.h. fed into weightChange(). weightChange does core_weight_lg2lg = core_weight_lg2lg + (learning_rate_core*core_plasticity*mPFC_Green) between 0 and 1.
		//learning_rate_core starts at 0.
		// core plasticity = core_DA - VTA_zero_val.
			//coreDA = VTA. VTA = (LH +VTA_baseline_activity) / (1+(RMTg + VTA_forwardinhibition->filter(OFC*0.1))*shunting_inhibition_factor) 
				//shunting inhibition factor = 200
				//LH = OFC
				//VTA_baseline_activity=0.10
				//RMTg = 0
				//VTA_forwardinhibition = new SecondOrderLowpasFilter(0.01)
			//VTA_zero_val = 0.0505
//Limbic_system::doStep()
//ctxneuron::doStep() = 
// now... where does my simulation come in. I have:
//Bools which switch when rat enters place.
//bool which switches when rat gets reward.
//bool that switches when rat can see reward.
//float to measure distance from rat to reward.
//change robot.cpp instance of reward to be set with callback?
Bluesw=limbic_system->getBlueOutput() * 2;
float exploreLeft=limbic_system->getExploreLeft(); //these should work as is? (limbic-system-model.h)
float exploreRight=limbic_system->getExploreRight();

if (Greensw > 1) {
	Greensw=1;
}

if (Bluesw > 1) {
	Bluesw=1;
}

Greendirection->doDirection(leftGreen,rightGreen,Greensw);
Bluedirection->doDirection(leftBlue,rightBlue,Bluesw);

float Greenspeed = Greendirection->getSpeed();
float Bluespeed = Bluedirection->getSpeed(); //return weighted floats for tendency to move towards green and blue

float exploreLeft=limbic_system->getExploreLeft(); //both are 0.1
float exploreRight=limbic_system->getExploreRight();

//dStep=ROBOT_SPEED*(Greenspeed+Bluespeed+exploreLeft+exploreRight)-BUMP_REVERSE_GAIN*f+BUMP_REVERSE_GAIN*b+sumStep; //turn this into something that works... ROBOT_SPEED is just 5
msg.angular.z = 1(exploreRight-exploreLeft)
*/
//float OFC = OFCNeuron->doStep(reward, serotoninConcentration+0.1);
//	if (OFC > 0.25) {
//		OFC = 0.25;
//	}


void calculateMotorSpeeds(geometry_msgs::Twist& msg){
	
	if((sensor_values[8]/255.0) - (sensor_values[2]/255.0) != 0||(sensor_values[7]/255.0) -(sensor_values[3]/255.0) != 0||(sensor_values[6]/255.0) - (sensor_values[4]/255.0) != 0){
		double error = (sensor_values[6]/255.0) + 2*(sensor_values[7]/255.0) + 3*(sensor_values[8]/255.0) - 3*(sensor_values[2]/255.0) - 2*(sensor_values[3]/255.0) - (sensor_values[4]/255.0);
		msg.angular.z = -error*0.05; //if -ve turn left when see red, if +ve turn right when see red
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