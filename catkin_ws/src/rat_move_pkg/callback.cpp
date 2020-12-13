#include "callback.h"

void rewardCallback(const std_msgs::Float32::ConstPtr& msg)
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
	bool blue_sight = msg->sight;
	float blue_distance = msg->distance; 
}

void seeGreenLandmarkCallback(const enki_ros_pck::Sight::ConstPtr& msg)
{
	bool green_sight = msg->sight;
	float green_distance = msg->distance; 
}

//void 

