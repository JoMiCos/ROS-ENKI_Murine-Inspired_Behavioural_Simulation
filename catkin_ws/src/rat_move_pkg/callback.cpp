#include "callback.h"

void rewardCallback(const std_msgs::Bool::ConstPtr& msg)
{
	bool reward = msg->data;
}

void placeCallback(const std_msgs::Bool::ConstPtr& msg, int index)
{
	bool placefield = msg->data;
}

void seeLandmark(const enki_ros_pck::Sight::ConstPtr& msg, int index)
{
	bool sight = msg->sight;
	float distance = msg->distance; 
}

void 

