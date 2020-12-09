#ifndef _CALLBACK_H
#define _CALLBACK_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"

void rewardCallback(const std_msgs::Bool::ConstPtr& msg);

#endif
