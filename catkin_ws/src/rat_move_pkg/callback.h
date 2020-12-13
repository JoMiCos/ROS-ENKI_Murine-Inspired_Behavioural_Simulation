#ifndef _CALLBACK_H
#define _CALLBACK_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "rat_move_pkg/Sight.h"
#include "Brain/limbic-system-model.h"
#include "std_msgs/Float32.h"

Limbic_system *limbic_system;

static float reward;
static float blue_placefield;
static float green_placefield;
static float blue_sight;
static float green_sight;
static float blue_distance;
static float green_distance;

void rewardCallback(const std_msgs::Float32::ConstPtr& msg);

void bluePlaceCallback(const std_msgs::Float32::ConstPtr& msg);

void seeBlueLandmarkCallback(const rat_move_pkg::Sight::ConstPtr& msg);

void greenPlaceCallback(const std_msgs::Float32::ConstPtr& msg);

void seeGreenLandmarkCallback(const rat_move_pkg::Sight::ConstPtr& msg);


#endif
