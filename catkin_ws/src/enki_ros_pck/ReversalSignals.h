#ifndef __REVERSAL_SIGNALS_H
#define __REVERSAL_SIGNALS_H

#include "ros/ros.h" 
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <ros/package.h>
#include "enki_ros_pck/Sight.h"
#include "sensor_msgs/Image.h"

#include <Enki.h>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iomanip>
#include "Racer.h"
#include "Geometry.h"

#include "bandpass.h"
#include "parameters.h"
#include <chrono>
#include "PhysicalEngine.h"

enum Colours {BLANK, RED, GREEN, BLUE, PURPLE};
/*
struct Pellet
{
Enki::PhysicalObject* name;
int colour;
double x_coord;
double y_coord;
};
typedef Pellet _Pellet;

_Pellet blue_pellet;
*/

void rewardDelay(int delay);

//void setPelletColour(_Pellet _pellet);
void setPelletColour(Enki::PhysicalObject* pellet, int pellet_colour);    


//void seenBool(enki_ros_pck::Sight& seen, sensor_msgs::Image msg, _Pellet _pellet);
void seenBool(enki_ros_pck::Sight& seen, sensor_msgs::Image msg, int pellet_colour);


//void rewardBool(Enki::Racer* racer, _Pellet _pellet, Enki::PhysicalObject* pellet, std_msgs::Float32& reward, double maxx, double maxy);
//void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Float32& reward, double maxx, double maxy, bool rewardDelay);


//void getDistance(Enki::Racer* racer, _Pellet _pellet, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight);
void getDistanceGreen(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight,float maxx,float maxy);

void getDistanceBlue(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight, float maxx, float maxy);

void placeBoolGreen(std_msgs::Float32& placeBool, Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY);

void placeBoolBlue(std_msgs::Float32& placeBool, Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY);

void on_contact_direction_blue();

void on_contact_direction_green();

#endif

//reward = rewardBool
//placefieldBlue = placeBoolBlue (edit to use camera instead of area?)
//placefieldGreen = placeBoolGreen
//on_contact_direction_Green = ..
//on_contact_direction_Blue = ..
//visual_direction_g = getDistanceGreen
//visual_direction_b = getDistanceBlue
//visual_reward_g = ...
// visual_reward_b = ...

