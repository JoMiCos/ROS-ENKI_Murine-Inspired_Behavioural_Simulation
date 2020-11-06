#ifndef __REVERSAL_SIGNALS_H
#define __REVERSAL_SIGNALS_H

#include "ros/ros.h" 
#include "std_msgs/Bool.h"
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

void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Bool& reward, double maxx, double maxy);

void getDistance(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight);

void seenBool(enki_ros_pck::Sight& seen, sensor_msgs::Image msg);

void placeBool(std_msgs::Bool& placeBool, Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY);


#endif
