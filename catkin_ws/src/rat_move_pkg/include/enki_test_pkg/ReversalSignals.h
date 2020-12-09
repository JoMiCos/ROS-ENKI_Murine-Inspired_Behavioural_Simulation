#ifndef __REVERSAL_SIGNALS_H
#define __REVERSAL_SIGNALS_H

#include "ros/ros.h" 
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <ros/package.h>

#include <Enki.h>
#include <QApplication>
#include <QtGui>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include "Racer.h"
#include "ReversalSignals.h"

#include "bandpass.h"
#include "parameters.h"
#include <chrono>
#include "PhysicalEngine.h"

void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Bool& reward);

#endif