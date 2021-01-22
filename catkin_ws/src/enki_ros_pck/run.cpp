/*
    MINE

    Enki - a fast 2D robot simulator
    Copyright (C) 2017 Bernd Porr <mail@berndporr.me.uk>
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication 
    arising from research using this software are asked to add the 
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
//#include <../../enki/Enki.h>
#include "ros/ros.h" //these have to go first for some reason...
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
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
#include <stdlib.h>
#include "Brain/limbic-system-model.h"
#include "bandpass.h"
#include "parameters.h"
#include <chrono>
#include "PhysicalEngine.h"
#include "Limbic-object.h"

#define reflex

using namespace Enki;
using namespace std;
using namespace std::chrono;

//signals passed to limbic-system::doStep()
static float reward; //from 0 to 1 when rat touches reward pellet
static float contactBlue; //0 to 1 when rat touches blue landmark, not in reward state
static float contactGreen; //0 to 1 when rat touches green landmark, not in reward state
static float green_reward_distance; //normalised distance from edge of green placefield to green reward (when rat is in placefield only)
static float blue_reward_distance; //normalised distance from edge of blue placefield to blue reward (when rat is in placefield only)
static float green_distance; //normalised distance from rat to green landmark (with respect to entire playground)
static float blue_distance; //normalised distance from rat to blue landmark (with respect to entire playground)
static float green_place; //0 to 1 if rat is in green placefield
static float blue_place; //0 to 1 if rat is in blue placefield
uint32_t step=0; //step counter

//explore signals published to rostopic mybot/limbic and used in rat_move.cpp
static float explore_left;
static float explore_right;

static bool rewardSet; //used to switch colour of reward

//dimensions of playground
static double	maxx = 250;
static double	maxy = 250;

//rat starting coordinates
static double   racerx = (maxx/2)+30;
static double   racery = (maxy/2)-50;

//Reward setting
static bool reward_flagG = 0; //if 1, turns green landmark to reward
static bool reward_flagB = 0; //if 1, turns blue landmark to reward

//Delay timer
static bool start_timer{0};
static auto start = system_clock::now();
static seconds delay(0); //used to determine delay between rat entering placefield and reward being revealed
uint8_t rewardcount{0};

//Toggle reward switch after time-steps or rewards received
uint8_t toggle_n=1;
//static int switch_toggle = STEPS_SWITCH; //starts reversal after steps_before_switch number of time steps (30Hz)
static int switch_toggle = REWARDS_SWITCH; //starts reversal after rewards_before_switch number of correct choices
static uint8_t rewards_before_switch = 10; 
static uint32_t steps_before_switch = 12500; //30 steps per second

static bool limbic_system_inst = 1;

int countSteps=0;
int countRuns=0;
int learningRateCount=0;
int firstStep =1; //so that it does propInputs once and then back/forth in that order
int nInputs= ROW1N+ROW2N+ROW3N; //this cannot be an odd number for icoLearner



//SIGNAL SETTTING FUNCTIONS

void rewardDelayB(seconds Delay){ //set blue landmark to reward after Delay
    if (start_timer = 0){
        start = system_clock::now();
        start_timer = 1;
    }
    if ((start_timer = 1) && (system_clock::now() - start) >= (delay)){
        reward_flagB=1;
        start_timer=0;
    }
}
void rewardDelayG(seconds Delay){ //set green landmark to reward after Delay
    if (start_timer = 0){
        start = system_clock::now();
        start_timer = 1;
    }
    if ((start_timer = 1) && (system_clock::now() - start) >= (delay)){
  
        reward_flagG=1;
        start_timer=0;
   
    }
}

float rewardBoolB(Enki::Racer* racer, Enki::PhysicalObject* pellet, double racerx, double racery, double circleCentreX, double circleCentreY, double circleRad) // sets blue reward
{
    if (blue_place > 0){
    rewardDelayB(delay);
        if (reward_flagB){
            pellet->setColor(Enki::Color(1,0,0)); //red - Enki::Colour(R,G,B) 0-255 normalised
            if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<14.0)
            {	//robot half Flength + food radius = 10 +3 = 13, 14 prevents collision, looks smoother
                racer->pos = Point(155, 75); //magic numbers for reset coords because for some reason using variables causes a bug where the rat respawns in the top right corner
                reward = 1; 
                pellet->setColor(Enki::Color(0,0,1)); //blue
                reward_flagB=0;
                rewardcount++;                
            }
            else
            {
                reward = 0;
            }
        }
    }
    else
    {   
        reward_flagB =0;
        pellet->setColor(Enki::Color(0,0,1)); //blue
        reward = 0;
    }
    
    return reward;
}

float rewardBoolG(Enki::Racer* racer, Enki::PhysicalObject* pellet, double racerx, double racery, double circleCentreX, double circleCentreY, double circleRad) // set green reward
{
    if (green_place > 0){
        rewardDelayG(delay);
        if (reward_flagG){
            
            pellet->setColor(Enki::Color(1,0,0));  //red: Enki::Colour(R,G,B) 0-255 normalised
            if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<14.0)
            {	//robot half length + food radius = 10 +3 = 13, 14 prevents collision, looks smoother
                racer->pos = Point(155, 75);// racerx, racery - when non-magic numbers used causes glitch where robot spawns in top right corner
                reward = 1; 
                pellet->setColor(Enki::Color(0,1,0)); //green
                reward_flagG=0;
                rewardcount++;
                
            }
            else
            {
                reward = 0;
            }
    
        }    
    }
    else
    {
        reward_flagG=0;
        pellet->setColor(Enki::Color(0,1,0)); //green
        reward = 0;
    }
    return reward;
}

float on_contact_direction_blue(Enki::Racer* racer, Enki::PhysicalObject* pellet) //detects if rat hits blue landmark
{
    if (reward_flagB==0){ //only triggers if reward not active
        if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<14.0)
        {	//robot half length + food radius = 10 +3 = 13, 14 prevents collision, looks smoother
            contactBlue = 1;
            racer->pos = Point(racerx, racery);
        }
        else
        {
            contactBlue = 0;
        }
    }
    return contactBlue;
}

float on_contact_direction_green(Enki::Racer* racer, Enki::PhysicalObject* pellet) //detects if rat hits green landmark
{
    if (reward_flagG==0){ // only triggers if reward not active
        if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<14.0)
        {	//robot half length + food radius = 10 +3 = 13, 14 prevents collision, looks smoother
            contactGreen = 1;
            racer->pos = Point(racerx, racery);
                
        }
        else
        {
            contactGreen = 0;
        }
    }
    return contactGreen;
}

float getDistanceRewardGreen(Enki::Racer* racer, Enki::PhysicalObject* pellet, float centrex, float centrey, float pfradius) //normalised distance from rat to reward. edge of placefield= 0, reward =1
{
    if (reward_flagG == 1) //only when reward revealed
    {
        float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
        float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    
        float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot + reward radius
        float normalised_distance = (1-(direct_distance/pfradius));
        if(normalised_distance > 1){normalised_distance=1.0;} //can happen if at an angle very close
        if(normalised_distance < 0){normalised_distance=0.0;}
        green_reward_distance = normalised_distance;
    }
    else
    {
        green_reward_distance = 0;
    }
    return green_reward_distance;
}

float getDistanceRewardBlue(Enki::Racer* racer, Enki::PhysicalObject* pellet, float centrex, float centrey, float pfradius)//normalised distance from rat to reward. edge of placefield= 0, reward =1
{
    if (reward_flagB == 1)
    {
        float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
        float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    
        float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
        float normalised_distance = (1-(direct_distance/pfradius));
        if(normalised_distance > 1){normalised_distance=1.0;} //can happen if at an angle very close
        if(normalised_distance < 0){normalised_distance=0.0;} 
        blue_reward_distance = normalised_distance;
    }
    else
    {
        blue_reward_distance = 0;
    }
return blue_reward_distance;
}   

float getDistanceGreen(Enki::Racer* racer, Enki::PhysicalObject* pellet, float maxx, float maxy)//normalised distance from rat to green landmark.
{
    float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
    float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    
    float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
    float max_distance = sqrt((maxx*maxx)+(maxy*maxy))-13;
    float normalised_distance = (1-(direct_distance/max_distance));
    if(normalised_distance > 1){normalised_distance=1.0;}
    green_distance = normalised_distance;

    return green_distance;
}

float getDistanceBlue(Enki::Racer* racer, Enki::PhysicalObject* pellet, float maxx, float maxy)//normalised distance from rat to blue landmark
{
    float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
    float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    
    float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
    float max_distance = sqrt((maxx*maxx)+(maxy*maxy))-13;
    float normalised_distance = (1-(direct_distance/max_distance));
    if(normalised_distance > 1){normalised_distance=1.0;}
    blue_distance = normalised_distance;

    return blue_distance;
}

float placeBoolGreen( Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY) //1 if in green placefield (approx) maybe change to using ground sensors if higher accuracy needed
{
    if ((racer->pos.x >= (circleCentreX - circleRad)) && (racer->pos.x <= (circleCentreX + circleRad)) && (racer->pos.y <= (circleCentreY + circleRad)) && (racer->pos.y >= (circleCentreY - circleRad))) //Makes a square instead of a circle
    {
        green_place = 1.0;
    }
    else
    {
        green_place = 0.0;
    }
    return green_place;
}

float placeBoolBlue(Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY) //1 if in blue placefield
{
    if ((racer->pos.x >= (circleCentreX - circleRad)) && (racer->pos.x <= (circleCentreX + circleRad)) && (racer->pos.y <= (circleCentreY + circleRad)) && (racer->pos.y >= (circleCentreY - circleRad))) //Makes square instead of circle
    {
        blue_place = 1.0;
    }
    else
    {
        blue_place = 0.0;
    }
    return blue_place;
}

//ENKI SIM
class EnkiPlayground : public EnkiWidget
{
private:
    void motors_callback(const geometry_msgs::Twist::ConstPtr& msg){ //sets motor speeds
        double angular_speed = msg->angular.z; //slightly inappropriate use of Twist msg but totally functional, probably not worth making a custom msg
        double linear = msg->linear.y;

        if(linear>0)
        {
            racer->leftSpeed = (angular_speed*50.0) + 60.0; //60 is pretty arbitrary, can be increased if need rat to move faster
            racer->rightSpeed = (angular_speed*(-50.0)) + 60.0;
        }
        else
        {
            racer->leftSpeed = 0;
            racer->rightSpeed = 0;
        }
    }

protected:
	//ENKI object creation
    Racer* racer;
    PhysicalObject* pelletL;
    PhysicalObject* pelletR;
    //PhysicalObject* wall1;
    //PhysicalObject* wall2;
    double speed = SPEED;
    double prevX;
    double prevY;
    double prevA;
    
    double pelletGx = GREENPELLETX; //set in parameters.h
    double pelletGy = GREENPELLETY;  
    double pelletBx = BLUEPELLETX;
    double pelletBy = BLUEPELLETY;
   
    FILE* errorlog = NULL;
    FILE* fcoord = NULL;
    FILE* fspeed = NULL;

    
    //ROS PUBLISHER SETUP

    //nodehandle
    ros::NodeHandle nh;
    //publisher to output the 8 line sensor values
    ros::Publisher line_sensor_pub[8];
    //publisher to output the 9x9 'front camera'
    ros::Publisher camera_pub;
    //publisher to colour camera
    ros::Publisher camera_pub_colour;
    //ros explore data pub
    ros::Publisher explr_pub;
       
    
    //publisher to placeB (if rat in blue placefield)
    ros::Publisher placeB_publish;
    //publisher to placeG (if rat in green placefield)
    ros::Publisher placeG_publish;

    //=== NOT CURRENTLY IN USE BY RAT_MOVE.CPP ===
    //publisher to is_Rewarded (if rat gets reward)
    //ros::Publisher reward_publish;
    //publisher to distG (distance from Green pellet)
    ros::Publisher distanceG_publish;
    //publisher to distB (distance from Blue pellet)
    ros::Publisher distanceB_publish;
    //publisher for contactBLue
    //ros::Publisher on_contact_blue_publish;
    //ros::Publisher on_contact_green_publish;
    //subscriber for the motor control messages
    
    //ROS SUBSCRIBER SETUP
    ros::Subscriber sub;
    

public:
    EnkiPlayground(World *world, QWidget *parent = 0):
		EnkiWidget(world, parent)
	{
        camera_pub = nh.advertise<sensor_msgs::Image>("mybot/camera1/image_raw", 1);
        camera_pub_colour = nh.advertise<sensor_msgs::Image>("mybot/colour_camera/image_raw", 1);
        explr_pub = nh.advertise<geometry_msgs::Twist>("mybot/limbic", 1); //just used because its native and has multiple float fields, maybe find a better one
       

        sub = nh.subscribe("mybot/cmd_vel", 1, &EnkiPlayground::motors_callback, this);

#ifdef reflex
        errorlog = fopen("errorReflex.tsv","wt");
        fcoord = fopen("coordReflex.tsv","wt");
#endif
        
        

        //Reversal publishers
        
        //reward_publish = nh.advertise<std_msgs::Float32>("mybot/isRewarded",1);
        placeG_publish = nh.advertise<std_msgs::Float32>("mybot/inPlaceGreen",1);
        placeB_publish = nh.advertise<std_msgs::Float32>("mybot/inPlaceBlue",1);
        distanceG_publish = nh.advertise<std_msgs::Float32>("mybot/distGreen",1);
        distanceB_publish = nh.advertise<std_msgs::Float32>("mybot/distBlue",1);
        //seenR_publish = nh.advertise<enki_ros_pck::Sight>("mybot/seeReward",1);
        //on_contact_blue_publish = nh.advertise<std_msgs::Float32>("mybot/contactBlue", 1);
        //on_contact_green_publish = nh.advertise<std_msgs::Float32>("mybot/contactGreen", 1);      
        

        racer = new Racer(nInputs);
        racer->pos = Point(racerx, racery); // x and y of the start point
		racer->leftSpeed = 0;
		racer->rightSpeed = 0;
        world->addObject(racer);

        //set camera sensor pixels -> 9x9 array of ground sensors infront of robot
        //args, y distance from centre of robot, number of sensor in row, spacing between sensors
        racer->setPreds(26, 9, 2);
        racer->setPreds(24, 9, 2);
        racer->setPreds(22, 9, 2);
        racer->setPreds(20, 9, 2);
        racer->setPreds(18, 9, 2);
        racer->setPreds(16, 9, 2);
        racer->setPreds(14, 9, 2);
        racer->setPreds(12, 9, 2);
        racer->setPreds(10, 9, 2); //10 is front of robot, since robot is 20long and 10wide
        
        pelletL = new PhysicalObject();
       // green_pellet.name = new PhysicalObject();
        pelletL->setCylindric(3,3,10000);
        //green_pellet.name->setColor(Enki::Color(1,0,0)); //Enki::Color(R,G,B) so (1,0,0) = true red = rgb8
        pelletL->setColor(Enki::Color(0,1,0)); //Enki::Color(R,G,B) so (1,0,0) = true red = rgb8
        //green_pellet.name->pos = Point(green_pellet.x_coord, green_pellet.y_coord);
        pelletL->pos = Point(pelletGx,(pelletGy));
        //world->addObject(green_pellet.name);
        world->addObject(pelletL);

        pelletR = new PhysicalObject();
        //blue_pellet.name = new PhysicalObject();
        pelletR->setCylindric(3,3,10000);
        //blue_pellet.name->setCylindric(2,2,100);
        pelletR->setColor(Enki::Color(0,0,1)); //Enki::Color(R,G,B) so (0,0,1) = true blue = rgb8
        //blue_pellet.name->setColor(Enki::Color(0,0,1)); //Enki::Color(R,G,B) so (0,1,0) = true green = rgb8
        pelletR->pos = Point(pelletBx,(pelletBy));
        //blue_pellet.name->pos = Point(blue_pellet.x_coord,(blue_pellet.y_coord));
        world->addObject(pelletR);
        //world->addObject(blue_pellet.name);

    }

    ~EnkiPlayground(){
        fclose(fcoord);
        fclose(errorlog);
        for(int i=0; i<8; i++){
            line_sensor_pub[i].shutdown();
        }
        camera_pub.shutdown();
        camera_pub_colour.shutdown();
    }
	


//this is called every 30Hz
virtual void sceneCompletedHook()
	{
        //CAMERA SETUP=======================================================================
        sensor_msgs::Image msg;
       
        msg.header.seq = countRuns;
        msg.header.frame_id = "camera_link";
        msg.height = 1;
        msg.width = 1;
        msg.encoding = "mono8";
        msg.is_bigendian = 0;
        msg.step = 1;
        msg.data.push_back(uint8_t(racer->lineSensors[0]->getValue()*255.0));

    
        //publish the camera stuff
        msg.height = 9;
        msg.width = 9;
        msg.step = 9; 

        msg.data.pop_back();
        for(int i=0; i<9; i++){
            //do it row by row since rows are flipped for some weird reason
            for(int j=0; j<9; j++){
                msg.data.push_back(uint8_t(racer->groundSensorArray[(((i+1)*9)-1)-j]->getValue()*255.0));
            }
        }

        camera_pub.publish(msg);

        //add colour camera
	    msg.encoding = "rgba8";
	    msg.height = 9;
        msg.width = 9;
        msg.step = 9*4; 

        msg.data.pop_back();
	    for(int i=0; i<81; i++)
        {
		  msg.data.push_back(uint8_t(racer->rgbcamera.image[i].r()*255.0));
		  msg.data.push_back(uint8_t(racer->rgbcamera.image[i].g()*255.0));
		  msg.data.push_back(uint8_t(racer->rgbcamera.image[i].b()*255.0));
		  msg.data.push_back(uint8_t(racer->rgbcamera.image[i].a()*255.0));
	    }
	    msg.data.resize(9*4*9);
        
   	    camera_pub_colour.publish(msg);

        //LIMBIC SYSTEM===========================================================================
        //Limbic-system instance - from reversal-5ht  
        if (limbic_system_inst == 1)
        {
            
            limbic_system_inst=0;
        }
        //Limbic_system ls;
        limbsys.doStep(step, reward,green_place,blue_place,contactGreen, contactBlue,green_distance,blue_distance,green_reward_distance,blue_reward_distance);
        step++;
        ROS_INFO("%d", step);
        
        explore_left =limbsys.getExploreLeft();
	    explore_right =limbsys.getExploreRight(); 
       
        float Greensw=limbsys.getGreenOutput();
		float Bluesw=limbsys.getBlueOutput();
        
        //Limbic system ROS Publishing
        geometry_msgs::Twist limbic;
        limbic.angular.x = explore_left;
        limbic.angular.y = explore_right;
        limbic.linear.x = Greensw;
        limbic.linear.y = Bluesw;
        explr_pub.publish(limbic);       


        //MISC=========================================================================================
        
        if ( pelletL->speed.x != 0) // return pellet to start point in case of collision (shouldnt happen but sometimes does)
        {
            pelletL->speed.x=0;
            pelletL->speed.y=0;
            pelletL->pos = Point((pelletGx),(pelletGy));
        }

        if ( pelletR->speed.x != 0)
        {
            pelletR->speed.x=0;
            pelletR->speed.y=0;
            pelletR->pos = Point((pelletBx),(pelletBy));
        }
        
        //LIMBIC-SYSTEM INPUTS
        
        switch (switch_toggle) //deals with reward switches whether done by timestep or rewards received
        {
       
        case 1: //timesteps
            
            if((steps_before_switch*toggle_n) == step)
            {
              if (rewardSet)
                {
                    rewardSet = 0;
                }
                else
                {
                    rewardSet = 1;
                }
                ROS_INFO("%s", "--REWARD SWITCHING--");
                toggle_n++;  
                racer->pos.x =racerx;
                racer->pos.y =racery;
            }
            break;
        case 0: //correct choices
        default:
            if(rewardcount >= rewards_before_switch)
            {   
            
                if (rewardSet)
                {
                    rewardSet = 0;
                }
                else
                {
                    rewardSet = 1;
                }
                ROS_INFO("%s", "--REWARD SWITCHING--");
                rewardcount = 0;
            }
            break;
        }
        
        switch (rewardSet) //Switches rewards
        {
        case 0: //Blue
            reward = rewardBoolB(racer, pelletR, maxx, maxy, BLUEPELLETX, BLUEPELLETY, 35);
            break;
        case 1: //Green
            reward = rewardBoolG(racer, pelletL, maxx, maxy, GREENPELLETX, GREENPELLETY, 35);
            break;
        }
        
        
        green_distance = getDistanceGreen(racer, pelletL, maxx, maxy); 
        blue_distance = getDistanceBlue(racer, pelletR, maxx, maxy);
        green_reward_distance = getDistanceRewardGreen(racer, pelletL, pelletGx, pelletGy, 35);
        blue_reward_distance = getDistanceRewardBlue(racer, pelletR, pelletBx, pelletBy, 35);
        
        contactBlue = on_contact_direction_blue(racer, pelletR);
        contactGreen = on_contact_direction_green(racer, pelletL);

        green_place = placeBoolGreen(racer, GREENPELLETX, GREENPELLETY, 35, maxx, maxy); // (msg, enki-racer, x coord of centre of place, y coord of centre of place, radius of place, max x coord of simulator, max y cord of simulator) call function which checks if rat is in defined place, sets reward.data. (located in ReversalSignals.h)
        blue_place = placeBoolBlue(racer, BLUEPELLETX, BLUEPELLETY, 35, maxx, maxy);
       
        //LIMBIC DATA PUBLISHING=============================================================================

        //ROS msgs        
        std_msgs::Float32 rewardSig;
        rewardSig.data = reward; 
        std_msgs::Float32 placeG;
        placeG.data = green_place; 
        std_msgs::Float32 placeB;
        placeB.data = blue_place;
        std_msgs::Float32 on_contact_blue_sig;
        on_contact_blue_sig.data = contactBlue; 
        std_msgs::Float32 on_contact_green_sig;
        on_contact_green_sig.data = contactGreen;
        std_msgs::Float32 distanceG; 
        distanceG.data  = green_distance;
        std_msgs::Float32 distanceB;
        distanceB.data = blue_distance;
        
        //Publish msgs
        //reward_publish.publish(rewardSig); //publishes data about if rat can see reward
        placeG_publish.publish(placeG); //publishes data about if rat is in defined place
        placeB_publish.publish(placeB); //publishes data about if rat is in defined place
        distanceG_publish.publish(distanceG); //publishes data about if rat can see reward and how far from reward it is.
        distanceB_publish.publish(distanceB);
        //seenR_publish.publish(seenR);
        //on_contact_blue_publish.publish(on_contact_blue_sig);
        //on_contact_green_publish.publish(on_contact_green_sig);
        
        ros::spinOnce();

        //----------------------------------------------------------------------------------------------------------

#ifdef reflex
        if (countRuns==0){
            fprintf(fcoord,"%e\t%e\n",racer->pos.x,racer->pos.y);
        }
#endif
        //fprintf(errorlog, "%e\t", error);
        //error = error * errorGain;

#ifdef reflex
        //racer->leftSpeed  = speed + error;
        //racer->rightSpeed = speed - error;
#endif

        countSteps ++;
        if (countSteps == 25000){ //sets maximum number of steps
            qApp->quit();
        }

	}
};

int main(int argc, char *argv[])
{
    srand(5);
    QApplication app(argc, argv);
    QImage gt;
    //Creating combpath which directs to our texture file location
	std::string pckpath = ros::package::getPath("enki_ros_pck"); //C++ string path to enki_ros_pck package folder
	QString qtpckpath = QString::fromStdString(pckpath); //convert to QString
	QString qtfile = QString().sprintf("/ColourSwapPF.png"); //convert /file_name.png to QString
	QString combpath = qtpckpath + qtfile; //combine both QStrings
    gt = QGLWidget::convertToGLFormat(QImage(combpath)); 
    if (gt.isNull()) {
        fprintf(stderr,"Texture file not found\n");
        exit(1);
    }
    const uint32_t *bits = (const uint32_t*)gt.constBits();
    World world(maxx, maxy,
                Color(1000, 1000, 1000), World::GroundTexture(gt.width(), gt.height(), bits));
    cout<<gt.width()<<" "<<gt.height()<<endl;

    ros::init(argc,argv,"enki_node");

    EnkiPlayground viewer(&world);
    viewer.show();

    return app.exec();
    return 0;
}