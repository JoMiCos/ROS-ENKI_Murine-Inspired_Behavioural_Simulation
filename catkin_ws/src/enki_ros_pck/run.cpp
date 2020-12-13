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
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <ros/package.h>
#include "enki_ros_pck/Sight.h"

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
#include "ReversalSignals.h"

#include "bandpass.h"
#include "parameters.h"
#include <chrono>
#include "PhysicalEngine.h"

#define reflex

using namespace Enki;
using namespace std;
using namespace std::chrono;

static double	maxx = 250;
static double	maxy = 250;
static double   racerx = (maxx/2)+30;
static double   racery = (maxy/2) +5;

int countSteps=0;
int countRuns=0;
int learningRateCount=0;
int firstStep =1; //so that it does propInputs once and then back/forth in that order

int nInputs= ROW1N+ROW2N+ROW3N; //this cannot be an odd number for icoLearner
static float contactBlue{0};
static float contactGreen{0};
int reward_counter{0};
static bool reward_flag = 0;
static bool start_timer{0};
static auto start = system_clock::now();
static seconds delay(21);//21==0????
//needs if in circle - change this to colour based on camera

void rewardDelay(seconds Delay){
    if (start_timer = 0){
        start = system_clock::now();
        start_timer = 1;
    }
    if ((start_timer = 1) && (system_clock::now() - start) > (delay)){
   // if (reward_counter = delay)
    //{
        reward_flag=1;
        start_timer=0;
    //    reward_counter=0;
    //}
    //else
    //{
    //    reward_counter++;
    }
}


void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Float32& reward, double racerx, double racery, double circleCentreX, double circleCentreY, double circleRad) // (racer->pos, pellet->pos)
{
    if ((racer->pos.x >= (circleCentreX - circleRad)) && (racer->pos.x <= (circleCentreX + circleRad)) && (racer->pos.y <= (circleCentreY + circleRad)) && (racer->pos.y >= (circleCentreY - circleRad))){
    rewardDelay(delay);
        if (reward_flag){
            //ROS_INFO("%s", "flag = true");
            pellet->setColor(Enki::Color(1,0,0));
            //if (sqrt((racer->pos.x - _pellet.name->pos.x)*(racer->pos.x - _pellet.name->pos.x)+(racer->pos.y - _pellet.name->pos.y)*(racer->pos.y - _pellet.name->pos.y))<13.0)
            if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<13.0)
            {	//robot half length + food radius = 10+2 = 12
                reward.data = true; 
                racer->pos = Point(racerx, racery);
                reward_flag=0;
                pellet->setColor(Enki::Color(0,0,1));
            }
            else
            {
                reward.data = false;
            }
        }
        else
        {
            //ROS_INFO("%s", "flag = false");
        }
    }
}

void on_contact_direction_blue(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Float32 contactBlue)
{
    if (reward_flag==0){
        if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<13.0)
        {	//robot half length + food radius = 10+2 = 12
            contactBlue.data = 1;
        }
        else
        {
            contactBlue.data = 0;
        }
    }
}

void on_contact_direction_green(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Float32 contactGreen)
{
    if (reward_flag==0){
        if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<13.0)
        {	//robot half length + food radius = 10+2 = 12
            contactGreen.data = 1;
        }
        else
        {
            contactGreen.data = 0;
        }
    }
}

void getDistanceReward(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight, float centrex, float centrey, float radius)
{
    if (reward_flag == 1)
    {
        float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
        float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    
        float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
        float max_distance = radius;
        float normalised_distance = (1-(direct_distance/radius));
        if(normalised_distance > 1){normalised_distance=1.0;}
        sight.distance = normalised_distance;
        sight.sight = 1;
    }
    else
    {
        sight.distance = 0;
        sight.sight = 0;
    }
    
}
class EnkiPlayground : public EnkiWidget
{
private:

    //callback function for neural network output to set motor speeds
    void motors_callback(const geometry_msgs::Twist::ConstPtr& msg){
        //random mapping I have assigned
        //need to find a way of doing this properly
        //racer->leftSpeed = msg->angular.z;
        //racer->rightSpeed = msg->angular.y;

        double angular_speed = msg->angular.z;

        //both will be 30 speed //used to multiply by 18
        racer->leftSpeed = (angular_speed*50.0) + 30.0;
        racer->rightSpeed = (angular_speed*(-50.0)) + 30.0;

        /**********************
         * 
         * MIGHT NEED TO CHANGE MULTIPLICATION HERE DEPENDING ON NEURAL NET OUTPUT VALUE....
         * 
         * **********************/
    }

protected:
	Racer* racer;
    PhysicalObject* pelletL;
    PhysicalObject* pelletR;
    PhysicalObject* wall1;
    PhysicalObject* wall2;
    double speed = SPEED;
    double prevX;
    double prevY;
    double prevA;

    /*
    _Pellet green_pellet;
    green_pellet.name = pelletL;
    green_pellet.x_coord = FOODCOORDX;
    green_pellet.y_coord = FOODCOORDY;
    green_pellet.colour = GREEN;


    _Pellet blue_pellet;
    blue_pellet.name = pelletR;
    blue_pellet.x_coord = maxx*41/50;
    blue_pellet.ycoord = maxy*4/5 -20;
    blue_pellet.colour = BLUE;
    */

    double pelletLx = GREENPELLETX; //set in parameters.h
    double pelletLy = GREENPELLETY;  
    double pelletRx = BLUEPELLETX;
    double pelletRy = BLUEPELLETY;
    double rewardPelletx = BLUEPELLETX;
    double rewardPellety = BLUEPELLETY; //going to need to write a function that flips which one is reward after certain number of completions.

    FILE* errorlog = NULL;
    FILE* fcoord = NULL;
    FILE* fspeed = NULL;


    //ros stuff
    ros::NodeHandle nh;
    //publisher to output the 8 line sensor values
    ros::Publisher line_sensor_pub[8];
    //publisher to output the 9x9 'front camera'
    ros::Publisher camera_pub;
    //publisher to colour camera
    ros::Publisher camera_pub_colour;
    //publisher to is_Rewarded (if rat gets reward)
    ros::Publisher reward_publish;
    //publisher to in_Blue_Place (if rat in assigned place)
    ros::Publisher placeB_publish;
    //publisher to in_Green_Place (if rat in assigned place)
    ros::Publisher placeG_publish;
    //publisher for is_Seen (if rat sees Green pellet)
    ros::Publisher seenG_publish;
    //publisher for is_Seen (if rat sees Blue pellet)
    ros::Publisher seenB_publish;
   
    ros::Publisher seenR_publish;
    //publisher for contactBLue
    ros::Publisher on_contact_blue_publish;

    ros::Publisher on_contact_green_publish;
    //subscriber for the motor control messages
    ros::Subscriber sub;

public:
    EnkiPlayground(World *world, QWidget *parent = 0):
		EnkiWidget(world, parent)
	{

        //N.B. all these use same topic names as gazebo
        line_sensor_pub[0] = nh.advertise<sensor_msgs::Image>("mybot/left_sensor4/image_raw",1);
        line_sensor_pub[1] = nh.advertise<sensor_msgs::Image>("mybot/left_sensor3/image_raw",1);
        line_sensor_pub[2] = nh.advertise<sensor_msgs::Image>("mybot/left_sensor2/image_raw",1);
        line_sensor_pub[3] = nh.advertise<sensor_msgs::Image>("mybot/left_sensor1/image_raw",1);
        line_sensor_pub[4] = nh.advertise<sensor_msgs::Image>("mybot/right_sensor1/image_raw",1);
        line_sensor_pub[5] = nh.advertise<sensor_msgs::Image>("mybot/right_sensor2/image_raw",1);
        line_sensor_pub[6] = nh.advertise<sensor_msgs::Image>("mybot/right_sensor3/image_raw",1);
        line_sensor_pub[7] = nh.advertise<sensor_msgs::Image>("mybot/right_sensor4/image_raw",1);

        camera_pub = nh.advertise<sensor_msgs::Image>("mybot/camera1/image_raw", 1);
        
        sub = nh.subscribe("mybot/cmd_vel", 1, &EnkiPlayground::motors_callback, this);

#ifdef reflex
        errorlog = fopen("errorReflex.tsv","wt");
        fcoord = fopen("coordReflex.tsv","wt");
#endif
        
        camera_pub_colour = nh.advertise<sensor_msgs::Image>("mybot/colour_camera/image_raw", 1);

        //Reversal publishers
        reward_publish = nh.advertise<std_msgs::Float32>("mybot/isRewarded",1);
        placeG_publish = nh.advertise<std_msgs::Float32>("mybot/inPlaceGreen",1);
        placeB_publish = nh.advertise<std_msgs::Float32>("mybot/inPlaceBlue",1);
        seenG_publish = nh.advertise<enki_ros_pck::Sight>("mybot/seeGreen",1);
        seenB_publish = nh.advertise<enki_ros_pck::Sight>("mybot/seeBlue",1);
        seenR_publish = nh.advertise<enki_ros_pck::Sight>("mybot/seeReward",1);
        on_contact_blue_publish = nh.advertise<std_msgs::Float32>("mybot/contactBlue", 1);
        on_contact_green_publish = nh.advertise<std_msgs::Float32>("mybot/contactGreen", 1);      

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
        pelletL->setCylindric(2,2,100);
        //green_pellet.name->setColor(Enki::Color(1,0,0)); //Enki::Color(R,G,B) so (1,0,0) = true red = rgb8
        pelletL->setColor(Enki::Color(0,1,0)); //Enki::Color(R,G,B) so (1,0,0) = true red = rgb8
        //green_pellet.name->pos = Point(green_pellet.x_coord, green_pellet.y_coord);
        pelletL->pos = Point(pelletLx,(pelletLy));
        //world->addObject(green_pellet.name);
        world->addObject(pelletL);

        pelletR = new PhysicalObject();
        //blue_pellet.name = new PhysicalObject();
        pelletR->setCylindric(2,2,100);
        //blue_pellet.name->setCylindric(2,2,100);
        pelletR->setColor(Enki::Color(0,0,1)); //Enki::Color(R,G,B) so (0,1,0) = true green = rgb8
        //blue_pellet.name->setColor(Enki::Color(0,0,1)); //Enki::Color(R,G,B) so (0,1,0) = true green = rgb8
        pelletR->pos = Point(pelletRx,(pelletRy));
        //blue_pellet.name->pos = Point(blue_pellet.x_coord,(blue_pellet.y_coord));
        world->addObject(pelletR);
        //world->addObject(blue_pellet.name);

        //maybe create own setColour - can use macros so easier to track...
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
	// here we do all the behavioural computations
	// as an example: line following and obstacle avoidance


//this is called every 30Hz
virtual void sceneCompletedHook()
	{
        /*
        int errorGain = ERRORGAIN;
		double leftGround = racer->groundSensorLeft.getValue();
		double rightGround = racer->groundSensorRight.getValue();
        double error = (leftGround - rightGround);
        */

        //setup the ros message
        sensor_msgs::Image msg;
        msg.header.seq = countRuns;
        msg.header.frame_id = "camera_link";
        msg.height = 1;
        msg.width = 1;
        msg.encoding = "mono8";
        msg.is_bigendian = 0;
        msg.step = 1;
        msg.data.push_back(uint8_t(racer->lineSensors[0]->getValue()*255.0));

        line_sensor_pub[0].publish(msg);

        for(int i=1; i<8; i++){

            msg.data.pop_back();
            msg.data.push_back(uint8_t(racer->lineSensors[i]->getValue()*255.0));
            line_sensor_pub[i].publish(msg);
        }

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

        //make this a function
        // return pellet to start point in case of collision (shouldnt happen but sometimes does)
        if ( pelletL->speed.x != 0)
        {
            pelletL->speed.x=0;
            pelletL->speed.y=0;
            pelletL->pos = Point((pelletLx),(pelletLy));
        }

        if ( pelletR->speed.x != 0)
        {
            pelletR->speed.x=0;
            pelletR->speed.y=0;
            pelletR->pos = Point((pelletRx),(pelletRy));
        }
        
        //---------------------------------REVERSAL DATA PUBLISHING------------------------------------------------
        //name ros msgs
        std_msgs::Float32 reward; //bool message for if rat is at reward
        std_msgs::Float32 placeG; // bool message is rat is in defined place
        std_msgs::Float32 placeB;
        std_msgs::Float32 on_contact_blue; //float message for if rat is touching blue landmark (0 or 1)
        std_msgs::Float32 on_contact_green; //float message for if rat is touching green landmark (0 or 1)
        enki_ros_pck::Sight seenG; //bool and distance message for if rat sees green pellet and how far it is from green pellet
        enki_ros_pck::Sight seenB; //bool and distance message for if rat sees blue pellet and how far it is from blue pellet
        enki_ros_pck::Sight seenR; //bool and distance message for if rat sees reward and how far it is from reward


        //assign values to msgs
        //rewardBool(racer, pelletL, reward, maxx, maxy); // calls rewardBool function which checks if rat has received reward (located in ReversalSignals.h)
        rewardBool(racer, pelletR, reward, maxx, maxy, BLUEPELLETX, BLUEPELLETY, 20);
        getDistanceGreen(racer, pelletL, seenG, maxx, maxy); //calls function which checks distance from rat to reward, sets seen.distance. (located in ReversalSignals.h)
        getDistanceBlue(racer, pelletR, seenB, maxx, maxy);
        getDistanceReward(racer, pelletR, seenR, rewardPelletx, rewardPellety, 20);
        on_contact_direction_blue(racer, pelletR, on_contact_blue);
        on_contact_direction_green(racer, pelletL, on_contact_green);


        seenBool(seenG, msg, GREEN); // calls function which checks is rat can see reward, sets seen.sight. (located in ReversalSignals.h)
        seenBool(seenB, msg, BLUE);

        placeBoolGreen(placeG, racer, GREENPELLETX, GREENPELLETY, 20, maxx, maxy); // (msg, enki-racer, x coord of centre of place, y coord of centre of place, radius of place, max x coord of simulator, max y cord of simulator) call function which checks if rat is in defined place, sets reward.data. (located in ReversalSignals.h)
        placeBoolBlue(placeB, racer, BLUEPELLETX, BLUEPELLETY, 20, maxx, maxy);
        //Publish msgs
        reward_publish.publish(reward); //publishes data about if rat can see reward
        placeG_publish.publish(placeG); //publishes data about if rat is in defined place
        placeB_publish.publish(placeB); //publishes data about if rat is in defined place
        seenG_publish.publish(seenG); //publishes data about if rat can see reward and how far from reward it is.
        seenB_publish.publish(seenB);
        seenR_publish.publish(seenR);
        on_contact_blue_publish.publish(on_contact_blue);
        on_contact_green_publish.publish(on_contact_green);

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
        if (countSteps == STEPSCOUNT){
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
	QString qtfile = QString().sprintf("/circles.png"); //convert /file_name.png to QString
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
}

//dostep( rewardBool, placeBoolGreen, placeBoolBlue, on_contact_direction_green, on_contact_direction_blue, getDistanceGreen, getDistanceBlue, need visual_reward_gb )