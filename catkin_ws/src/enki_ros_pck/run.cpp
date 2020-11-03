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
//#include "ReversalSignals.h"

#include "bandpass.h"
#include "parameters.h"
#include <chrono>
#include "PhysicalEngine.h"

#define reflex

using namespace Enki;
using namespace std;
using namespace std::chrono;

double	maxx = 250;
double	maxy = 250;

int countSteps=0;
int countRuns=0;
int learningRateCount =0;
int firstStep =1; //so that it does propInputs once and then back/forth in that order

int nInputs= ROW1N+ROW2N+ROW3N; //this cannot be an odd number for icoLearner
//bool rewardBool;
std_msgs::Bool reward;
//reward.data = false;

//move this to its own file and header
//void rewardBool(float ratx, float raty, float pelx, float pely, std_msgs::Bool& reward) // (racer->pos, pellet->pos)
//{
   //ROS_INFO("%s", "Entered Function");
  //  if (sqrt((ratx - pelx)*(ratx - pelx)+(raty - pely)*(raty - pely))<13.0)
   // {	//robot half length + food radius = 10+2 = 12
   //     reward.data = true; 
        //ROS_INFO("%s", "Entered True");
   // }
   // else
   // {
        //ROS_INFO("%f", "pelx");
   //     reward.data = false;
   // }
    
//}

void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Bool& reward, double maxx, double maxy) // (racer->pos, pellet->pos)
{
   // ROS_INFO("%f", racer->pos.x);
   // ROS_INFO("%f", racer->pos.y);
   // ROS_INFO("%f", pellet->pos.x);
   // ROS_INFO("%f", pellet->pos.x);
    //ROS_INFO("%f",sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x))+sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y)));
    if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<13.0)
    {	//robot half length + food radius = 10+2 = 12
        reward.data = true; 
        racer->pos = Point(maxx/2, maxy/2 -30);
    //    ROS_INFO("%s", "Entered True");
    }
    else
    {
        //ROS_INFO("%f", "pellet->pos.x");
        reward.data = false;
    }
    
}

void getDistance(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight sight)
{
    float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
    float ydistance = sqrt((racer->pos.y - pellet->pos.y) * (racer->pos.y- pellet->pos.y));
    float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
    ROS_INFO("%f",direct_distance);
    sight.distance = direct_distance;

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
    PhysicalObject* pellet;
    double speed = SPEED;
    double prevX;
    double prevY;
    double prevA;
    

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
    //publisher to in_Place (if rat in assigned place)
    ros::Publisher place_publish;
    //publisher for is_Seen (if rat sees reward)
    ros::Publisher seen_publish;
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
        camera_pub_colour = nh.advertise<sensor_msgs::Image>("mybot/colour_camera/image_raw", 1);

        //Reversal publishers
        reward_publish = nh.advertise<std_msgs::Bool>("mybot/isRewarded",1);
        place_publish = nh.advertise<std_msgs::Bool>("mybot/inPlace",1);
        //seen_publish = nh.advertise<std_msgs::Bool>("mybot/isSeen", 1);
        seen_publish = nh.advertise<enki_ros_pck::Sight>("mybot/isSeen",1);
        sub = nh.subscribe("mybot/cmd_vel", 1, &EnkiPlayground::motors_callback, this);

#ifdef reflex
        errorlog = fopen("errorReflex.tsv","wt");
        fcoord = fopen("coordReflex.tsv","wt");
#endif
        racer = new Racer(nInputs);
        racer->pos = Point(maxx/2 +30, maxy/2 +5); // x and y of the start point
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

        pellet = new PhysicalObject();
        pellet->setCylindric(2,2,100);
        pellet->setColor(Enki::Color(1,0,0)); //Enki::Color(R,G,B) so (1,0,0) = true red = rgb8
        pellet->pos = Point((maxx*9/50),(maxy*4/5 -20));
        world->addObject(pellet);

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

        //------------Move everything inside to headers-----------
        //set robot back if it gets very close to food
	    //if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<13.0)
        //{	//robot half length + food radius = 10+2 = 12
		    //racer->pos = Point(maxx/2, maxy/2 -30); //return racer to start point
            //rewardBool = true;  
        //}

        //else
        //{
            //rewardBool = false;
        //}
        
        //-------------------------------------------------------------
    
        // return pellet to start point in case of collision (shouldnt happen but sometimes does)
        if ( pellet->speed.x != 0)
        {
            pellet->speed.x=0;
            pellet->speed.y=0;
            pellet->pos = Point((maxx*9/50),(maxy*4/5 -20));
        }
        //-------------------------------------------------------------

        //Write bool true if within set circle. (make this a function when you work out how to pass racer->pos into a function...)
        double circleOnex = maxx*9/50;
        double circleOney = (maxy*4/5 -20);
        double circleOneRad = 20;
        bool placeBool = 0;

        if (((racer->pos.x >= (circleOnex - circleOneRad)) && (racer->pos.x <= (circleOnex + circleOnex))) && ((racer->pos.y <= (circleOney + circleOneRad)) && (racer->pos.y >= (circleOney - circleOneRad))))
        {
            placeBool = 1;
        }
        else
        {
            placeBool = 0;
        }
        //---------------------------------------------------------

        //reward seen.
        static int vision[9]; //fix this later (if im right and it needs fixed)
        bool seenBool = 0;
        //sensor_msgs::Image fov;
        //sensor_msgs::Image
        //msg.data[324]; //field of view data 
               
        for (int i=0; i<7; i++)
        {
          vision[i] = msg.data[((80)+(i*16))]; //creates array of all R values from RGB vision-array (reward is pure red so == (255,0,0) uintRGB - R value every 4 but that makes it lag so 16
        }
        
       // why is the rat blind on the left side?

        //if((vision[8] != vision[2]) ||(vision[7] != vision[3]) || (vision[6] != vision[4]))
        if( vision[0]==255 || vision[1]==255 || vision[2]==255 || vision[3]==255 || vision[4]==255 || vision[5]==255 || vision[6]==255 || vision[7]==255 || vision[8]==255 || vision[9]==255 || vision[10]==255 || vision[11]==255 || vision[12]==255 || vision[13]==255 || vision[14]==255 || vision[15]==255 || vision[16]==255 || vision[17]==255 || vision[18]==255 || vision[19]==255 || vision[20]==255 || vision[21]==255 || vision[22]==255 || vision[23]==255 || vision[24]==255 || vision [25]==255 || vision[26]==255 || vision[27]==255)        
        {
           /* ROS_INFO("%s","____________");
            ROS_INFO("%d",vision[0]);
            ROS_INFO("%d",vision[1]);
            ROS_INFO("%d",vision[2]);
            ROS_INFO("%d",vision[3]);
            ROS_INFO("%d",vision[4]);
            ROS_INFO("%d",vision[5]);
            ROS_INFO("%d",vision[6]);
            ROS_INFO("%d",vision[7]);
            ROS_INFO("%d",vision[8]);
            ROS_INFO("%d",vision[9]);
            ROS_INFO("%d",vision[10]);
            ROS_INFO("%d",vision[11]);
            ROS_INFO("%d",vision[12]);
            ROS_INFO("%d",vision[13]);
            ROS_INFO("%d",vision[14]);
            ROS_INFO("%d",vision[15]);
            ROS_INFO("%d",vision[16]);
            ROS_INFO("%d",vision[17]);
            ROS_INFO("%d",vision[18]);
            ROS_INFO("%d",vision[19]);
            ROS_INFO("%d",vision[20]);
            ROS_INFO("%d",vision[21]);
            ROS_INFO("%d",vision[22]);
            ROS_INFO("%d",vision[23]);
            ROS_INFO("%d",vision[24]);
            ROS_INFO("%d",vision[25]);
            ROS_INFO("%d",vision[26]);
            ROS_INFO("%d",vision[27]); */

    
            seenBool = true;
        }
        
        else
        {
            
            seenBool = false;
        }
        //camera_pub_colour->data
        //if((sensor_values[8]/255.0) - (sensor_values[2]/255.0) != 0||(sensor_values[7]/255.0) -(sensor_values[3]/255.0) != 0||(sensor_values[6]/255.0) - (sensor_values[4]/255.0) != 0){
        

        //Reversal publishing
        std_msgs::Bool reward; //bool message for if rat is at reward
        std_msgs::Bool place;
        //std_msgs::Bool seen; //make this a custom msg later, add distance
        enki_ros_pck::Sight seen;

        //reward.data = rewardBool; //true if rat very close
        rewardBool(racer, pellet, reward, maxx, maxy);
        place.data = placeBool;
        seen.sight = seenBool;
        getDistance(racer, pellet, seen);
        
    /// Delete this second section - try not using custom msg and see if that fixes tilt...

        reward_publish.publish(reward);
        place_publish.publish(place);
        seen_publish.publish(seen);
        ros::spinOnce();

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

    //getCoordinates();

    ros::init(argc,argv,"enki_node");

    EnkiPlayground viewer(&world);
    viewer.show();

    return app.exec();
}