#include "ReversalSignals.h"

using namespace Enki;
using namespace std;
using namespace std::chrono;

void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Bool& reward, double maxx, double maxy) // (racer->pos, pellet->pos)
{
    if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<13.0)
    {	//robot half length + food radius = 10+2 = 12
        reward.data = true; 
        racer->pos = Point(maxx/2, maxy/2 -30);
    }
    else
    {
        reward.data = false;
    }
    
}

void getDistance(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight)
{
    float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
    float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
    
    sight.distance = direct_distance;
}

void seenBool(enki_ros_pck::Sight& seen, sensor_msgs::Image msg)
{
    static int vision[9];
            
    for (int i=0; i<7; i++)
    {
        vision[i] = msg.data[((80)+(i*16))]; //creates array of all R values from RGB vision-array (reward is pure red so == (255,0,0) uintRGB - R value every 4 but that makes it lag so 16
    }
            
    if( vision[0]==255 || vision[1]==255 || vision[2]==255 || vision[3]==255 || vision[4]==255 || vision[5]==255 || vision[6]==255 || vision[7]==255 || vision[8]==255 || vision[9]==255 || vision[10]==255 || vision[11]==255 || vision[12]==255 || vision[13]==255 || vision[14]==255 || vision[15]==255 || vision[16]==255 || vision[17]==255 || vision[18]==255 || vision[19]==255 || vision[20]==255 || vision[21]==255 || vision[22]==255 || vision[23]==255 || vision[24]==255 || vision [25]==255 || vision[26]==255 || vision[27]==255)        
    {
        seen.sight = true;
    }
        
    else
    {
        seen.sight = false;
    }
}

void placeBool(std_msgs::Bool& placeBool, Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY)
{
    if ((racer->pos.x >= (circleCentreX - circleRad)) && (racer->pos.x <= (circleCentreX + circleRad)) && (racer->pos.y <= (circleCentreY + circleRad)) && (racer->pos.y >= (circleCentreY - circleRad))) //This doesnt actually work, I think it makes a square... maybe do it with objects (or at least see if you can). Although having tested it, it basically works...
    {
        placeBool.data = true;
    }
    else
    {
        placeBool.data = false;
    }
}