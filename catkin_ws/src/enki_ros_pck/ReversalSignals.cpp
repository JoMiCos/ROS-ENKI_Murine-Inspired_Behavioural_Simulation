#include "ReversalSignals.h"

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

void getDistance(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight sight)
{
    float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
    float ydistance = sqrt((racer->pos.y - pellet->pos.y) * (racer->pos.y- pellet->pos.y));
    float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
    ROS_INFO("%f",direct_distance);
    sight.distance = direct_distance;
}
