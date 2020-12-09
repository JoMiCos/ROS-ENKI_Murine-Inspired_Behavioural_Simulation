#include "ReversalSignals.h"

void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Bool& reward)
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