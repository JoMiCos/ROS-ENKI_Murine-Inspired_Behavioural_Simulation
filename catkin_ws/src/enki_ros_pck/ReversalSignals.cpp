#include "ReversalSignals.h"

using namespace Enki;
using namespace std;
using namespace std::chrono;

//void setPelletColour(_Pellet _pellet){ //comments are for struct case, but I cannot get it working
void setPelletColour(Enki::PhysicalObject* pellet, int pellet_colour){    
    //int pellet_colour = _pellet.colour;
    switch (pellet_colour){
    case RED:
       // _pellet.name->setColor(Enki::Color(1,0,0));
        pellet->setColor(Enki::Color(1,0,0));
        break;
    case GREEN:
       // _pellet.name->setColor(Enki::Color(0,1,0));
       pellet->setColor(Enki::Color(0,1,0)); 
        break;
    case BLUE:
        //_pellet.name->setColor(Enki::Color(0,0,1));
        pellet->setColor(Enki::Color(0,0,1));
        break;
    case PURPLE:
        //_pellet.name->setColor(Enki::Color(1,0,1));
        pellet->setColor(Enki::Color(1,0,1));
        break;
    default:
        //_pellet.name->setColor(Enki::Color(0,0,0));
        pellet->setColor(Enki::Color(0,0,0));
        std::cout << "ERROR setPelletColour: please set pellet colout to RED, GREEN, BLUE, or PURPLE." << std::endl;
        break;
    }
}

//void seenBool(enki_ros_pck::Sight& seen, sensor_msgs::Image msg, _Pellet _pellet){
void seenBool(enki_ros_pck::Sight& seen, sensor_msgs::Image msg, int pellet_colour){
    static int vision[9];
    //int pellet_colour = _pellet.colour;
    switch(pellet_colour){
    case RED:    
        for (int i=0; i<7; i++)
        {
            vision[i] = msg.data[((80)+(i*16))]; //creates array of all R values from RGB vision-array (reward is pure red so == (255,0,0) uintRGB - R value every 4 but that makes it lag so 16
        }
            
        if( vision[0]==255 || vision[1]==255 || vision[2]==255 || vision[3]==255 || vision[4]==255 || vision[5]==255 || vision[6]==255 || vision[7]==255 || vision[8]==255 || vision[9]==255 || vision[10]==255 || vision[11]==255 || vision[12]==255 || vision[13]==255 || vision[14]==255 || vision[15]==255 || vision[16]==255 || vision[17]==255 || vision[18]==255 || vision[19]==255 || vision[20]==255 || vision[21]==255 || vision[22]==255 || vision[23]==255 || vision[24]==255 || vision [25]==255 || vision[26]==255 || vision[27]==255)        
        {
            seen.sight = 1.0;
        }
        
        else
        {
            seen.sight = 0.0;
        }
        break;
    
    case GREEN:
        for (int i=0; i<7; i++)
        {
            vision[i] = msg.data[((81)+(i*16))]; //creates array of all R values from RGB vision-array (reward is pure red so == (255,0,0) uintRGB - R value every 4 but that makes it lag so 16
        }
            
        if( vision[0]==255 || vision[1]==255 || vision[2]==255 || vision[3]==255 || vision[4]==255 || vision[5]==255 || vision[6]==255 || vision[7]==255 || vision[8]==255 || vision[9]==255 || vision[10]==255 || vision[11]==255 || vision[12]==255 || vision[13]==255 || vision[14]==255 || vision[15]==255 || vision[16]==255 || vision[17]==255 || vision[18]==255 || vision[19]==255 || vision[20]==255 || vision[21]==255 || vision[22]==255 || vision[23]==255 || vision[24]==255 || vision [25]==255 || vision[26]==255 || vision[27]==255)        
        {
            seen.sight = 1.0;
        }
        
        else
        {
            seen.sight = 0.0;
        }
        break;

    case BLUE:
        for (int i=0; i<7; i++)
        {
            vision[i] = msg.data[((82)+(i*16))]; //creates array of all R values from RGB vision-array (reward is pure red so == (255,0,0) uintRGB - R value every 4 but that makes it lag so 16
        }
            
        if( vision[0]==255 || vision[1]==255 || vision[2]==255 || vision[3]==255 || vision[4]==255 || vision[5]==255 || vision[6]==255 || vision[7]==255 || vision[8]==255 || vision[9]==255 || vision[10]==255 || vision[11]==255 || vision[12]==255 || vision[13]==255 || vision[14]==255 || vision[15]==255 || vision[16]==255 || vision[17]==255 || vision[18]==255 || vision[19]==255 || vision[20]==255 || vision[21]==255 || vision[22]==255 || vision[23]==255 || vision[24]==255 || vision [25]==255 || vision[26]==255 || vision[27]==255)        
        {
            seen.sight = 1.0;
        }
        
        else
        {
            seen.sight = 0.0;
        }
        break;
    default:
        std::cout << "seenBool ERROR: Please set char colour to either RED, GREEN, or BLUE." << std::endl;
        seen.sight = false;
        break;
    }
}



//void rewardBool(Enki::Racer* racer, _Pellet _pellet, std_msgs::Bool& reward, double maxx, double maxy) // (racer->pos, pellet->pos)
/*void rewardBool(Enki::Racer* racer, Enki::PhysicalObject* pellet, std_msgs::Bool& reward, double maxx, double maxy, bool rewardDelay) // (racer->pos, pellet->pos)
{
     rewardDelay();
    //if (sqrt((racer->pos.x - _pellet.name->pos.x)*(racer->pos.x - _pellet.name->pos.x)+(racer->pos.y - _pellet.name->pos.y)*(racer->pos.y - _pellet.name->pos.y))<13.0)
    if (sqrt((racer->pos.x - pellet->pos.x)*(racer->pos.x - pellet->pos.x)+(racer->pos.y - pellet->pos.y)*(racer->pos.y - pellet->pos.y))<13.0)
    {	//robot half length + food radius = 10+2 = 12
        reward.data = true; 
        racer->pos = Point(maxx/2, maxy/2 -30);
    }
    else
    {
        reward.data = false;
    }
} //need some flag to check if landmark is reward
*/
//void getDistance(Enki::Racer* racer, _Pellet _pellet, enki_ros_pck::Sight& sight)
void getDistanceGreen(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight, float maxx, float maxy)
{
    //float xdistance = sqrt((racer->pos.x -  _pellet.name->pos.x)*(racer->pos.x -  _pellet.name->pos.x));
    //float ydistance = sqrt((racer->pos.y - _pellet.name->pos.y)*(racer->pos.y- _pellet.name->pos.y));
    
    float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
    float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    
    float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
    float max_distance = sqrt((maxx*maxx)+(maxy*maxy))-13;
    float normalised_distance = (1-(direct_distance/max_distance));
    if(normalised_distance > 1){normalised_distance=1.0;}
    sight.distance = normalised_distance;
}

void getDistanceBlue(Enki::Racer* racer, Enki::PhysicalObject* pellet, enki_ros_pck::Sight& sight, float maxx, float maxy)
{
    //float xdistance = sqrt((racer->pos.x -  _pellet.name->pos.x)*(racer->pos.x -  _pellet.name->pos.x));
    //float ydistance = sqrt((racer->pos.y - _pellet.name->pos.y)*(racer->pos.y- _pellet.name->pos.y));
    
    float xdistance = sqrt((racer->pos.x -  pellet->pos.x)*(racer->pos.x -  pellet->pos.x));
    float ydistance = sqrt((racer->pos.y - pellet->pos.y)*(racer->pos.y- pellet->pos.y));
    
    float direct_distance = sqrt((xdistance*xdistance)+(ydistance*ydistance)) -13; //13 = halflength of robot
    float max_distance = sqrt((maxx*maxx)+(maxy*maxy))-13;
    float normalised_distance = (1-(direct_distance/max_distance));
    if(normalised_distance > 1){normalised_distance=1.0;}
    sight.distance = normalised_distance;
}

void placeBoolGreen(std_msgs::Float32& placeBool, Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY)
{
    if ((racer->pos.x >= (circleCentreX - circleRad)) && (racer->pos.x <= (circleCentreX + circleRad)) && (racer->pos.y <= (circleCentreY + circleRad)) && (racer->pos.y >= (circleCentreY - circleRad))) //This doesnt actually work, I think it makes a square... maybe do it with objects (or at least see if you can). Although having tested it, it basically works (Longterm, add colour camera looking beneath)
    {
        placeBool.data = 1.0;
    }
    else
    {
        placeBool.data = 0.0;
    }
}

void placeBoolBlue(std_msgs::Float32& placeBool, Enki::Racer* racer, double circleCentreX, double circleCentreY, double circleRad ,double maxX, double maxY)
{
    if ((racer->pos.x >= (circleCentreX - circleRad)) && (racer->pos.x <= (circleCentreX + circleRad)) && (racer->pos.y <= (circleCentreY + circleRad)) && (racer->pos.y >= (circleCentreY - circleRad))) //This doesnt actually work, I think it makes a square... maybe do it with objects (or at least see if you can). Although having tested it, it basically works...
    {
        placeBool.data = 1.0;
    }
    else
    {
        placeBool.data = 0.0;
    }
}

/*
static int reward_counter{0};
static bool reward_flag{0};
//function that if within place for x time, pellet set to reward.
bool rewardDelay(int delay){
    if (reward_counter = delay)
    {
        reward_flag=1;
        reward_counter=0;
    }
    else
    {
        reward_counter++;
    }
return reward_flag;
}
*/