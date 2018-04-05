#include "updatePos.h"

// Constructor, add the update task task to the robot
// id should be the trackable number assigned to this robot
updatePos::updatePos(ArRobot *robot, ExternalGPS *GPS,int id) :
		updatePosCB(this, &updatePos::updatePosition)
{
	myRobot = robot;
	myGPS = GPS;
	myID = id;

	myRobot->addUserTask("updatePosition",90,&updatePosCB);
}


void updatePos::updatePosition()
{
    obj_data_t GPS_pos;

    GPS_pos = myGPS->obj_data_buff[myID];

    // if x,y=0 then the GPS system has lost track of the
    // the robot, so do NOT update the position, or else
    // the robot will think it is at 0.0,0.0
    if(!(GPS_pos.x)==0.0 && !(GPS_pos.y==0.0)) {
		ArPose GPSUpdate(GPS_pos.x,GPS_pos.y,GPS_pos.th);
		myRobot->moveTo(GPSUpdate);
    }

    // Note for Optitrack:
    // z axis in the GPS, x axis for the robot
    // x axis in the GPS, y axis for the robot
    // y axis in the GPS, z axis for the robot
    // yaw    in the GPS, theta  for the robot
}

// Destructor, empty
updatePos::~updatePos()
{
}
