// Note for Optitrack:
// z axis in the GPS, x axis for the robot
// x axis in the GPS, y axis for the robot
// y axis in the GPS, z axis for the robot
// yaw    in the GPS, theta  for the robot

#ifndef UPDATEPOS_H_
#define UPDATEPOS_H_

#include "IndoorGPS.h"
#include "Aria.h"
class updatePos
{
public:
  updatePos(ArRobot *robot, ExternalGPS *GPS, int id);
  virtual ~updatePos();
  void updatePosition(void);

protected:
  ArRobot *myRobot;
  ExternalGPS *myGPS;
  ArFunctorC<updatePos> updatePosCB;
  int myID;
};


#endif /* UPDATEPOS_H_ */
