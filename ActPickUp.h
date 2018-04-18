/*
 "Grip" should be the highest priority. It will lower and open the grip before doing anything.
 	  Once the grip is lowered and open, "Grip" sets no actions until the object is in the gripper range.
 "Turn" should be the next highest priority. It will continually monitor whether it needs to turn toward
      the object until it is within gripping range.
 "MoveForward" is the lowest priority. It will move toward the object as long as "Grip" and "Turn" are not
 	  submitting move commands.
 Once 'MoveForward' has gotten the object within 220 from the center of the robot, "Grip" will take over,
      stop the robot from moving, and pick up the object.
 Once the object is picked up, "Grip"'s callback function will be activated, which should terminate this action.
*/

#ifndef ACTPICKUP_H_
#define ACTPICKUP_H_
#include "Aria.h"
#include "IndoorGPS.h"

// Very simple callback. You may have a more specific callback that you may want to use.
class CallbackContainer
{
public:
  void PickUpCB(bool success, ArActionGroup* group) {group->deactivate(); worked = success;};
  bool worked;
};

// This function is used in most of the classes, so declare it here.
// Verifies that a sensor reading is within a 200mm radius of the object's GPS coordinates
bool VerifyArea(double distance, double angle, double obj_x, double obj_y,ArRobot* robot);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class PickUpGrip : public ArAction
{
	public:
		PickUpGrip(ArRobot* robot,ArLaser* laser,ExternalGPS* GPS, ArGripper* gripper, ArActionGroup* group);
		virtual ~PickUpGrip(void) {};
		void Initialize(int obj_id);
		void setCallback(ArFunctor2<bool,ArActionGroup*> *callback) {myCB=callback;};
		virtual ArActionDesired *fire(ArActionDesired currentDesired);

	protected:
		ArRobot* robot;
		ArGripper* myGripper;
		ArActionDesired myDesired;
		ExternalGPS* myGPS;
		ArLaser* myLaser;
		ArActionGroup* myGroup;
		ArFunctor2<bool,ArActionGroup*> *myCB;
		ArTime time;
		float obj_x;
		float obj_y;
		float obj_z;
		bool ContToPickUp;
		bool firstiteration;
		double rot;
		int myObjID;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PickUpTurn : public ArAction
{
	public:
		PickUpTurn(ArRobot* robot,ArLaser* laser, ExternalGPS* GPS);
		virtual ~PickUpTurn(void) {};
		void Initialize(int obj_id);
		virtual ArActionDesired *fire(ArActionDesired currentDesired);

	protected:
		ArLaser* myLaser;
		ArActionDesired myDesired;
		ArRobot* myRobot;
		double myTurnAmount;
		ExternalGPS* myGPS;
		float obj_x;
		float obj_y;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class PickUpMoveForward : public ArAction
{
	public:
		PickUpMoveForward(ArRobot* robot,ArLaser* laser,ExternalGPS* GPS);
		virtual ~PickUpMoveForward(void) {};
		void Initialize(int obj_id);
		virtual ArActionDesired *fire(ArActionDesired currentDesired);
		int myObjID;
	protected:
		ArLaser* myLaser;
		ArActionDesired myDesired;
		ArRobot* myRobot;
		ExternalGPS* myGPS;
		float obj_x;
		float obj_y;
		bool first_run;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PickUpDropOff : public ArAction
{
	public:
		PickUpDropOff(ArRobot* robot,ArLaser* laser, ArGripper* gripper,ArActionGroup* group);
		virtual ~PickUpDropOff(void) {};
		void Initialize(void);
		void setCallback(ArFunctor2<bool,ArActionGroup*> *callback) {myCB=callback;};
		virtual ArActionDesired *fire(ArActionDesired currentDesired);
	protected:
		ArLaser* myLaser;
		ArActionDesired myDesired;
		ArRobot* myRobot;
		ArGripper* myGripper;
		ArTime myTime;
		ArFunctor2<bool,ArActionGroup*> *myCB;
		ArActionGroup* myGroup;

};


#endif
