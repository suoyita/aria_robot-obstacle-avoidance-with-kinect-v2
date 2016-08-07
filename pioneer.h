#ifndef _PIONEER_H_
#define _PIONEER_H_

#include "global.h"
#include <signal.h>
#include <Aria.h>
#include <ArMode.h>
#include<cmath>
#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArAction.h"

class Avoid : public ArAction
{
public:

  AREXPORT Avoid(const char *name = "Avoid side",
                  ArPose goal = ArPose(0.0, 0.0, 0.0),
                      double obstacleFrontDistance = 450,
                  double obstacleSideDistance = 300,
                      double minDistance=10000);

  AREXPORT virtual ~Avoid();

  AREXPORT bool haveAchievedGoal(void);

  AREXPORT void cancelGoal(void);

  AREXPORT void setGoal(ArPose goal);

  AREXPORT virtual ArActionDesired * fire(ArActionDesired currentDesired);
  AREXPORT virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  AREXPORT virtual const ArActionDesired *getDesired(void) const
                                                        { return &myDesired; }
#endif
protected:
  ArPose obstacle;
  ArPose myGoal;
  double myObsDist_front;
  double myObsDist_side;
  double myTurnAmount;
  double minimum_Dist;
  int myTurning;

  ArActionDesired myDesired;
  ArPose myOldGoal;
  double leftDist_before;
  double rightDist_before;
  volatile bool STOP_CAR_FIRST;
  volatile bool ACHIEVE_ANGLE;
  volatile bool TURN_TO_GOAL;
  double poseAngle;

  enum State
  {
    STATE_NO_GOAL,
    STATE_ACHIEVED_GOAL,
    STATE_GOING_TO_GOAL
  };
  State myState;

};



#endif
