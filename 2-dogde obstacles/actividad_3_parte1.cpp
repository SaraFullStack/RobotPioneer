#include "Aria.h"
using namespace std;

class ActionGo : public ArAction
{
public:
  ActionGo(double maxSpeed, double stopDistance);
  virtual ~ActionGo(void) {};
  virtual ArActionDesired *fire(ArActionDesired currentDesired);
  virtual void setRobot(ArRobot *robot);

protected:
  ArRangeDevice *mySonar;
  ArActionDesired myDesired;
  double myMaxSpeed;
  double myStopDistance;
};

class ActionTurn : public ArAction
{
public:
  ActionTurn(double turnThreshold, double turnAmount);
  virtual ~ActionTurn(void) {};
  virtual ArActionDesired *fire(ArActionDesired currentDesired);
  virtual void setRobot(ArRobot *robot);

protected:
  ArRangeDevice *mySonar;
  ArActionDesired myDesired;
  double myTurnThreshold;
  double myTurnAmount;
  int myTurning;
};

ActionGo::ActionGo(double maxSpeed, double stopDistance) :
  ArAction("Go")
{
  mySonar = NULL;
  myMaxSpeed = maxSpeed;
  myStopDistance = stopDistance;
  setNextArgument(ArArg("Maximum speed ", &myMaxSpeed, " Maximum speed to go."));
  setNextArgument(ArArg("Stop distance" , &myStopDistance, " Distance at which to stop."));
}

void ActionGo::setRobot(ArRobot *robot)
{
  ArAction::setRobot(robot);
  mySonar = robot->findRangeDevice("sonar");
  if (robot == NULL)
    {
      ArLog::log(ArLog::Terse, "Warning: I found no sonar, deactivating.");
      deactivate();
    }
}

ArActionDesired *ActionGo::fire(ArActionDesired currentDesired)
{
  double range;
  double speed;

  myDesired.reset();

  if (mySonar == NULL)
  {
    deactivate();
    return NULL;
  }

  range = mySonar->currentReadingPolar(-45, 45) - myRobot->getRobotRadius();
  if (range > myStopDistance)
  {
    speed = range * .3;
    if (speed > myMaxSpeed)
      speed = myMaxSpeed;
      
    myDesired.setVel(speed);
  }
  else
  {
    myDesired.setVel(0);
  }

  return &myDesired;
}

ActionTurn::ActionTurn(double turnThreshold, double turnAmount) :
  ArAction("Turn")
{
  if (ArMath::randomInRange(0, 100) % 2 == 0) {
    turnAmount = -90;
  }
  myTurnThreshold = turnThreshold;
  myTurnAmount = turnAmount;
  setNextArgument(ArArg("Turn threshold (mm) ", &myTurnThreshold, " The number of mm away from obstacle to begin turnning."));
  setNextArgument(ArArg("Turn amount (deg) ", &myTurnAmount, " The number of degress to turn if turning."));
  myTurning = 0;
}

void ActionTurn::setRobot(ArRobot *robot)
{
  ArAction::setRobot(robot);
  mySonar = robot->findRangeDevice("sonar");
  if (mySonar == NULL)
  {
    ArLog::log(ArLog::Terse, "Warning: I found no sonar, deactivating.");
    deactivate(); 
  }
}

ArActionDesired *ActionTurn::fire(ArActionDesired currentDesired)
{
  double leftRange, rightRange;

  myDesired.reset();

  if (mySonar == NULL)
  {
    deactivate();
    return NULL;
  }
  
  leftRange = (mySonar->currentReadingPolar(0, 1000) - myRobot->getRobotRadius());
  rightRange = (mySonar->currentReadingPolar(-1000, 0) - myRobot->getRobotRadius());

  cout << leftRange << " leftRange " << myTurnThreshold << "\n";
  cout << rightRange << " rightRange " << myTurnThreshold << "\n";

  if (leftRange > myTurnThreshold && rightRange > myTurnThreshold)
  {
    myTurning = 0;
    myDesired.setDeltaHeading(0);
  }
  else if (myTurning)
  {
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  else if (leftRange < rightRange)
  {
    myTurning = -1;
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  else 
  {
    myTurning = 1;
    myDesired.setDeltaHeading(myTurnAmount * myTurning);
  }
  
  return &myDesired;
}



int main(int argc, char** argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;
  ArSonarDevice sonar;

  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  ArLog::log(ArLog::Normal, "Connected to robot.");

  ActionGo go(500, 1000);
  ActionTurn turn(1000, 90);
  ArActionStallRecover recover;

  robot.addRangeDevice(&sonar);
  robot.addAction(&recover, 100);
  robot.addAction(&go, 49);
  robot.addAction(&turn, 50);
  robot.enableMotors();
  robot.run(true);
  
  Aria::exit(0);
}