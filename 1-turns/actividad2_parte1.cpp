#include "Aria.h"

class ConnHandler {
  public:
    ConnHandler(ArRobot * robot);
  ~ConnHandler(void) {}
  void connected(void);
  void connFail(void);
  void disconnected(void);
  protected:
    ArRobot * myRobot;
  ArFunctorC < ConnHandler > myConnectedCB;
  ArFunctorC < ConnHandler > myConnFailCB;
  ArFunctorC < ConnHandler > myDisconnectedCB;
};

ConnHandler::ConnHandler(ArRobot * robot):
  myConnectedCB(this, & ConnHandler::connected),
  myConnFailCB(this, & ConnHandler::connFail),
  myDisconnectedCB(this, & ConnHandler::disconnected)

{
  myRobot = robot;
  myRobot -> addConnectCB( & myConnectedCB, ArListPos::FIRST);
  myRobot -> addFailedConnectCB( & myConnFailCB, ArListPos::FIRST);
  myRobot -> addDisconnectNormallyCB( & myDisconnectedCB, ArListPos::FIRST);
  myRobot -> addDisconnectOnErrorCB( & myDisconnectedCB, ArListPos::FIRST);
}

void ConnHandler::connFail(void) {
  myRobot -> stopRunning();
  Aria::exit(1);
  return;
}

void ConnHandler::connected(void) {
  myRobot -> comInt(ArCommands::SONAR, 0);
  myRobot -> comInt(ArCommands::ENABLE, 1);
  myRobot -> comInt(ArCommands::SOUNDTOG, 0);
}

void ConnHandler::disconnected(void) {
  Aria::exit(0);
}

int main(int argc, char ** argv) {
  Aria::init();

  ArArgumentParser argParser( & argc, argv);
  argParser.loadDefaultArguments();

  ArRobot robot;
  ArRobotConnector con( & argParser, & robot);

  ConnHandler ch( & robot);

  if (!Aria::parseArgs()) {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }

  if (!con.connectRobot()) {

    if (argParser.checkHelpAndWarnUnparsed()) {
      Aria::logOptions();
    }
    Aria::exit(1);
    return 1;
  }

  if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed()) {
    Aria::logOptions();
    Aria::exit(1);
  }

  //Codigo base del ejemplo de directMotion, con las modificaciones necesarias para el ejercicio
  robot.runAsync(false);
  int grades = 0;
  while( 1 ) {

    //Avanzamos 4 metros (4000mm)
    printf("Tell the robot to move forward 4 meters\n");
    robot.lock();
    robot.move(4000);
    robot.unlock();
    while( 1 ) {
      robot.lock();
      if( robot.isMoveDone() ) {
        printf("Distance ended \n");
        robot.unlock();
        break;
      }
      robot.unlock();
    }

    //Sumamos los 90 grados para el siguiente giro hasta completar la vuelta que volvemos a empezar con el primer giro de 90 grados
    //Giros realizados 90-180-270-360 grados
    grades += 90;
    if( grades > 360 ) grades = 90;
    
    printf("Tell the robot to turn 90 grades\n");
    robot.lock();
    robot.setHeading(grades);
    robot.unlock();
    while( 1 ) {
      robot.lock();
      if( robot.isHeadingDone() ) {
        printf("Turn completed \n");
        robot.unlock();
        break;
      }
      robot.unlock();
    }
  }
  
  printf("Ready, exiting. \n");
  Aria::exit(0);
  return 0;
}