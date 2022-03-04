#include "Aria.h"

class ConnHandler
{
public:
  ConnHandler(ArRobot *robot);
  ~ConnHandler(void) {}
  void connected(void);
  void connFail(void);
  void disconnected(void);
protected:
  ArRobot *myRobot;
  ArFunctorC<ConnHandler> myConnectedCB;
  ArFunctorC<ConnHandler> myConnFailCB;
  ArFunctorC<ConnHandler> myDisconnectedCB;
};

ConnHandler::ConnHandler(ArRobot *robot) :
  myConnectedCB(this, &ConnHandler::connected),  
  myConnFailCB(this, &ConnHandler::connFail),
  myDisconnectedCB(this, &ConnHandler::disconnected)

{
  myRobot = robot;
  myRobot->addConnectCB(&myConnectedCB, ArListPos::FIRST);
  myRobot->addFailedConnectCB(&myConnFailCB, ArListPos::FIRST);
  myRobot->addDisconnectNormallyCB(&myDisconnectedCB, ArListPos::FIRST);
  myRobot->addDisconnectOnErrorCB(&myDisconnectedCB, ArListPos::FIRST);
}

void ConnHandler::connFail(void)
{
  myRobot->stopRunning();
  Aria::exit(1);
  return;
}

void ConnHandler::connected(void)
{
  myRobot->comInt(ArCommands::SONAR, 0);
  myRobot->comInt(ArCommands::ENABLE, 1);
  myRobot->comInt(ArCommands::SOUNDTOG, 0);
}

void ConnHandler::disconnected(void)
{
  Aria::exit(0);
}

int main(int argc, char **argv) 
{
  Aria::init();
  
  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();

  ArRobot robot;
  ArRobotConnector con(&argParser, &robot);

  ConnHandler ch(&robot);

  if(!Aria::parseArgs())
  {
    Aria::logOptions();
    Aria::exit(1);
    return 1;
  }

  if(!con.connectRobot())
  {
    if(argParser.checkHelpAndWarnUnparsed()) 
    {
      Aria::logOptions();
	}
    Aria::exit(1);
    return 1;
  }

  if(!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  //Codigo base del ejemplo de directMotion, con las modificaciones necesarias para el ejercicio
  robot.runAsync(false);
  printf("Move the robot in a circle with a radius of two meters\n");

  //Para realizar una circunferencia de radio 2 calculamos la longitud de la circunferencia
  // 2 * PI * RADIO (2*3.14*2000) =  12560
  //Para dar algo de lentitud al giro hemos decidido devidir entre 8
  //Teniendo asi que el robot gira 45 (360/8) grados cada segundo y avanza 1570 (12560/8) mm cada segundo
  while( 1 )
  {
    robot.lock();
    //Velocidad de rotacion de 45 grados/segundo
    robot.setRotVel(45);
    //Velocidad deavance linear de 1570 mm/segundo
    robot.setVel(1570);
    robot.unlock();
  }
  printf("Ready, exiting. \n");
  robot.unlock();
  Aria::exit(0);
  return 0;
}

