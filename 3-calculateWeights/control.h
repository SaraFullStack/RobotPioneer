#ifndef CONTROL_H
#define CONTROL_H

#include <Aria.h>

class Control
{
public:
    Control(ArRobot* robot);
    void execute();

    double sensoresEntry[5];
    int valoresUmbral[5];
    int pesos[5][2];
    double controlOut[2];
    double e = 2.71828;

protected:
    virtual void init();
    virtual void input();
    virtual void proccess();
    virtual void output();

protected:
    ArRobot* robot;
    ArLaser* laser;
    ArSonarDevice* sonar;

};

#endif // CONTROL_H
