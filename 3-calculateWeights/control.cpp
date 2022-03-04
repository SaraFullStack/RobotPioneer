#include "control.h"

Control::Control(ArRobot *robot) : robot(robot)
{
    init();
}


// IMPORTANTE
// We must "lock" the ArRobot object
// before calling its methods, and "unlock" when done, to prevent conflicts
// with the background thread started by the call to robot.runAsync() above.
// See the section on threading in the manual for more about this.
// Make sure you unlock before any sleep() call or any other code that will
// take some time; if the robot remains locked during that time, then
// ArRobot's background thread will be blocked and unable to communicate with
// the robot, call tasks, etc.

void Control::init()
{
    robot->lock();
    // Activar motores
    robot->enableMotors();

    // Establecer laser y sonar,
    // si no está alguno será = 0
    laser = robot->findLaser(1);
    sonar = (ArSonarDevice*)robot->findRangeDevice("sonar");

    // MobileSim no devuelve correctamente este valor si no se establece antes
    robot->setTransVelMax(600);

    robot->unlock();

    //Inicializamos los valores umbral
    valoresUmbral[0] = 250;
    valoresUmbral[1] = 500;
    valoresUmbral[2] = 1000;
    valoresUmbral[3] = 500;
    valoresUmbral[4] = 250;

    //Inicializamos los pesos que podemos variar para estudiar las salidas segun las prioridades aqui indicadas
    pesos[0][0] = 1;
    pesos[0][1] = 1;
    pesos[1][0] = 1;
    pesos[1][1] = 1;
    pesos[2][0] = 1;
    pesos[2][1] = 1;
    pesos[3][0] = 1;
    pesos[3][1] = 1;
    pesos[4][0] = 1;
    pesos[4][1] = 1;

}


void Control::execute()
{
    // Bucle de control
    while(true) {
        input();
        proccess();
        output();
        ArUtil::sleep(100);
    }
}

void Control::input()
{
    if(laser) {
        laser->lockDevice();

        /* Recogemos los valores del laser dividiendo el rango de -90..90 en 5 secciones
         * Aplicamos los controles de umbral para detectar solo los obstaculos cercanos
         * Normalizacion (entre 0 y 1) de los valores de entrada mediante la sigmoide como indica la practica
         */
        int valor;
        int control = 0;
        for( int i = -90; i <= 54; i += 36 ) {
            valor = laser->currentReadingPolar( i, ( i + 36) );

            if( valor > valoresUmbral[control] ) {
                sensoresEntry[control] = 0;
            } else {
                sensoresEntry[control] = 1 / ( 1 + pow( e, valor) );
            }

            control++;
        }

        laser->unlockDevice();
    }
    if(sonar) {
        sonar->getCurrentBufferAsVector();
    }
}

void Control::proccess()
{
    /* Calculo de los valores de salida mediante el sumatorio de 1 a total de valores de entrada
     * De la multiplicacion del valor de entrada en la posicion del sumatorio por el valor de la matriz de pesos en la posicion del sumatorio - posicion de la salida
     * Mas el valor de la constante zeta (0.5 segun indica la practica)
     * Y normalizacion de los datos entre -1..1 mediante el calculo de la tangente hiperbolica como indica la practica
     */
    double zeta = 0.5;

    for( int i = 0; i <= (int)( sizeof( pesos[0] ) / sizeof( *pesos[0] ) ); i++ ) {
        double salida = 0;
        for( int j = 0; j <= (int)( sizeof( sensoresEntry ) / sizeof( *sensoresEntry ) ); j++ ) {
            salida += sensoresEntry[j] * pesos[j][i];
        }
        salida += zeta;
        controlOut[i] = ( ( pow(e, salida) ) - ( pow(e, -salida) ) ) / ( ( pow(e, salida) ) + ( pow(e, -salida) ) );
    }
}

void Control::output()
{
    robot->lock();

    /* Trsladamos el vector resultante de la salida a las velocidades de cada rueda
     * De manera que se dirija a la direccion esperada para evitar el obstaculo
     */
    double vel = robot->getTransVelMax();
    double velDch = controlOut[0] * vel;
    double velIzq = controlOut[1] * vel;

    robot->setVel2( velIzq,  velDch );
    robot->unlock();
}


