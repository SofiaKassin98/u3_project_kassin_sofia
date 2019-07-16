/*
* File: u3_exam_kassin_sofia.c
* Date:
* Description:
* Author:
* Modifications:
*/
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>

#include <stdio.h>
#include <math.h>

/*
* macros
*/
#define TIME_STEP 64
#define PI 3.141592
#define OBSTACLE_DISTANCE 55705.0


enum {     //es para darle un valor numerico a las palabras 0,1,2,3...
    Autonomous,
    Manual,
    Go,
    Turn,
    TurnL,
    TurnR,
    left,
    right,
    FreeWay,
    Obstacle
};

int A = 65, S = 83, G = 71, W = 87;    //codigo ASCII para cada una de las letras si se utiliza con el keyboard
int state;  //estado inicial
double initial_angle_wheel1; //variable que se utiliza en el calculo de giro

int checkForObstacles(WbDeviceTag distance_sensor) {   //funcion para checar obstaculos es el nombre del distance sensor
    double distance = wb_distance_sensor_get_value(distance_sensor);  //aqui se obtiene el valor del distance senspr

    if (distance > OBSTACLE_DISTANCE) //se compara el valor que se definio con el obstaculo si la distancia es mayor le da el camino libre si hay ,retorna el obstaculo
        return FreeWay;
else                  //el diagrama del profe
    return Obstacle; 
}

void goRobot(WbDeviceTag *wheels, double velocity) {  //funciones para que se muevan las llantas 
    wb_motor_set_velocity(wheels[0],0);
    wb_motor_set_velocity(wheels[1], velocity);
    wb_motor_set_velocity(wheels[2], -velocity);

}

void backRobot(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[0], 0);
    wb_motor_set_velocity(wheels[1],-6);
    wb_motor_set_velocity(wheels[2], 6);
}

void leftRobot(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[0], 6);
    wb_motor_set_velocity(wheels[1], 0);
    wb_motor_set_velocity(wheels[2],-6);
}

void rightRobot(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[0],-6);
    wb_motor_set_velocity(wheels[1], 0);
    wb_motor_set_velocity(wheels[2], 6);
}

void stopRobot(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[0], 0);
    wb_motor_set_velocity(wheels[1], 0);
    wb_motor_set_velocity(wheels[2], 0);
}

void turnLeft(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[0], 6);
    wb_motor_set_velocity(wheels[1], 6);
    wb_motor_set_velocity(wheels[2], 6);
}

void turnRight(WbDeviceTag *wheels) {
    wb_motor_set_velocity(wheels[0],-6);
    wb_motor_set_velocity(wheels[1],-6);
    wb_motor_set_velocity(wheels[2],-6);
}

double getAngleRobot(WbDeviceTag pos_sensor) {   //para obtener el angulo (parte en el diagrama de 90 >90, etc)
    double angle_wheel1 = wb_position_sensor_get_value(pos_sensor);  //obteniendo el valor que me da el position sensor
    double angle;  //variable que se llama angulo 

    angle = fabs(angle_wheel1 - initial_angle_wheel1);  //fabs es una funcion matematica que calcula el angulo que rota (buscar fabs)

    return angle;  //regrega el valor de angulo
}

/*
* main
*/
int main(int argc, char **argv)  //comienza el main
{
/* necessary to initialize webots stuff */
    wb_robot_init();   //se inicializa el robot
    wb_keyboard_enable(TIME_STEP);  //inicializa el teclado
    
    int key;                           //se declaran las variables , key asigna la tecla que tu presionas
    float velocity;                  //cambio en la velocidad de arranuqe
    short int ds_state, ds_state1, robot_state = Go;     //estados del robot de acuerdo a la meidcion de las distancias 
    float angle;     //  angulo
    float dis1, dis2;            //imprime la distancia 

    WbDeviceTag wheels[2];    //se inicializa las llantas y las pocisiones al infinito para que sigan girando
        wheels[0] = wb_robot_get_device("wheel1");
        wheels[1] = wb_robot_get_device("wheel2");
        wheels[2] = wb_robot_get_device("wheel3");
    wb_motor_set_position (wheels[0], INFINITY);
    wb_motor_set_position (wheels[1], INFINITY);
    wb_motor_set_position (wheels[2], INFINITY);

    WbDeviceTag DSensor[1];       //inicializando los sensores de distancia 
        DSensor[0] = wb_robot_get_device("DSENSOR_1");
        DSensor[1] = wb_robot_get_device("DSENSOR_2");
    wb_distance_sensor_enable(DSensor[0], TIME_STEP);
    wb_distance_sensor_enable(DSensor[1], TIME_STEP);
    
    WbDeviceTag encoder = wb_robot_get_device("enco1");  //inicializando el encoder
    wb_position_sensor_enable(encoder, TIME_STEP);  
/* 
* main loop
*/
while (wb_robot_step(TIME_STEP) != -1) {        

    key = wb_keyboard_get_key();            //asigno la tecla que presione a la variable key
    
    if (key == G)               //se empieza a declarar los estados dependiendo de cada letra
        state = Autonomous;  
    else if (key == W)
        state = Manual;
    else if (key == S){
        state = left;
        initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
    } else if (key == A){
        state = right;
        initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
}
    if (state == Autonomous){          //aqui comienza el codigo para el estado autonomo
        if (robot_state == Go) {             //subestado para que el robot avance , lo del diagrama de la libreta
            ds_state = checkForObstacles(DSensor[0]);  //ambos sensores estan revisando por obstaculos
            ds_state1 = checkForObstacles(DSensor[1]);
            
            dis1 = wb_distance_sensor_get_value(DSensor[0])/65535*20;    //asignas el valor de la distancia  la variable dis y se hace operaciones para que la distancia la muestre en centimetros
            dis2 = wb_distance_sensor_get_value(DSensor[1])/65535*20;
            
            printf ("dis1 : %lf cm \t ",dis1);   //imprime el valor de los sensores de distancia enc entimetros
            printf ("dis2 : %lf cm\n",dis2);
            
                if (ds_state == FreeWay && ds_state1 == FreeWay) {  //todos los casos que puede presentar el robot , es una tabla de verdad
                    velocity = 8;  
                    goRobot(wheels, velocity);
                    
                } else if (ds_state == Obstacle && ds_state1 == FreeWay) {
                    robot_state = TurnL;
                    stopRobot(wheels);
                    
                } else if (ds_state == FreeWay && ds_state1 == Obstacle) {
                    robot_state = TurnR;
                    stopRobot(wheels);
                    
                } else if (ds_state == Obstacle && ds_state1 == Obstacle) {
                    robot_state = TurnL;
                    stopRobot(wheels);
                }
        } else if (robot_state == TurnL){   //se encuentra girando hasta que no encuentra un obstaculo y si lo encuentra se detiene y gira al lado correspondiente 
            turnLeft(wheels);
            ds_state = checkForObstacles(DSensor[0]);
            ds_state1 = checkForObstacles(DSensor[1]);
            
            if (ds_state == FreeWay && ds_state1 == FreeWay) {
                robot_state = Go;
                stopRobot(wheels);
            }
            
        } else if (robot_state == TurnR) {
            turnRight(wheels);
            ds_state = checkForObstacles(DSensor[0]);
            ds_state1 = checkForObstacles(DSensor[1]);
            
            if (ds_state1 == FreeWay && ds_state == FreeWay) {
                robot_state = Go;
                stopRobot(wheels);
            }
        } 
  
    } else {    
        if (key == WB_KEYBOARD_UP){
            velocity = 6;
            goRobot(wheels, velocity);
            angle = wb_position_sensor_get_value(encoder);  //guarda la distancia que avanzo en angulo para cuando tenga que rotar
            

            printf("Angle: %lf\n", angle);
            
        }else if (key == WB_KEYBOARD_DOWN){
            backRobot(wheels);
            angle = wb_position_sensor_get_value(encoder);
            
            printf("Angle: %lf\n", angle);
            
        } else if (key == WB_KEYBOARD_LEFT){
            leftRobot(wheels);
            angle = wb_position_sensor_get_value(encoder);  //siempre se almacena , no importa las teclas que presione

            printf("Angle: %lf\n", angle);

        } else if (key == WB_KEYBOARD_RIGHT){
            rightRobot(wheels);
            angle = wb_position_sensor_get_value(encoder);
            
            printf("Angle: %lf\n", angle);
            
        } else if (state == left){
            turnLeft(wheels);
 
            angle = getAngleRobot(encoder);  //par saber si ya pasaron los angulos , funciones que hice arriba debajo de las llantas
            
            if (angle >= 0.4*PI) {  //depende de las llantas entonces se sabe que 4 PI es una llanta completa
                robot_state = Go;      //0.4 PI son aprox los 45 grados , no es exacto sino un aproximado que se saca con esta funcion
                stopRobot(wheels);
            }
        } else if (state == right){
            turnRight(wheels);
            angle = getAngleRobot(encoder);  //esto es lo mismo pero hacia el otro lado
            
            if (angle >= 0.4*PI) {
                robot_state = Go;
                stopRobot(wheels);
            }
            
        } else {
            stopRobot(wheels);    //si no apriestas nada que el robot se quede detenido
        }
            dis1 = wb_distance_sensor_get_value(DSensor[0])/65535*20;
            dis2 = wb_distance_sensor_get_value(DSensor[1])/65535*20;  //imprimiendo distance sensor 

            printf ("dis1 : %lf cm \t  ",dis1);
            printf ("dis2 : %lf cm \n",dis2);
        }

    }

/* Enter your cleanup code here */

/* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}
