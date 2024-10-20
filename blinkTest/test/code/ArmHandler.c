
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "../servo/servo.h"
#include "ArmHandler.h"
#include <math.h>


#define MIN_ANGLE 0      // Ángulo mínimo (grados)
#define MAX_ANGLE 180    // Ángulo máximo (grados)
#define MIN_PULSE 400    // Tiempo en microsegundos para 0 grados (400 µs)
#define MAX_PULSE 2300   // Tiempo en microsegundos para 180 grados (2400 µs)
#define PI 3.141592654
bool direction = true;
int currentMillis = 800;
int initialMillis = 800;

unsigned int angleToMillis(int angle) {
    // Limitar el ángulo entre MIN_ANGLE y MAX_ANGLE
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;

    // Mapeo del ángulo al rango de pulsos
    return MIN_PULSE + ((MAX_PULSE - MIN_PULSE) * (angle - MIN_ANGLE)) / (MAX_ANGLE - MIN_ANGLE);
}

// Función para inicializar los pines del brazo robótico
void RobotArm_init(RobotArm *arm, int pinA, int pinB, int pinC) {
    arm->pinA = pinA;
    arm->pinB = pinB;
    arm->pinC = pinC;
    setServo(arm->pinA, arm->currentMillisA = angleToMillis(90));
    sleep_ms(400);
    setServo(arm->pinB, arm->currentMillisB = angleToMillis(0));
    sleep_ms(400);
    setServo(arm->pinC, arm->currentMillisC = angleToMillis(90));
    sleep_ms(1000);
}

// Función de prueba de movimiento (testmov)
void RobotArm_testmov(RobotArm *arm) {
    for (int i = 0; i < 1000; i++)
    {
        currentMillis += (direction)?1:-1;
        if (currentMillis >= 2400) direction = false;
        if (currentMillis <= 400) direction = true;
        setMillis(arm->pinA, currentMillis);
        sleep_ms(10);
    }
    currentMillis = initialMillis;
    for (int i = 0; i < 1000; i++)
    {
        currentMillis += (direction)?1:-1;
        if (currentMillis >= 2400) direction = false;
        if (currentMillis <= 400) direction = true;
        setMillis(arm->pinB, currentMillis);
        sleep_ms(10);
    }

    currentMillis = initialMillis;
    for (int i = 0; i < 1000; i++)
    {
        currentMillis += (direction)?1:-1;
        if (currentMillis >= 2400) direction = false;
        if (currentMillis <= 400) direction = true;
        setMillis(arm->pinC, currentMillis);
        sleep_ms(10);
    }
}

void moveTo(RobotArm * arm,float x, float y, float z)
{
    float * angles;
    
    angles = calculateAngles(x,y,z);

    setMillis(arm->pinA, angleToMillis(angles[0]));
    if (angleToMillis(angles[1])>10)setMillis(arm->pinB, angleToMillis(angles[1]));
    if (160>angleToMillis(angles[2]))setMillis(arm->pinC, angleToMillis(angles[2]));


    free(angles);
    //printf("  %f, %f, %f  \n", angles[0], angles[1], angles[2]);
}
int followTarget(RobotArm * arm, float weightNextValue)
{
    float weightCurrentValue = 1 - weightNextValue;
                     //vamos tomando solo el 20% del valor siguiente, generando un suavizado el movimiento
    arm->currentMillisA = arm->currentMillisA *weightCurrentValue + arm->targetMillisA*weightNextValue;
    arm->currentMillisB = arm->currentMillisB *weightCurrentValue + arm->targetMillisB*weightNextValue;
    arm->currentMillisC = arm->currentMillisC *weightCurrentValue + arm->targetMillisC*weightNextValue;
    setMillis(arm->pinA, round(arm->currentMillisA));
    setMillis(arm->pinB, round(arm->currentMillisB));
    setMillis(arm->pinC, round(arm->currentMillisC));
    int a =   abs(arm->currentMillisA - arm->targetMillisA) + 
                abs(arm->currentMillisB - arm->targetMillisB) +
                abs(arm->currentMillisC - arm->targetMillisC);
    printf("diferencia  %d \n",abs((int)(arm->currentMillisA - arm->targetMillisA)));
    printf("diferencia2 %d \n",abs((int) (arm->currentMillisB - arm->targetMillisB)));
    printf("diferencia3 %d \n",abs((int) (arm->currentMillisC - arm->targetMillisC)));
    printf("diferenciaT %d \n",a);
    return a;

}
void setTarget(RobotArm * arm,float x, float y, float z)
{
    float * angles;
    angles = calculateAngles(x,y,z);
    
    arm->targetMillisA = angleToMillis(angles[0]);
    arm->targetMillisB = angleToMillis(CONSTRAIN(angles[1], 15, 160));
    arm->targetMillisC = angleToMillis(CONSTRAIN(angles[2], 15, 160));
    
    free(angles);
}
float * calculateAngles(float x, float y, float z)
{
    float length0 = 1;
    float length1 = 1;
    float * rAngles;
    rAngles = (float *) malloc(sizeof(float)*3);
    float jointAngle2 = atan(z/x) * (180/PI);
  
    //Rotate point to xy plane

    float newX = sqrt(z*z + x*x);
    float newY = y;

    float length2 = sqrt(newX*newX + newY*newY);

    float cosAngle0 = ((length2 * length2) + (length0 * length0) - (length1 * length1)) / (2 * length2 * length0);

    float angle0 = acos(cosAngle0) * (180/PI);

    float cosAngle1 = ((length1 * length1) + (length0 * length0) - (length2 * length2)) / (2 * length1 * length0);

    float angle1 = acos(cosAngle1) * (180/PI);
    float atang = atan(newY/newX) * (180/PI);
    float jointAngle0 = atang+angle0 ;// Angle shoulder
    float jointAngle1 = 180.0   - angle1;// Angle elbow

    rAngles[0] = jointAngle0;
    rAngles[1] = jointAngle1;
    rAngles[2] = jointAngle2;

    return rAngles;
}





