#ifndef LED_DELAY_MS
#define LED_DELAY_MS 500
#endif 
#define LED_PIN 19
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
//Servo stuff
#include "servo/servo.h"

#include "code/ArmHandler.h"

static bool led = 1;

//bool direction = true;
//int currentMillis = 400;
int servoPin = 1;


bool blink_pin_forever() 
{
    gpio_put(LED_PIN, !led);
    sleep_ms(LED_DELAY_MS);
    return !led;
}



int main()
{
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    RobotArm myRobotArm;
    RobotArm_init(&myRobotArm, 1, 3, 4);
    float minx = 1;

    float maxx = -0.5;
    //RobotArm_testmov(&myRobotArm);
    //calculateAngles(1,1,0);
    //printf("--------------");
    //calculateAngles(2,0,0);
    //printf("++++++++++++++");
    

    //setServo(servoPin, currentMillis);
    //METER INICIALIZACION DEL ROBOT
    setServo(5,angleToMillis(0));

    setTarget(&myRobotArm, 1.5, 0.3, 0);
//
    while (followTarget(&myRobotArm,0.01))
    {
       printf("yendooo1");
    }
    setTarget(&myRobotArm, 0.2, 1.2, 0);
    while (followTarget(&myRobotArm, 0.01))
    {
       printf("yendooo2");
    }
    setTarget(&myRobotArm, -0.2, 1.5, 0);
    while (followTarget(&myRobotArm, 0.01))
    {
       printf("yendooo2");
    }
    setTarget(&myRobotArm, -0.8, 1.5, 0);
    while (followTarget(&myRobotArm, 0.01))
    {
       printf("yendooo2");
    }
}
