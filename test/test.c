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
    //RobotArm_testmov(&myRobotArm);
    //calculateAngles(1,1,0);
    //printf("--------------");
    //calculateAngles(2,0,0);
    //printf("++++++++++++++");
    

    //setServo(servoPin, currentMillis);
    //METER INICIALIZACION DEL ROBOT
    //setServo(4,1200);
    while (1)
    {



        for (float i = 0.5; i < 1.8; i=i+0.1)
        {
            moveTo(&myRobotArm,0.75,i,0);
            sleep_ms(500);
        }
        //
        //moveTo(&myRobotArm,1,1.3,0);
        //sleep_ms(500);
        //moveTo(&myRobotArm,0.75,1.3,0);
        //sleep_ms(500);
        //moveTo(&myRobotArm,0.5,1.3,0);
        //sleep_ms(500);
        //moveTo(&myRobotArm,0.25,1.3,0);
        //sleep_ms(500);
        //moveTo(&myRobotArm,0,1.3,0);
        //sleep_ms(500);
//
       
       
       //setMillis(3, angleToMillis(90));
       //setMillis(1, angleToMillis(90));
       //float * angles;
       //angles = calculateAngles(0,2,0);
       //
       //for ( int i = 0; i < 180; i++)
       //{
       //    setMillis(4, angleToMillis(i));
       //    sleep_ms(100);
       //}
       //for ( int i = 180; 0 < i; i--)
       //{
       //    setMillis(4, angleToMillis(i));
       //    sleep_ms(100);
       //}
       //
       ////printf("  %f, %f, %f  \n", angles[0], angles[1], angles[1]);
       //
       //int input = 0;
        //scanf("%d", &input);
        //if (input == 1)
        //{
        //    calculateAngles(0,2,0);
        //    printf("++++++++++++++");
        //}
        
        sleep_ms(100);

        //led = blink_pin_forever();
        //currentMillis += (direction)?5:-5;
        //if (currentMillis >= 2400) direction = false;
        //if (currentMillis <= 400) direction = true;
        //setMillis(servoPin, currentMillis);
        //sleep_ms(10);
    }
    
}
