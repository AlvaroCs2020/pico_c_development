#ifndef ArmHandler_h
#define ArmHandler_h
#define CONSTRAIN(value, min, max) ((value) < (min) ? (min) : (value) > (max) ? (max) : (value))
typedef struct {
    int pinA;
    int pinB;
    int pinC;
    float currentMillisA; //float, in order to hold de acumulative
    float currentMillisB;
    float currentMillisC;
    int targetMillisA;
    int targetMillisB;
    int targetMillisC;
} RobotArm;
unsigned int angleToMillis(int angle);
void RobotArm_testmov(RobotArm *arm);
void RobotArm_init(RobotArm *arm, int pinA, int pinB, int pinC);
float * calculateAngles(float x, float y, float z);
void moveTo(RobotArm * arm,float x, float y, float z);
int followTarget(RobotArm * arm, float weightNextValue);
void setTarget(RobotArm * arm,float x, float y, float z);
#endif