#ifndef ArmHandler_h
#define ArmHandler_h
typedef struct {
    int pinA;
    int pinB;
    int pinC;
} RobotArm;

void RobotArm_testmov(RobotArm *arm);
void RobotArm_init(RobotArm *arm, int pinA, int pinB, int pinC);
float * calculateAngles(float x, float y, float z);
void moveTo(RobotArm * arm,float x, float y, float z);

#endif