#include <ros.h>
#include <std_msgs/UInt16.h>
#include <lib_uArm.h>

void doMove(const std_msgs::UInt16 &mov_action);

UF_uArm uArm;
ros::NodeHandle node;
ros::Subscriber<std_msgs::UInt16> moves("movements", doMove);

void doMove(const std_msgs::UInt16 &mov_action)
{
    unsigned upper = mov_action.data >> 8;
    unsigned lower = mov_action.data & 0xFF;

    int input = (int) upper;
    int angle = (int) lower;

    switch (input)
    {
        case 1:
            uArm.setPositionAngle(SERVO_LOWER, angle);
            delay(100);
            break;
        case 2:
            uArm.setPositionAngle(SERVO_UPPER, angle);
            delay(100);
            break;
        case 3:
            uArm.setPositionAngle(SERVO_BASE, angle);
            delay(100);
            break;
        case 4:
            uArm.pumpOn();
            delay(100);
            uArm.valveOn();
            break;
        case 5:
            uArm.pumpOff();
            delay(100);
            uArm.valveOff();
            break;
        case 6:
            uArm.reset();
            delay(100);
            break;
        case 7:
            // not implemented
            break;
        case 8:
            uArm.setPositionAngle(SERVO_HEAD, angle);
            delay(20);
            break;
    }
}

void setup()
{
    node.initNode();
    node.subscribe(moves);
    uArm.init();
    delay(100);
}

void loop()
{
    //uArm.setPosition(5, 15, 0, 0);
    node.spinOnce();
    delay(500);
}
