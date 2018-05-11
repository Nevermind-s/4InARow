#include <lib_uArm.h>

UF_uArm::UF_uArm()
{
    lowerPos = 25;
    upperPos = 70; 
    basePos  = 80; 
    headPos  = 90;
}

void UF_uArm::init()
{
    pinMode(PUMP_EN,  OUTPUT);
    pinMode(VALVE_EN, OUTPUT);
    digitalWrite(PUMP_EN,  LOW);  // initiate pump
    digitalWrite(VALVE_EN, LOW);  // initiate valve
    servoLower.attach(SERVO_LOWER);
    servoUpper.attach(SERVO_UPPER);
    servoBase.attach(SERVO_BASE);
    servoHead.attach(SERVO_HEAD);
    setServoSpeed(SERVO_LOWER, 1);  // 0=full speed, 1-255 slower to faster
    setServoSpeed(SERVO_UPPER, 1);  // 0=full speed, 1-255 slower to faster
    setServoSpeed(SERVO_BASE,  1);  // 0=full speed, 1-255 slower to faster
    setServoSpeed(SERVO_HEAD,  1);
	reset();
}

void UF_uArm::setPosition(double _stretch, double _height, int _armRot, int _handRot)
{
	int offsetL = 0, offsetR = 0, heightLst = 0;
	_armRot = -_armRot;
	if(!digitalRead(LIMIT_SW) && _height < heightLst) //limit switch protection
	_height = heightLst;
	// input limit
	_stretch = constrain(_stretch, ARM_STRETCH_MIN,   ARM_STRETCH_MAX) + 55;		// +55, set zero -stretch 
	_height  = constrain(_height,  ARM_HEIGHT_MIN,    ARM_HEIGHT_MAX);
	_armRot  = constrain(_armRot,  ARM_ROTATION_MIN,  ARM_ROTATION_MAX) + 90;		// +90, change -90~90 to 0~180
	_handRot = constrain(_handRot, HAND_ROTATION_MIN, HAND_ROTATION_MAX) + 90;	// +90, change -90~90 to 0~180
	// angle calculation
	double stretch2height2 = _stretch * _stretch + _height * _height;              // 
	double angleA = (acos( (ARM_A2B2 - stretch2height2) / ARM_2AB )) * RAD_TO_DEG; // angle between the upper and the lower
	double angleB = (atan(_height/_stretch)) * RAD_TO_DEG;                         // 
	double angleC = (acos((ARM_A2 + stretch2height2 -ARM_B2)/(2 * ARM_A * sqrt(stretch2height2)))) * RAD_TO_DEG; // 
	int angleR = 180 - angleA - angleB - angleC + FIXED_OFFSET_R + offsetR;        // 
	int angleL = angleB + angleC + FIXED_OFFSET_L + offsetL;                       // 
	// angle limit
	angleL = constrain(angleL, 10 + offsetL, 145 + offsetL);
	angleR = constrain(angleR, 25 + offsetR, 150 + offsetR);
	angleR = constrain(angleR, angleL - 90 + offsetR, angleR);	// behind  -120+30 = -90
	if(angleL < 15 + offsetL)
	angleR = constrain(angleR, 70 + offsetR, angleR);			// front down
	// set servo position
	servoUpper.write(angleR,  servoSpdUpper, false);
	servoLower.write(angleL,  servoSpdLower, false);
	servoBase.write(_armRot,  servoSpdBase,  false);
	servoHead.write(_handRot, servoSpdHead,  false);
}

void UF_uArm::detachServo(int servoID)
{
    switch(servoID)
    {
        case SERVO_LOWER:
        	servoLower.detach();
        	break;
        case SERVO_UPPER:
        	servoUpper.detach();
        	break;
        case SERVO_BASE:
        	servoBase.detach();
        	break;
       	case SERVO_HEAD:
        	servoHead.detach();
    }
}

void UF_uArm::arretUrgent()
{
    servoLower.detach();
    servoUpper.detach();
    servoBase.detach();
    servoHead.detach();
}

void UF_uArm::reset()
{
    setPositionAngle(SERVO_LOWER, lowerPos);
    setPositionAngle(SERVO_UPPER, upperPos);
    setPositionAngle(SERVO_BASE,  basePos); 
    setPositionAngle(SERVO_HEAD,  headPos);
}

void UF_uArm::pumpOn()
{
    digitalWrite(PUMP_EN, HIGH);
}

void UF_uArm::pumpOff()
{
    digitalWrite(PUMP_EN, LOW);
}

void UF_uArm::valveOn()
{
    digitalWrite(VALVE_EN, LOW);
}

void UF_uArm::valveOff()
{
    digitalWrite(VALVE_EN, HIGH);
}

void UF_uArm::setServoSpeed(int servoID, int servoSpeed)
{
    // 0=full speed, 1-255 slower to faster
    switch (servoID)
    {
	    case SERVO_LOWER:
		    servoSpdLower = servoSpeed;
		    break;
	    case SERVO_UPPER:
		    servoSpdUpper = servoSpeed;
		    break;
	    case SERVO_BASE:
		    servoSpdBase = servoSpeed;
		    break;
	    case SERVO_HEAD:
		    servoSpdHead = servoSpeed;
    }
}

void UF_uArm::setPositionAngle(int servoID, int angle)
{
    switch (servoID)
    {
	    case SERVO_LOWER:
		    servoLower.write(angle);
		    break;
	    case SERVO_UPPER:
		    servoUpper.write(angle);
		    break;	    
		case SERVO_BASE:
		    servoBase.write(angle);
		    break;
	    case SERVO_HEAD:
		    servoHead.write(angle);
    }
}

int UF_uArm::getPositionAngle(int servoID)
{
    switch (servoID)
    {
	    case SERVO_LOWER:
		    servoLower.read();
		    break;
	    case SERVO_UPPER:
		    servoUpper.read();
		    break;
	    case SERVO_BASE:
		    servoBase.read();
		    break;
	    case SERVO_HEAD: 
		    servoHead.read();
		    break;
	    default :
		    return 0;
    }
}

