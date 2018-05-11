#ifndef lib_uArm_h
#define lib_uArm_h

#include <Arduino.h>
// from https://github.com/netlabtoolkit/VarSpeedServo
#include <VarSpeedServo.h>

/****************  Macro definitions  ****************/
#define ARM_A                   148    // upper arm
#define ARM_B                   160    // lower arm
#define ARM_2AB                 47360  // 2*A*B
#define ARM_A2                  21904  // A^2
#define ARM_B2                  25600  // B^2
#define ARM_A2B2                47504  // A^2 + B^2
#define ARM_STRETCH_MIN         0
#define ARM_STRETCH_MAX         210
#define ARM_HEIGHT_MIN          -180
#define ARM_HEIGHT_MAX          150
#define ARM_ROTATION_MIN        -90
#define ARM_ROTATION_MAX        90
#define HAND_ROTATION_MIN       -90
#define HAND_ROTATION_MAX       90
#define HAND_ANGLE_OPEN         25
#define HAND_ANGLE_CLOSE        70
#define FIXED_OFFSET_L          18
#define FIXED_OFFSET_R          36
#define D150A_SERVO_MIN_PUL     535
#define D150A_SERVO_MAX_PUL     2415
#define D009A_SERVO_MIN_PUL     600
#define D009A_SERVO_MAX_PUL     2550
#define SAMPLING_DEADZONE       2
#define INIT_POS_L              37
#define INIT_POS_R              25
#define BTN_TIMEOUT_1000        1000
#define BTN_TIMEOUT_3000        3000
#define CATCH					0x01
#define RELEASE					0x02
#define CALIBRATION_FLAG		0xEE
#define SERVO_MAX				605
#define SERVO_MIN				80
#define MEMORY_SERVO_PER		335   //  eeprom: (1024 - 3 - 14)/3=335
#define DATA_FLAG				255
#define BUFFER_OUTPUT			5
/*****************  Port definitions  *****************/
#define BTN_D4                  4     //
#define BTN_D7                  7     //
#define BUZZER                  3     //
#define LIMIT_SW                2     // Limit Switch
#define PUMP_EN       			6
#define VALVE_EN      			5
#define SERVO_HEAD   			10
#define SERVO_BASE   			11
#define SERVO_UPPER  			12
#define SERVO_LOWER  			13

class UF_uArm {

    public : 
	UF_uArm();
	void init(void);
	void pumpOn(void);
    void pumpOff(void);
    void valveOn(void);
    void valveOff(void);
	void reset(void);
	void arretUrgent(void);
	void detachServo(int servoID);
	void setServoSpeed(int servoID, int servoSpeed);	   	
	void setPositionAngle(int servoID, int angle);
	int  getPositionAngle(int servoID);
	void setPosition(double _stretch, double _height, int _armRot, int _handRot);

    private:
	int lowerPos;
	int upperPos; 
	int basePos; 
	int headPos;

	int servoSpdLower;
	int servoSpdUpper;
	int servoSpdBase;
	int servoSpdHead;

	VarSpeedServo servoLower;
	VarSpeedServo servoUpper;
	VarSpeedServo servoBase;
	VarSpeedServo servoHead;
};

#endif /* lib_uArm */

