/******************************************************************
*	Inverse Kinematics code to control a (modified) 
*	LynxMotion AL5D robot arm using a PS2 controller.
*
*	Original IK code by Oleg Mazurov:
*		www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
*
*	Great intro to IK, with illustrations:
*		github.com/EricGoldsmith/AL5D-BotBoarduino-PS2/blob/master/Robot_Arm_IK.pdf
*
*	Revamped to use BotBoarduino microcontroller:
*		www.lynxmotion.com/c-153-botboarduino.aspx
*	Arduino Servo library:
*		arduino.cc/en/Reference/Servo
*	and PS2X controller library:
*		github.com/madsci1016/Arduino-PS2X
*
* 	PS2 Controls
*		Right Joystick L/R:	Gripper tip X position (side to side)
*		Right Joystick U/D:	Gripper tip Y position (distance out from base center)
*		R1/R2 buttons:		Gripper tip Z position (height from surface)
*		Left Joystick U/D:	Wrist angle
*
*	Eric Goldsmith
*	www.ericgoldsmith.com
*
* 	Version history
*		0.1 Initial port of code to use Arduino Server Library
*
*		0.2 Added PS2 controls
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* <http://www.gnu.org/licenses/>
*  
******************************************************************/

#include <Servo.h>
#include <PS2X_lib.h>

//#define DEBUG				// Uncomment to turn on debugging output
//#define WRIST_ROTATE		// Uncomment if wrist rotate hardware is installed

// Servo control for a modified AL5D arm
// Modifications entail lengthened arm segments
 
// Arm dimensions (mm)
#define BASE_HGT 67.31      //base height 2.65"
#define HUMERUS 260.35      //shoulder-to-elbow "bone" 10.25"
#define ULNA 327.025        //elbow-to-wrist "bone" 12.875"
#define GRIPPER 85.725      //gripper length 3.375"
 
// Arduino pin numbers for servo connections
#define BAS_SERVO_PIN 2		// Base servo HS-485HB
#define SHL_SERVO_PIN 3		// Shoulder Servo HS-805HB
#define ELB_SERVO_PIN 4		// Elbow Servo HS-755HB
#define WRI_SERVO_PIN 10	// Wrist servo HS-645MG
#define GRI_SERVO_PIN 11	// Gripper servo HS-422

#ifdef WRIST_ROTATE
#define WRO_SERVO_PIN 12	// Wrist rotate servo HS-485HB
#endif

#define SERVO_MIDPOINT 90	// 90 degrees is the rotation midpoint of each servo

// Set boundary constraints (in degrees) per servo
#define BAS_MIN 0
#define BAS_MAX 180
#define SHL_MIN 5 
#define SHL_MAX 155
#define ELB_MIN 15
#define ELB_MAX 180
#define WRI_MIN 0
#define WRI_MAX 180
#define GRI_MIN 0
#define GRI_MAX 180

#ifdef WRIST_ROTATE
#define GRI_MIN 0
#define GRI_MAX 180
#endif

// Define pin numbers for PS2 controller connections
#define PS2_CLK 9
#define PS2_CMD 7
#define PS2_ATT 8
#define PS2_DAT 6

// Joystick characteristics
#define JS_MIDPOINT 128		// Numeric value for joystick midpoint
#define JS_DEADBAND 4		// Ignore movement this close to the center position
#define JS_SCALE_FACTOR 100	// Divisor for scaling JS output
#define Z_INCREMENT			// Change in Z axis per button press
 
// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

// PS2 Controller object
PS2X	Ps2x;
byte	ps2_stat;

// Servo objects 
Servo	Bas_Servo;
Servo	Shl_Servo;
Servo	Elb_Servo;
Servo	Wri_Servo;
Servo	Gri_Servo;

#ifdef WRIST_ROTATE
Servo	Wro_Servo;
#endif

// Global values for arm position
float X = 0				// mm - left/right from base centerline. 0 = straight
float Y = 100			// mm - away (out) from base center
float Z = 100			// mm - up from surface
float Grip_Angle = 90;	// degrees - wrist angle. 90 = horizontal
 
void setup()
{
#ifdef DEBUG
	Serial.begin(115200);
#endif

	// Setup PS2 controller pins and settings and check for error
	// 	GamePad(clock, command, attention, data, Pressures?, Rumble?)
	ps2_stat = Ps2x.config_gamepad(PS2_CLK,PS2_CMD,PS2_ATT,PS2_DAT, true, true);
 
#ifdef DEBUG
	if (ps2_stat == 0)
		Serial.println("Found Controller, configured successfully.");

	else if (ps2_stat == 1)
		Serial.println("No controller found.");

	else if (ps2_stat == 2)
		Serial.println("Controller found but not accepting commands.");

	else if (ps2_stat == 3)
		Serial.println("Controller refusing to enter 'Pressures' mode, may not support it. ");      
#endif

	// Attach to the servos
	Bas_Servo.attach(BAS_SERVO_PIN);
	Shl_Servo.attach(SHL_SERVO_PIN);
	Elb_Servo.attach(ELB_SERVO_PIN);
	Wri_Servo.attach(WRI_SERVO_PIN);
	Gri_Servo.attach(GRI_SERVO_PIN);
#ifdef WRIST_ROTATE
	Wro_Servo.attach(WRO_SERVO_PIN);
#endif

	servo_park();
	
#ifdef DEBUG
	Serial.println("Start");
#endif
	delay(500);
}
 
void loop()
{

	if (ps2_stat == 1) //skip loop if no controller found
		return; 

	Ps2x.read_gamepad();          //read controller

	// Read the left and right joysticks and translate the 
	// normal range of values (0-255) to zero-centered values (-128 - 128)
	byte ly_trans = JS_MIDPOINT - ps2x.Analog(PSS_LY);
	byte lx_trans = ps2x.Analog(PSS_LX) - JS_MIDPOINT;
	byte ry_trans = JS_MIDPOINT - ps2x.Analog(PSS_RY);
	byte rx_trans = ps2x.Analog(PSS_RX) - JS_MIDPOINT;

	// X Position
	// Can be positive or negative, so no range checks needed
	// TODO: incorporate current servo position feedback
	if (abs(lx_trans) > DEADBAND)
		X += (lx_trans / JS_SCALE_FACTOR);

	// Y Position
	// Can only be positive, so enforce lower bound
	// TODO: incorporate current servo position feedback
	if (abs(ly_trans) > DEADBAND)
		Y += max((ly_trans / JS_SCALE_FACTOR), 0);

	// Z Position
	// Can only be positive, so enforce lower bound
	// TODO: incorporate current servo position feedback
	if (Ps2x.Button(PSB_R1) || Ps2x.Button(PSB_R2)) {
		if (Ps2x.Button(PSB_R1))
			Z += Z_INCREMENT;				// up
		else
			Z = max((Z - Z_INCREMENT), 0);	// down
	}
	
	set_arm(X, Y, Z, Grip_Angle);

	delay(50);
 }
 
// Arm positioning routine utilizing inverse kinematics
// z is height, y is distance from base center out, x is side to side. y,z can only be positive
void set_arm(float x, float y, float z, float grip_angle_d)
{
	//grip angle in radians for use in calculations
	float grip_angle_r = radians(grip_angle_d);    
  
	// Base angle and radial distance from x,y coordinates
	float bas_angle_r = atan2(x, y);
	float rdist = sqrt((x * x) + (y * y));
  
	// rdist is y coordinate for the arm
	y = rdist;
	
	// Grip offsets calculated based on grip angle
	float grip_off_z = (sin( grip_angle_r)) * GRIPPER;
	float grip_off_y = (cos( grip_angle_r)) * GRIPPER;
	
	// Wrist position
	float wrist_z = (z - grip_off_z) - BASE_HGT;
	float wrist_y = y - grip_off_y;
	
	// Shoulder to wrist distance (AKA sw)
	float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
	float s_w_sqrt = sqrt(s_w);
	
	// s_w angle to ground
	//float a1 = atan2(wrist_y, wrist_z);
	float a1 = atan2(wrist_z, wrist_y);
	
	// s_w angle to humerus
	float a2 = acos(((hum_sq - uln_sq) + s_w) / (2 * HUMERUS * s_w_sqrt));
	
	// shoulder angle
	float shl_angle_r = a1 + a2;
	float shl_angle_d = degrees(shl_angle_r);
	
	// elbow angle
	float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));
	float elb_angle_d = degrees(elb_angle_r);
	float elb_angle_dn = -(180.0 - elb_angle_d);
	
	// wrist angle
	float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;
 
 	// Calculate and constrain servo positions
	float bas_pos = constrain(SERVO_MIDPOINT - degrees(bas_angle_r), BAS_MIN, BAS_MAX);
 	float shl_pos = constrain(SERVO_MIDPOINT + (shl_angle_d - 90.0), SHL_MIN, SHL_MAX);
	float elb_pos = constrain(SERVO_MIDPOINT - (elb_angle_d - 90.0), ELB_MIN, ELB_MAX);
	float wri_pos = constrain(SERVO_MIDPOINT + wri_angle_d, WRI_MIN, WRI_MAX);
 
  	// Servo output
  	Bas_Servo.write(bas_pos);
	Shl_Servo.write(shl_pos);
	Elb_Servo.write(elb_pos);
	Wri_Servo.write(wri_pos); 
}
 
// Move servos to parking position
void servo_park()
{

	set_arm(X, Y, Z, Grip_Angle);
/*
	Bas_Servo.write(SERVO_MIDPOINT);
	Shl_Servo.write(SERVO_MIDPOINT);
	Elb_Servo.write(SERVO_MIDPOINT);
	Wri_Servo.write(SERVO_MIDPOINT);
*/	
	Gri_Servo.write(SERVO_MIDPOINT);
#ifdef WRIST_ROTATE
	Wro_Servo.write(SERVO_MIDPOINT);
#endif

	return;
}
 
void zero_x()
{
	for (double yaxis = 150.0; yaxis < 356.0; yaxis += 1) {
		set_arm(0, yaxis, 127.0, 0);
		delay(10);
	}
	
	for (double yaxis = 356.0; yaxis > 150.0; yaxis -= 1) {
		set_arm(0, yaxis, 127.0, 0);
		delay(10);
	}
}
 
// Moves arm in a straight line
void line()
{
	for (double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5) {
		set_arm(xaxis, 250, 100, 0);
		delay(10);
	}
	
	for (float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5) {
		set_arm(xaxis, 250, 100, 0);
		delay(10);
	}
}
 
void circle()
{
	#define RADIUS 80.0
	float zaxis, yaxis;

	for (float angle = 0.0; angle < 360.0; angle += 1.0) {
		yaxis = RADIUS * sin(radians(angle)) + 300;
		zaxis = RADIUS * cos(radians(angle)) + 300;
		set_arm(0, yaxis, zaxis, 0);
		delay(5);
	}
}

