/******************************************************************
*   Inverse Kinematics code to control a (modified) 
*   LynxMotion AL5D robot arm using a PS2 controller.
*
*   Original IK code by Oleg Mazurov:
*       www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
*
*   Great intro to IK, with illustrations:
*       github.com/EricGoldsmith/AL5D-BotBoarduino-PS2/blob/master/Robot_Arm_IK.pdf
*
*   Revamped to use BotBoarduino microcontroller:
*       www.lynxmotion.com/c-153-botboarduino.aspx
*   Arduino Servo library:
*       arduino.cc/en/Reference/Servo
*   and PS2X controller library:
*       github.com/madsci1016/Arduino-PS2X
*
*   PS2 Controls
*       Right Joystick L/R: Gripper tip X position (side to side)
*       Right Joystick U/D: Gripper tip Y position (distance out from base center)
*       R1/R2 Buttons:      Gripper tip Z position (height from surface)
*       Left Joystick L/R   Wrist Rotate (if installed)
*       Left Joystick U/D:  Wrist angle
*       L1/L2 Buttons:      Gripper close/open
*       X:                  Gripper fully open
*
*   Eric Goldsmith
*   www.ericgoldsmith.com
*
*   Version history
*       0.1 Initial port of code to use Arduino Server Library
*
*       0.2 Added PS2 controls
*
*   Current Version:
*       https://github.com/EricGoldsmith/AL5D-BotBoarduino-PS2
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

//#define DEBUG             // Uncomment to turn on debugging output
#define CYL_IK              // Apply only 2D, or cylindrical, kinematics. The X-axis component is
                            // removed from the equations by fixing it at 0. The arm position is
                            // calculated in the Y and Z planes, and simply rotates around the base.
//#define WRIST_ROTATE      // Uncomment if wrist rotate hardware is installed

// Arm dimensions (mm). Standard AL5D arm, but with longer arm segments
//#define BASE_HGT 80.9625    // Base height 3.1875"
#define BASE_HGT 105.9625   // hack to get gripper to touch surface - need to fix
#define HUMERUS 263.525     // Shoulder-to-elbow "bone" 10.375"
#define ULNA 325.4375       // Elbow-to-wrist "bone" 12.8125"
#define GRIPPER 85.725      // Gripper length 3.375"

// Arduino pin numbers for servo connections
#define BAS_SERVO_PIN 2     // Base servo HS-485HB
#define SHL_SERVO_PIN 3     // Shoulder Servo HS-805HB
#define ELB_SERVO_PIN 4     // Elbow Servo HS-755HB
#define WRI_SERVO_PIN 10    // Wrist servo HS-645MG
#define GRI_SERVO_PIN 11    // Gripper servo HS-422
#ifdef WRIST_ROTATE
#define WRO_SERVO_PIN 12    // Wrist rotate servo HS-485HB
#endif

// Arduino pin numbers for PS2 controller connections
#define PS2_CLK_PIN 9        // Clock
#define PS2_CMD_PIN 7        // Command
#define PS2_ATT_PIN 8        // Attention
#define PS2_DAT_PIN 6        // Data

// Arduino pin number of on-board speaker
#define SPK_PIN 5

// Set physical limits (in degrees) per servo
#define BAS_MIN 0.0
#define BAS_MAX 180.0
#define SHL_MIN 5.0 
#define SHL_MAX 140.0
#define ELB_MIN 15.0
#define ELB_MAX 155.0
#define WRI_MIN 0.0
#define WRI_MAX 180.0
#define GRI_MIN 25.0        // Fully open
#define GRI_MAX 165.0       // Fully closed
#ifdef WRIST_ROTATE
#define WRO_MIN 0.0
#define WRO_MAX 180.0
#endif

#define SERVO_MIDPOINT 90.0 // 90 degrees is the rotation midpoint of each servo

// PS2 controller characteristics
#define JS_MIDPOINT 128     // Numeric value for joystick midpoint
#define JS_DEADBAND 4       // Ignore movement this close to the center position
#define JS_IK_SCALE 100.0   // Divisor for scaling JS output for IK control
#define JS_SCALE 300.00     // Divisor for scaling JS output for raw servo control
#define Z_INCREMENT 1.0     // Change in Z axis per button press
#define G_INCREMENT 2.0     // Change in Gripper position per button press

// Audible feedback sounds
#define TONE_IK_ERROR 500      // Hz
#define TONE_SERVO_LIMIT 1000  // Hz
#define TONE_DURATION 200      // ms
 
// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

// PS2 Controller object
PS2X    Ps2x;

// Servo objects 
Servo   Bas_Servo;
Servo   Shl_Servo;
Servo   Elb_Servo;
Servo   Wri_Servo;
Servo   Gri_Servo;
#ifdef WRIST_ROTATE
Servo   Wro_Servo;
#endif

// Park positions
#define PARK_MIDPOINT 1     // Servos at midpoints
#define PARK_READY 2        // Servos at Ready To Run positions

// Ready To Run arm position
#ifdef CYL_IK   // 2D kinematics
#define READY_X 90.0        // degrees. 90 = straight
#else           // 3D kinematics
#define READY_X 0.0         // mm. 0 = straight
#endif
#define READY_Y 200.0       // mm
#define READY_Z 200.0       // mm
#define READY_GA 0.0        // degrees. 0 = horizontal
#define READY_G SERVO_MIDPOINT  //degrees. 90 degrees is mid-way open
#ifdef  WRIST_ROTATE
#define READY_WR SERVO_MIDPOINT // degrees. 90 degrees is horizontal
#endif

// Global variables for arm position, and initial settings
float X = READY_X;          // Left/right from base centerline.
float Y = READY_Y;          // Away (out) from base center.
float Z = READY_Z;          // Up from surface.
float GA = READY_GA;        // Gripper angle, in degrees. 
float G = READY_G;          // Gripper jaw opening. 
#ifdef  WRIST_ROTATE
float WR = READY_WR;        // Wrist Rotate.
#endif

 
void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
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

    // Setup PS2 controller pins and settings and check for error
    //  GamePad(clock, command, attention, data, Pressures?, Rumble?)
    byte    ps2_stat;
    do {
        ps2_stat = Ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN, true, true);
#ifdef DEBUG
        if (ps2_stat == 1)
            Serial.println("No controller found. Re-trying ...");
#endif
    } while (ps2_stat == 1);
 
#ifdef DEBUG
    switch (ps2_stat) {
        case 0:
            Serial.println("Found Controller, configured successfully.");
            break;
        case 2:
            Serial.println("Controller found but not accepting commands.");
            break;
        case 3:
            Serial.println("Controller refusing to enter 'Pressures' mode, may not support it. ");      
            break;
    }
#endif

    servo_park(PARK_READY);

#ifdef DEBUG
    Serial.println("Start");
#endif

//  delay(500);
}
 
void loop()
{
    // Store desired position in tmp variables until confirmed by set_arm() logic
    float x_tmp = X;
    float y_tmp = Y;
    float z_tmp = Z;
    float ga_tmp = GA;
    
    // Used to indidate whether an input occurred that can move the arm
    boolean arm_move = false;

    Ps2x.read_gamepad();        //read controller

    // Read the left and right joysticks and translate the 
    // normal range of values (0-255) to zero-centered values (-128 - 128)
    int ly_trans = JS_MIDPOINT - Ps2x.Analog(PSS_LY);
    int lx_trans = Ps2x.Analog(PSS_LX) - JS_MIDPOINT;
    int ry_trans = JS_MIDPOINT - Ps2x.Analog(PSS_RY);
    int rx_trans = Ps2x.Analog(PSS_RX) - JS_MIDPOINT;

#ifdef CYL_IK   // 2D kinematics
    // Base Position (in degrees)
    // Restrict to MIN/MAX range of servo
    if (abs(rx_trans) > JS_DEADBAND) {
        X += (rx_trans / JS_SCALE);
        X = constrain(X, BAS_MIN, BAS_MAX);
        Bas_Servo.write(X);
    }
#else           // 3D kinematics
    // X Position (in mm)
    // Can be positive or negative, so no range checks needed
    if (abs(rx_trans) > JS_DEADBAND) {
        x_tmp += (rx_trans / JS_IK_SCALE);
        arm_move = true;
    }
#endif

    // Y Position (in mm)
    // Can only be positive, so enforce lower bound
    if (abs(ry_trans) > JS_DEADBAND) {
        y_tmp += (ry_trans / JS_IK_SCALE);
        y_tmp = max(y_tmp, 0);
        arm_move = true;
    }

    // Z Position (in mm)
    // Can only be positive, so enforce lower bound
    if (Ps2x.Button(PSB_R1) || Ps2x.Button(PSB_R2)) {
        if (Ps2x.Button(PSB_R1))
            z_tmp += Z_INCREMENT;   // up
        else {
            z_tmp -= Z_INCREMENT;   // down
            z_tmp = max(z_tmp, 0);
        }
        arm_move = true;
    }

    // Gripper angle (in degrees)
    // Restrict to MIN/MAX range of servo
    if (abs(ly_trans) > JS_DEADBAND) {
        ga_tmp += (ly_trans / JS_IK_SCALE);
        // Since gripper angle is relative to arm (i.e. 0 = horizontal), add 90 when comparing to servo limits
        ga_tmp = constrain((ga_tmp + SERVO_MIDPOINT), WRI_MIN, WRI_MAX);
        arm_move = true;
    }

    // Gripper jaw position (in degrees - determines width of jaw opening)
    // Restrict to MIN/MAX range of servo
    if (Ps2x.Button(PSB_L1) || Ps2x.Button(PSB_L2)) {
        if (Ps2x.Button(PSB_L1))
            G += G_INCREMENT;   // close
        else
            G -= G_INCREMENT;   // open

        G = constrain(G, GRI_MIN, GRI_MAX);
        Gri_Servo.write(G);
    }

    // Fully open gripper
    if (Ps2x.Button(PSB_BLUE)) {
        G = GRI_MIN;
        Gri_Servo.write(G);
    }
    
#ifdef WRIST_ROTATE
    // Wrist rotate (in degrees)
    // Restrict to MIN/MAX range of servo
    if (abs(lx_trans) > JS_DEADBAND) {
        WR += (lx_trans / JS_SCALE);
        WR = constrain(WR, WRO_MIN, WRO_MAX);
        Wro_Servo.write(WR);
    }
#endif

    // Only call this if arm motion is needed.
    if (arm_move) {
#ifdef CYL_IK   // 2D kinematics
        if (set_arm(0, y_tmp, z_tmp, ga_tmp) == 0) {
            // If the arm was positioned successfully, record
            // the new vales. Otherwise, ignore them.
            Y = y_tmp;
            Z = z_tmp;
            GA = ga_tmp;
        } else
            // Send audible feedback of IK error
            tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
            
#else           // 3D kinematics
        if (set_arm(x_tmp, y_tmp, z_tmp, ga_tmp) == 0) {
            // If the arm was positioned successfully, record
            // the new vales. Otherwise, ignore them.
            X = x_tmp;
            Y = y_tmp;
            Z = z_tmp;
            GA = ga_tmp;
        } else
            // Send audible feedback of IK error
            tone(SPK_PIN, TONE_IK_ERROR, TONE_DURATION);
#endif

        // Reset the flag
        arm_move = false;
    }

    delay(10);
 }
 
// Arm positioning routine utilizing inverse kinematics
// z is height, y is distance from base center out, x is side to side. y, z can only be positive
// If resulting arm position is physically unreachable, return error (1). Otherwise, return success (0)
int set_arm(float x, float y, float z, float grip_angle_d)
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
    float a1 = atan2(wrist_z, wrist_y);
    
    // s_w angle to humerus
    float a2 = acos(((hum_sq - uln_sq) + s_w) / (2 * HUMERUS * s_w_sqrt));
    
    // Shoulder angle
    float shl_angle_r = a1 + a2;
    // If result is NAN or Infinity, the desired arm position is not possible
    if (isnan(shl_angle_r) || isinf(shl_angle_r))
        return 1;    // Return error
    float shl_angle_d = degrees(shl_angle_r);
    
    // Elbow angle
    float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));
    // If result is NAN or Infinity, the desired arm position is not possible
    if (isnan(elb_angle_r) || isinf(elb_angle_r))
        return 1;    // Return error
    float elb_angle_d = degrees(elb_angle_r);
    float elb_angle_dn = -(180.0 - elb_angle_d);
    
    // Wrist angle
    float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;
 
    // Calculate servo angles
    float bas_pos_tmp = SERVO_MIDPOINT + degrees(bas_angle_r);
    float shl_pos_tmp = SERVO_MIDPOINT + (shl_angle_d - 90.0);
    float elb_pos_tmp = SERVO_MIDPOINT - (elb_angle_d - 90.0);
    float wri_pos_tmp = SERVO_MIDPOINT + wri_angle_d;
    
    // Constrain servo positions
    float bas_pos = constrain(bas_pos_tmp, BAS_MIN, BAS_MAX);
    float shl_pos = constrain(shl_pos_tmp, SHL_MIN, SHL_MAX);
    float elb_pos = constrain(elb_pos_tmp, ELB_MIN, ELB_MAX);
    float wri_pos = constrain(wri_pos_tmp, WRI_MIN, WRI_MAX);
    
    // If we hit any servo limits, send audible feedback
    if ((bas_pos_tmp != bas_pos) || (shl_pos_tmp != shl_pos) || (elb_pos_tmp != elb_pos) || (wri_pos_tmp != wri_pos))
        tone(SPK_PIN, TONE_SERVO_LIMIT, TONE_DURATION);
 
    // Servo output
#ifdef CYL_IK   // 2D kinematics
    // Do not control base servo
#else           // 3D kinematics
    Bas_Servo.write(bas_pos);
#endif
    Shl_Servo.write(shl_pos);
    Elb_Servo.write(elb_pos);
    Wri_Servo.write(wri_pos);

#ifdef DEBUG
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("  Y: ");
    Serial.print(y);
    Serial.print("  Z: ");
    Serial.print(z);
    Serial.print("  WA: ");
    Serial.print(grip_angle_d);
    Serial.println();
    Serial.print("Base Pos: ");
    Serial.print(bas_pos);
    Serial.print("  Shld Pos: ");
    Serial.print(shl_pos);
    Serial.print("  Elbw Pos: ");
    Serial.print(elb_pos);
    Serial.print("  Wrst Pos: ");
    Serial.println(wri_pos);
    Serial.print("bas_angle_d: ");
    Serial.print(degrees(bas_angle_r));  
    Serial.print("  shl_angle_d: ");
    Serial.print(shl_angle_d);  
    Serial.print("  elb_angle_d: ");
    Serial.print(elb_angle_d);
    Serial.print("  wri_angle_d: ");
    Serial.println(wri_angle_d);
    Serial.println();
#endif

    return 0;
}
 
// Move servos to parking position
void servo_park(int park_type)
{
    switch (park_type) {
        // All servos at midpoint
        case PARK_MIDPOINT:
            Bas_Servo.write(SERVO_MIDPOINT);
            Shl_Servo.write(SERVO_MIDPOINT);
            Elb_Servo.write(SERVO_MIDPOINT);
            Wri_Servo.write(SERVO_MIDPOINT);
            Gri_Servo.write(SERVO_MIDPOINT);
#ifdef WRIST_ROTATE
            Wro_Servo.write(SERVO_MIDPOINT);
#endif
            break;
        
        // Ready To Run position
        case PARK_READY:
#ifdef CYL_IK   // 2D kinematics
            set_arm(0.0, READY_Y, READY_Z, READY_GA);
            Bas_Servo.write(READY_X);
#else           // 3D kinematics
            set_arm(READY_X, READY_Y, READY_Z, READY_GA);
#endif
            Gri_Servo.write(READY_G);
#ifdef WRIST_ROTATE
            Wro_Servo.write(READY_WR);
#endif
            break;
    }

    return;
}


