// 
// TREX_controller 
//
// DAGU T'REX Motor Controller Firmware
// Developed with [embedXcode](http://embedXcode.weebly.com)
// 
// Author	 	Donald Poole
// 				DAGU Hi-Tech Electronics
//
// Date			8/29/14 11:20 PM
// Version		<#version#>
// 
// Copyright	© Donald Poole, 2014
// License		<#license#>
//
// See			ReadMe.txt for references
//

// Core library for code-sense
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"   
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad MSP430, Stellaris and Tiva, Experimeter Board FR5739 specific
#include "Energia.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif

// Include application, user and local libraries
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "IOpins.h"
#include "LocalLibrary.h"

// For serial communications each datapacket must start with this byte
#define startbyte       0x0F
#define MAX_MOTOR_SPEED 0xFF

// Comment out to create release firmware
#define DEBUG

#include <ros.h>
#include <ros_dagu_trex_controller/status.h>
#include <geometry_msgs/Twist.h>

// Define variables and constants
//
// Brief    I2C slave address
// Details  Set the I2C address of the controller
//
byte I2Caddress;

// Brief    I2C clock frequency can be 100kHz(default) or 400kHz
// Details
byte i2cfreq;

// Brief    The low threshold of the battery voltage
// Details  If the battery falls below this threshold, the controller shutsdown
//
int lowbat = 550;
int volts;             // battery voltage*10 (accurate to 1 decimal place)
byte errorflag;        // non zero if bad data packet received
byte pwmfreq;          // value from 1-7

// Brief    ROS Node Handle for Publishers and Subscibers
// Details  Node handle, which allows our program to create publishers and
//          subscribers. The node handle also takes care of serial port
//          communications.
//
ros::NodeHandle nh;

//
// Brief	Interface to the T'REX Controller Hardware Components
// Details	Interface to acces the various hardware components on the T'REX
//          controller to set or get data.
//
ControllerInterface ctrlIf( &nh );

// Brief    The Current operating mode of the controller
// Details  mode=0: I2C / mode=1: Radio Control / mode=2: Bluetooth / mode=3: Shutdown
//
byte ControllerInterface::mode = 0;

// Brief
// Details
//
byte ControllerInterface::lmbrake = 0;

// Brief
// Details
//
byte ControllerInterface::rmbrake = 0;

// Brief
// Details
//
byte ControllerInterface::devibrate = 50;

// Brief
// Details
//
int ControllerInterface::sensitivity = 50;

// Brief    left motor speeds -255 to +255
// Details
//
int ControllerInterface::lmspeed = 0;

// Brief    right motor speeds -255 to +255
// Details
//
int ControllerInterface::rmspeed = 0;

// Brief    left motor current
// Details
//
int ControllerInterface::lmcur = 0;

// Breif    right motor current
// Details
//
int ControllerInterface::rmcur = 0;

// Breif    left encoder values
// Details
//
int ControllerInterface::lmenc = 0;

// Breif    right encoder values
// Details
//
int ControllerInterface::rmenc = 0;

// Brief X, Y, Z accelerometer readings
// Details
//
int ControllerInterface::xaxis = 0;
int ControllerInterface::yaxis = 0;
int ControllerInterface::zaxis = 0;

// Brief X, Y, Z impact readings
// Details
//
int ControllerInterface::deltx = 0;
int ControllerInterface::delty = 0;
int ControllerInterface::deltz = 0;
    
// Brief array stores position data for up to 6 servos
// Details
//
int ControllerInterface::servopos[6];

// Brief
// Details
//
char tempStr[54];
void twistCb(const geometry_msgs::Twist& twist_msg)
{
    // Stop the vehicle when the stick is centered
    if(twist_msg.linear.x == 0 && twist_msg.linear.y == 0)
    {
        ControllerInterface::lmspeed = ControllerInterface::rmspeed = 0;
    }
    // Directly forward // Directly backwards
    else if((twist_msg.linear.y == 1.0 || twist_msg.linear.y == -1.0) &&
            twist_msg.linear.x == 0.0)
    {
        ControllerInterface::lmspeed = ControllerInterface::rmspeed = round(MAX_MOTOR_SPEED * twist_msg.linear.y);
#ifdef DEBUG
        sprintf(tempStr, "L%i, R%i", ControllerInterface::lmspeed, ControllerInterface::rmspeed);
        nh.loginfo( tempStr );
#endif
    }
    // Turning to the left
    else if(twist_msg.linear.x < 0)
    {
        ControllerInterface::lmspeed = round(MAX_MOTOR_SPEED * twist_msg.linear.y);
        if(twist_msg.linear.y > 0)
        {
            ControllerInterface::rmspeed = -round(MAX_MOTOR_SPEED * twist_msg.linear.x);
        }
        else
        {
            ControllerInterface::rmspeed = round(MAX_MOTOR_SPEED * twist_msg.linear.x);
        }
#ifdef DEBUG
        sprintf(tempStr, "L%i, R%i", ControllerInterface::lmspeed, ControllerInterface::rmspeed);
        nh.loginfo( tempStr );
#endif
    }
    // Turning to the right
    else if(twist_msg.linear.x > 0)
    {
        ControllerInterface::rmspeed = round(MAX_MOTOR_SPEED * twist_msg.linear.y);
        if(twist_msg.linear.y > 0)
        {
            ControllerInterface::lmspeed = round(MAX_MOTOR_SPEED * twist_msg.linear.x);
        }
        else
        {
            ControllerInterface::lmspeed = -round(MAX_MOTOR_SPEED * twist_msg.linear.x);
        }
#ifdef DEBUG
        sprintf(tempStr, "L%i, R%i", ControllerInterface::lmspeed, ControllerInterface::rmspeed);
        nh.loginfo( tempStr );
#endif
    }
    
    ctrlIf.Motors();
}

// Brief
// Details
//
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistCb);

// Brief    The T'REX ROS Status Message to send periodically
// Details
//
ros_dagu_trex_controller::status status_msg;

// Brief    The T'REX Controller Publisher
// Details  The actual publisher for the statuses of the T'REX controller
//
ros::Publisher status_pub("t_rex/status", &status_msg);

// Brief    Timer used to monitor accelerometer and encoders
// Details
//
unsigned long time = 0;

// Brief    Switch between accelerometer/power readings
// Details  Variable used to alternate between reading accelerometer and power
//          analog inputs
//
byte alternate = 1;

// Brief    Counter to blink the publish the status message
// Details  Counter to publish status messages every 1 second
//
unsigned int publishCounter = 0;

//----------------------- Report Status to ROS System -----------------------//
void PublishStatusMessage()
{
    // Populate the status message
    status_msg.seqId++;
    status_msg.errorFlag = errorflag;
    status_msg.batteryVoltage = volts;
    status_msg.leftMotorCurrent = ControllerInterface::lmcur;
    status_msg.leftMotorEncoder = ControllerInterface::lmenc;
    status_msg.rightMotorCurrent = ControllerInterface::rmcur;
    status_msg.rightMotorEncoder = ControllerInterface::rmenc;
    status_msg.accel_x = ControllerInterface::xaxis;
    status_msg.accel_y = ControllerInterface::yaxis;
    status_msg.accel_z = ControllerInterface::zaxis;
    status_msg.delta_x = ControllerInterface::deltx;
    status_msg.delta_y = ControllerInterface::delty;
    status_msg.delta_z = ControllerInterface::deltz;
    
    // Publish the message
    status_pub.publish( &status_msg );
    
    // Reset erroflag once error has been reported to I²C Master
    errorflag = 0;
}

int led = 13;
//
// Brief	Setup
// Details	Define the pin the LED is connected to
//
// Add setup code 
void setup()
{
    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);
    
    // Initialize the sequence id of the status message
    status_msg.seqId = 0;
    
    // Initialize the ROS node handle for our publisher and subscriber
    nh.initNode();
    nh.advertise( status_pub );
    nh.subscribe( sub );

    //========================================== Choose your desired motor PWM frequency ================================================//
    //                       Note that higher frequencies increase inductive reactance and reduce maximum torque                         //
    //                               Many smaller motors will not work efficiently at higher frequencies                                 //
    //                      The default is 122Hz. This provides relatively low noise and relatively smooth torque                        //
    //                                    This setting can be changed using I2C or Bluetooth                                             //
    //                                                                                                                                   //
    //     Thanks to macegr - http://forum.arduino.cc/index.php?PHPSESSID=n1691l4esq4up52krpcb77bgm1&topic=16612.msg121031#msg121031     //
    //===================================================================================================================================//

    //TCCR2B = TCCR2B & B11111000 | B00000001; pwmfreq = 1;    // set timer 2 divisor to    1 for PWM frequency of  31250.000000000 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000010; pwmfreq = 2;    // set timer 2 divisor to    8 for PWM frequency of   3906.250000000 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000011; pwmfreq = 3;    // set timer 2 divisor to   32 for PWM frequency of    976.562500000 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000100; pwmfreq = 4;    // set timer 2 divisor to   64 for PWM frequency of    488.281250000 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000101; pwmfreq = 5;    // set timer 2 divisor to  128 for PWM frequency of    244.140625000 Hz
      TCCR2B = TCCR2B & B11111000 | B00000110; pwmfreq = 6;    // set timer 2 divisor to  256 for PWM frequency of    122.070312500 Hz
    //TCCR2B = TCCR2B & B11111000 | B00000111; pwmfreq = 7;    // set timer 2 divisor to 1024 for PWM frequency of     30.517578125 Hz


    // all IO pins are input by default on powerup --------- configure motor control pins for output -------- pwm autoconfigures -----------

    pinMode(lmpwmpin, OUTPUT);                            // configure left  motor PWM       pin for output
    pinMode(lmdirpin, OUTPUT);                            // configure left  motor direction pin for output
    pinMode(lmbrkpin, OUTPUT);                            // configure left  motor brake     pin for output
  
    pinMode(rmpwmpin, OUTPUT);                            // configure right motor PWM       pin for output
    pinMode(rmdirpin, OUTPUT);                            // configure right motor direction pin for output
    pinMode(rmbrkpin, OUTPUT);                            // configure right motor brake     pin for output
  
    //----------------------------------------------------- Test for RC inputs ------------------------------------------------------------

    digitalWrite(RCspeedpin, 1);                          // enable weak pullup resistor on input to prevent false triggering
    digitalWrite(RCsteerpin, 1);                          // enable weak pullup resistor on input to prevent false triggering
    delay( 100 );
    int t1 = int( pulseIn(RCspeedpin, HIGH, 30000) );     // read throttle/left stick
    int t2 = int( pulseIn(RCsteerpin, HIGH, 30000) );     // read steering/right stick
    if(t1 > 1000 && t1 < 2000 && t2 > 1000 && t2 < 2000)  // RC signals detected - go to RC mode
    {
        nh.loginfo("(T'REX Controller) Entered RC Mode");
        ControllerInterface::mode = 1;                    // set mode to RC
        ctrlIf.MotorBeep( 3 );                            // generate 3 beeps from the motors to indicate RC mode enabled
    }
    
    ControllerInterface::mode = 2;
    ctrlIf.MotorBeep( 2 ); // ROS Serial Mode
}

//
// Brief	Loop
// Details	Execute the motor controller
//
void loop()
{
    //----------------------------------------------------- Diagnostic mode --------------------------------------------------------------
    /*
    ctrlIf.DiagnosticMode();
    return;
    */
    //----------------------------------------------------- Shutdown mode ----------------------------------------------------------------
    if(ControllerInterface::mode == 3)                            // if battery voltage too low
    {
        ctrlIf.Shutdown();                                        // Shutdown motors and servos
        return;
    }
  
    //----------------------------------------------------- RC Mode -----------------------------------------------------------------------
    if(ControllerInterface::mode == 1)
    {
        ctrlIf.RCmode();                                          // monitor signal from RC receiver and control motors
        return;                                            // I²C, Bluetooth and accelerometer are ignored
    }

    //===================================================== Programmer's Notes ============================================================
    //                                    Detecting impacts requires reasonably accurate timing.                                         //
    //                  As all timers are in use this code uses the micros() function to simulate a timer interrupt.                     //
    //                                                                                                                                   //
    //                      Reading an analog input takes 260uS so reading 3 analog inputs can take about 800uS.                         //
    //                 This code alternates between reading accelerometer data and voltage / current readings every 1mS.                 //
    //                  If you edit this code then be aware that impact detection may be affected if care is not taken.                  //
    //=====================================================================================================================================
  
  
    //----------------------------------------------------- Perform these functions every 1mS ----------------------------------------------
    if(micros()-time > 999)
    {
        // Reset timer
        time = micros();
        // Toggle alternate between 0 and 1
        alternate = alternate^1;
        // Check encoder status every 1mS
        ctrlIf.Encoders();

        //--------------------------------------------------- These functions must alternate as they both take in excess of 780uS ------------
        if( alternate )
        {
            // Monitor accelerometer every second millisecond
            ctrlIf.Accelerometer();
        }
        else
        {
            // Read left motor current sensor and convert reading to mA
            ControllerInterface::lmcur = (analogRead(lmcurpin) - 511) * 48.83;
            // Read right motor current sensor and convert reading to mA
            ControllerInterface::rmcur = (analogRead(rmcurpin) - 511) * 48.83;

            // Read battery level and convert to volts with 2 decimal places
            // (eg. 1007 = 10.07 V)
            volts = analogRead( voltspin ) * 10 / 3.357;
#ifndef DEBUG
            if(volts < lowbat)
            {
                // Change to shutdown mode if battery voltage too low
                ControllerInterface::mode = 3;
            }
#endif
        }
        
        // Send a status message
        if(publishCounter % 999 == 0)
        {
            PublishStatusMessage();
#ifdef DEBUG
            if( alternate )
                digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
            else
                digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
#endif
        }
        publishCounter++;
        
        // Read & Write
        nh.spinOnce();
    }
}
