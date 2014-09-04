#ifndef LOCALLIBRARY_H
#define LOCALLIBRARY_H

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
#include <ros.h>
#include "IOpins.h"

// For serial communications each datapacket must start with this byte
#define startbyte 0x0F
 
/*
 The provided sample code requires external controllers to use the I²C data packet protocol listed below.
 Each data packet starts with a start byte (0x0F, decimal 15) as a simple means of error checking.
 
 The T'REX controller joins the I²C bus as a slave with the I²C address 0x07 (decimal address 7).
 This address can be re-defined at the start of the sample code if required.
 All integers are sent as high byte and then low byte.
 
 
 ===== Command data packet - 27 bytes =====
 byte  Start 0x0F
 byte  PWMfreq 1=31.25kHz    7=30.5Hz
 int   lmspeed
 byte  lmbrake
 int   rmspeed
 byte  rmbrake
 int   servo 0
 int   servo 1
 int   servo 2
 int   servo 3
 int   servo 4
 int   servo 5
 byte  devibrate                       default=50 (100mS)   0-255
 int   impact sensitivity  0 to 1023   default=50
 int   low battery  550 to 30000       5.5V to 30V
 byte  I²C address
 byte  I²C clock frequency: 0=100kHz   1=400kHz
 
 
 ===== Status data packet - 24 bytes =====
 byte  Start 0x0F
 byte  errorflag
 int   battery voltage
 int   left  motor current
 int   left  motor encoder
 int   right motor current
 int   right motor encoder
 int   X-axis
 int   Y-axis
 int   Z-axis
 int   deltaX
 int   deltaY
 int   deltaZ
 */

class ControllerInterface
{
public:
    ControllerInterface(ros::NodeHandle *nodeHandle);
    
    void Accelerometer();
    void EmptyBuffer();
    void DiagnosticMode();
    void Encoders();
    void Motors();
    void MotorBeep(byte beeps);
    void RCmode();
    void Servos();
    void Shutdown();
    
    static byte mode;             // mode=0: I2C / mode=1: Radio Control / mode=2: Bluetooth / mode=3: Shutdown
    // left and right brakes - non zero values enable brake
    static byte lmbrake;
    static byte rmbrake;
    static byte devibrate;        // number of 2mS intervals to wait after an impact has occured before a new impact can be recognized
    static int sensitivity;       // minimum magnitude required to register as an impact
    // left and right motor speeds -255 to +255
    static int lmspeed;
    static int rmspeed;
    static int lmcur;             // left right motor current
    static int rmcur;             // right motor current
    // left and right encoder values
    static int lmenc;
    static int rmenc;
    // X, Y, Z accelerometer readings
    static int xaxis;
    static int yaxis;
    static int zaxis;
    // X, Y, Z impact readings
    static int deltx;
    static int delty;
    static int deltz;
    
    static int servopos[6];       // array stores position data for up to 6 servos

    
private:
    ros::NodeHandle *mNodeHandle;
    int magnitude;         // impact magnitude
    byte RCdeadband;       // RCsignal can vary this much from 1500uS without controller responding
    byte servopin[6];      // array stores IO pin for each servo
    Servo servo[6];        // create 6 servo objects as an array
};

#endif