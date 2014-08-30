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

#include <ros.h>
#include <std_msgs/String.h>


// Define variables and constants
//
// Breif    I2C slave address
// Details  Set the I2C address of the controller
//
byte I2Caddress = 0x07;

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

// Brief    The Current operating mode of the controller
// Details  mode=0: I2C / mode=1: Radio Control / mode=2: Bluetooth / mode=3: Shutdown
//
byte ControllerInterface::mode = 0;
byte ControllerInterface::lmbrake = 0;
byte ControllerInterface::rmbrake = 0;

// Brief
// Details
//
byte ControllerInterface::devibrate = 50;

// Brief
// Details
//
int ControllerInterface::sensitivity = 50;

// left and right motor speeds -255 to +255
int ControllerInterface::lmspeed = 0;
int ControllerInterface::rmspeed = 0;
int ControllerInterface::lmcur = 0;             // left right motor current
int ControllerInterface::rmcur = 0;             // right motor current
// left and right encoder values
int ControllerInterface::lmenc = 0;
int ControllerInterface::rmenc = 0;
// X, Y, Z accelerometer readings
int ControllerInterface::xaxis = 0;
int ControllerInterface::yaxis = 0;
int ControllerInterface::zaxis = 0;
// X, Y, Z impact readings
int ControllerInterface::deltx = 0;
int ControllerInterface::delty = 0;
int ControllerInterface::deltz = 0;
    
int ControllerInterface::servopos[6];       // array stores position data for up to 6 servos

//
// Brief	Interface to the T'REX Controller Hardware Components
// Details	Interface to acces the various hardware components on the T'REX
//          controller to set or get data.
//
ControllerInterface ctrlIf;

// Brief    Timer used to monitor accelerometer and encoders
// Details
//
unsigned long time;

// For serial communications each datapacket must start with this byte
#define startbyte 0x0F

//------------------------------------------------------------------------------- Receive commands from I²C Master -----------------------------------------------
void I2Ccommand(int recvflag)
{
    byte b;                                                                      // byte from buffer
    int i;                                                                       // integer from buffer
    
    do                                                                           // check for start byte
    {
        b = Wire.read();                                                             // read a byte from the buffer
        if(b != startbyte || recvflag != 27)
            errorflag = errorflag | 1;                 // if byte does not equal startbyte or Master request incorrect number of bytes then generate error
    } while(errorflag > 0 && Wire.available() > 0);                                 // if errorflag>0 then empty buffer of corrupt data
    
    if(errorflag > 0)                                                              // corrupt data received
    {
        ctrlIf.Shutdown();                                                                // shut down motors and servos
        return;                                                                    // wait for valid data packet
    }
    //----------------------------------------------------------------------------- valid data packet received ------------------------------

    b = Wire.read();                                                               // read pwmfreq from the buffer
    if(b > 0 && b < 8)                                                               // if value is valid (1-7)
    {
        pwmfreq = b;                                                                 // update pwmfreq
        TCCR2B = TCCR2B & B11111000 | pwmfreq;                                     // change timer 2 clock pre-scaler
    }
    else
    {
        errorflag = errorflag | 2;                                                 // incorrect pwmfreq given
    }
    
    i = Wire.read() * 256 + Wire.read();                                               // read integer from I²C buffer
    if(i > -256 && i < 256)
    {
        ControllerInterface::lmspeed = i;                                                                 // read new speed for   left  motor
    }
    else
    {
        errorflag = errorflag | 4;                                                 // incorrect motor speed given
    }
    ControllerInterface::lmbrake = Wire.read();                                                         // read new left  motor brake status
    
    i = Wire.read() * 256 + Wire.read();                                               // read integer from I²C buffer
    if(i > -256 && i < 256)
    {
        ControllerInterface::rmspeed = i;                                                                 // read new speed for   right motor
    }
    else
    {
        errorflag = errorflag | 4;                                                 // incorrect motor speed given
    }
    ControllerInterface::rmbrake = Wire.read();                                                         // read new right motor brake status
    
    if(errorflag & 4)                                                            // incorrect motor speed / shutdown motors
    {
        ControllerInterface::lmspeed = 0;                                                                 // set left  motor speed to 0
        ControllerInterface::rmspeed = 0;                                                                 // set right motor speed to 0
    }
    
    for(byte j = 0; j < 6; j++)                                                        // read position information for 6 servos
    {
        i = Wire.read() * 256 + Wire.read();                                             // read integer from I²C buffer
        if(abs(i) > 2400)
            errorflag = errorflag | 8;                                 // incorrect servo position given
        ControllerInterface::servopos[j] = i;                                                             // read new servo position -- 0 = no servo present
    }
    
    ControllerInterface::devibrate = Wire.read();                                                       // update devibrate setting - default=50 (100mS)
    i = Wire.read() * 256 + Wire.read();
    if(i > -1 && i < 1024)
    {
        ControllerInterface::sensitivity = i;                                                             // impact sensitivity from 0-1023 - default is 50
    }
    else
    {
        errorflag = errorflag | 16;                                                // incorrect sensitivity given
    }
    
    i = Wire.read() * 256 + Wire.read();                                               // read integer from I²C buffer
    if(i > 549 && i < 3001)
    {
        lowbat = i;                                                                  // set low battery value (values higher than battery voltage will force a shutdown)
    }
    else
    {
        errorflag = errorflag | 32;                                                // incorrect lowbat given
    }
    
    b = Wire.read();                                                               // read byte from buffer
    if(b < 128)
    {
        I2Caddress = b;                                                              // change I²C address
        EEPROM.write(1, b);                                                         // update EEPROM with new I²C address
    }
    else
    {
        errorflag = errorflag | 64;                                                // incorrect I²C address given
    }
    
    b = Wire.read();                                                               // read byte from buffer
    if(b < 2)
    {
        i2cfreq = b;                                                                 // 0=I²C clock 100kHz  -  >0=I²C clock 400kHz
        if(i2cfreq == 0)                                                             // thanks to Nick Gammon: http://gammon.com.au/i2c
        {
            TWBR = 72;                                                                 // default I²C clock is 100kHz
        }
        else
        {
            TWBR = 12;                                                                 // change the I²C clock to 400kHz
        }
    }
    else
    {
        errorflag = errorflag | 128;                                               // incorrect i2cfreq given
    }
    
    ControllerInterface::mode = 0;                                                                      // breaks out of Shutdown mode when I²C command is given
    ctrlIf.Motors();                                                                    // update brake, speed and direction of motors
    ctrlIf.Servos();                                                                    // update servo positions
}

//------------------------------------------------- Report control status to I²C Master --------------------------------------------------------
void I2Cstatus()
{
    byte datapack[24];                             // array to store data packet in prior to transmission
    datapack[0] = startbyte;                         // each packet starts with startbyte
    datapack[1] = errorflag;                         // nonzero if bad data received - Master must wait until buffer has been flushed and send again
    
    datapack[2] = highByte(volts);                   // battery voltage      high byte
    datapack[3] = lowByte(volts);                   // battery voltage      low  byte
    
    datapack[4] = highByte(ControllerInterface::lmcur);                   // left  motor current  high byte
    datapack[5] = lowByte(ControllerInterface::lmcur);                   // left  motor current  low  byte
    
    datapack[6] = highByte(ControllerInterface::lmenc);                   // left  motor encoder  high byte
    datapack[7] = lowByte(ControllerInterface::lmenc);                   // left  motor encoder  low  byte
    
    datapack[8] = highByte(ControllerInterface::rmcur);                   // right motor current  high byte
    datapack[9] = lowByte(ControllerInterface::rmcur);                   // right motor current  low  byte
    
    datapack[10] = highByte(ControllerInterface::rmenc);                  // right motor encoder  high byte
    datapack[11] = lowByte(ControllerInterface::rmenc);                  // right motor encoder  low  byte
    
    datapack[12] = highByte(ControllerInterface::xaxis);                  // accelerometer X-axis high byte
    datapack[13] = lowByte(ControllerInterface::xaxis);                  // accelerometer X-axis low  byte
    
    datapack[14] = highByte(ControllerInterface::yaxis);                  // accelerometer Y-axis high byte
    datapack[15] = lowByte(ControllerInterface::yaxis);                  // accelerometer Y-axis low  byte
    
    datapack[16] = highByte(ControllerInterface::zaxis);                  // accelerometer Z-axis high byte
    datapack[17] = lowByte(ControllerInterface::zaxis);                  // accelerometer Z-axis low  byte
    
    datapack[18] = highByte(ControllerInterface::deltx);                  // X-axis impact data   high byte
    datapack[19] = lowByte(ControllerInterface::deltx);                  // X-axis impact data   low  byte
    
    datapack[20] = highByte(ControllerInterface::delty);                  // Y-axis impact data   high byte
    datapack[21] = lowByte(ControllerInterface::delty);                  // Y-axis impact data   low  byte
    
    datapack[22] = highByte(ControllerInterface::deltz);                  // Z-axis impact data   high byte
    datapack[23] = lowByte(ControllerInterface::deltz);                  // Z-axis impact data   low  byte
    
    Wire.write(datapack, 24);                       // transmit data packet of 24 bytes
    errorflag = 0;                                   // reset erroflag once error has been reported to I²C Master
    /*
     Serial.println("Status data packet sent to Master:");
     for(byte i = 0; i < 24; i++)
     {
        Serial.print(i, DEC);
        Serial.print("\t");
        Serial.println(datapack[i], DEC);
     }
     */
}


//
// Brief	Setup
// Details	Define the pin the LED is connected to
//
// Add setup code 
void setup() {
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


    //all IO pins are input by default on powerup --------- configure motor control pins for output -------- pwm autoconfigures -----------

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
        ControllerInterface::mode = 1;                                         // set mode to RC
        ctrlIf.MotorBeep( 3 );                            // generate 3 beeps from the motors to indicate RC mode enabled
    }
  
    //----------------------------------------------------- Test for Bluetooth module ------------------------------------------------------
    if(ControllerInterface::mode == 0)                                         // no RC signals detected
    {
        ctrlIf.BluetoothConfig();                         // attempts to configure bluetooth module - changes to mode 2 if successful
        if(ControllerInterface::mode == 2)
            ctrlIf.MotorBeep(2);                          // generate 2 beeps from the motors to indicate bluetooth mode enabled
    }
  
    //----------------------------------------------------- Configure for I²C control ------------------------------------------------------
    if(ControllerInterface::mode == 0)                                         // no RC signal or bluetooth module detected
    {
        ctrlIf.MotorBeep( 1 );                            // generate 1 beep from the motors to indicate I²C mode enabled
        byte i = EEPROM.read(0);                          // check EEPROM to see if I²C address has been previously stored
        if(i == 0x55)                                     // B01010101 is written to the first byte of EEPROM memory to indicate that an I2C address has been previously stored
        {
            I2Caddress = EEPROM.read( 1 ); // read I²C address from EEPROM
        }
        else                                              // EEPROM has not previously been used by this program
        {
            EEPROM.write(0, 0x55);                        // set first byte to 0x55 to indicate EEPROM is now being used by this program
            EEPROM.write(1, 0x07);                        // store default I²C address
            I2Caddress = 0x07;                            // set I²C address to default
        }
    
        Wire.begin( I2Caddress );                           // join I²C bus as a slave at I2Caddress
        Wire.onReceive( I2Ccommand );              // specify ISR for data received
        Wire.onRequest( I2Cstatus );               // specify ISR for data to be sent
    }
}

//
// Brief	Loop
// Details	Execute the motor controller
//
void loop() {
    //----------------------------------------------------- Diagnostic mode --------------------------------------------------------------
    /*
    ctrlIf.DiagnosticMode();
    return;
    */
    //----------------------------------------------------- Shutdown mode ----------------------------------------------------------------
    if(ControllerInterface::mode == 3)                                         // if battery voltage too low
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
  
    //----------------------------------------------------- Bluetooth mode ----------------------------------------------------------------
    if(ControllerInterface::mode == 2)
    {
        ctrlIf.Bluetooth();                                       // control using Android phone and sample app
        return;
    }
  
    //----------------------------------------------------- I²C mode ----------------------------------------------------------------------
  

    //===================================================== Programmer's Notes ============================================================
    //                                    Detecting impacts requires reasonably accurate timing.                                         //
    //                  As all timers are in use this code uses the micros() function to simulate a timer interrupt.                     //
    //                                                                                                                                   //
    //                      Reading an analog input takes 260uS so reading 3 analog inputs can take about 800uS.                         //
    //                 This code alternates between reading accelerometer data and voltage / current readings every 1mS.                 //
    //                  If you edit this code then be aware that impact detection may be affected if care is not taken.                  //
    //=====================================================================================================================================


    static byte alternate;                               // variable used to alternate between reading accelerometer and power analog inputs
  
  
  
    //----------------------------------------------------- Perform these functions every 1mS ----------------------------------------------
    if(micros()-time > 999)
    {
        time = micros();                                     // reset timer
        alternate = alternate^1;                             // toggle alternate between 0 and 1
        ctrlIf.Encoders();                                        // check encoder status every 1mS

        //--------------------------------------------------- These functions must alternate as they both take in excess of 780uS ------------
        if(alternate)
        {
            ctrlIf.Accelerometer();                                 // monitor accelerometer every second millisecond
        }
        else
        {
            ControllerInterface::lmcur = (analogRead(lmcurpin) - 511) * 48.83;          // read left motor current sensor and convert reading to mA
            ControllerInterface::rmcur = (analogRead(rmcurpin) - 511) * 48.83;          // read right motor current sensor and convert reading to mA
            volts = analogRead(voltspin) * 10 / 3.357;             // read battery level and convert to volts with 2 decimal places (eg. 1007 = 10.07 V)
            if(volts < lowbat)
                ControllerInterface::mode = 3;                         // change to shutdown mode if battery voltage too low
        }
    }
}
