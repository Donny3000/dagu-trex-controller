
//------------------------------------------------------------------------------- Receive commands from I²C Master -----------------------------------------------
void I2Ccommand(int recvflag)     
{
  byte b;                                                                      // byte from buffer
  int i;                                                                       // integer from buffer
  
  do                                                                           // check for start byte
  {
    b=Wire.read();                                                             // read a byte from the buffer
    if(b!=startbyte || recvflag!=27)errorflag = errorflag | 1;                 // if byte does not equal startbyte or Master request incorrect number of bytes then generate error
  } while (errorflag>0 && Wire.available()>0);                                 // if errorflag>0 then empty buffer of corrupt data
  
  if(errorflag>0)                                                              // corrupt data received 
  {
    Shutdown();                                                                // shut down motors and servos
    return;                                                                    // wait for valid data packet
  }  
  //----------------------------------------------------------------------------- valid data packet received ------------------------------
  
  b=Wire.read();                                                               // read pwmfreq from the buffer
  if(b>0 && b<8)                                                               // if value is valid (1-7)  
  {
    pwmfreq=b;                                                                 // update pwmfreq
    TCCR2B = TCCR2B & B11111000 | pwmfreq;                                     // change timer 2 clock pre-scaler
  }
  else
  {
    errorflag = errorflag | 2;                                                 // incorrect pwmfreq given
  }
  
  i=Wire.read()*256+Wire.read();                                               // read integer from I²C buffer
  if(i>-256 && i<256)
  {
    lmspeed=i;                                                                 // read new speed for   left  motor
  }
  else
  {
    errorflag = errorflag | 4;                                                 // incorrect motor speed given
  }
  lmbrake=Wire.read();                                                         // read new left  motor brake status
  
  i=Wire.read()*256+Wire.read();                                               // read integer from I²C buffer
  if(i>-256 && i<256)
  {
    rmspeed=i;                                                                 // read new speed for   right motor
  }
  else
  {
    errorflag = errorflag | 4;                                                 // incorrect motor speed given
  }
  rmbrake=Wire.read();                                                         // read new right motor brake status
  
  if(errorflag & 4)                                                            // incorrect motor speed / shutdown motors 
  {
    lmspeed=0;                                                                 // set left  motor speed to 0
    rmspeed=0;                                                                 // set right motor speed to 0
  }
    
  for(byte j=0;j<6;j++)                                                        // read position information for 6 servos
  {
    i=Wire.read()*256+Wire.read();                                             // read integer from I²C buffer
    if(abs(i)>2400) errorflag = errorflag | 8;                                 // incorrect servo position given
    servopos[j]=i;                                                             // read new servo position -- 0 = no servo present
  }
  
  devibrate=Wire.read();                                                       // update devibrate setting - default=50 (100mS)
  i=Wire.read()*256+Wire.read();
  if(i>-1 && i<1024)
  {
    sensitivity=i;                                                             // impact sensitivity from 0-1023 - default is 50
  }
  else
  {
    errorflag = errorflag | 16;                                                // incorrect sensitivity given
  }
  
  i=Wire.read()*256+Wire.read();                                               // read integer from I²C buffer
  if(i>549 && i<3001)
  {
    lowbat=i;                                                                  // set low battery value (values higher than battery voltage will force a shutdown)
  }
  else
  {
    errorflag = errorflag | 32;                                                // incorrect lowbat given
  }
  
  b=Wire.read();                                                               // read byte from buffer
  if(b<128)
  {
    I2Caddress=b;                                                              // change I²C address
    EEPROM.write(1,b);                                                         // update EEPROM with new I²C address
  }
  else
  {
    errorflag = errorflag | 64;                                                // incorrect I²C address given
  }
  
  b=Wire.read();                                                               // read byte from buffer
  if(b<2)
  {
    i2cfreq=b;                                                                 // 0=I²C clock 100kHz  -  >0=I²C clock 400kHz
    if(i2cfreq==0)                                                             // thanks to Nick Gammon: http://gammon.com.au/i2c
    {
      TWBR=72;                                                                 // default I²C clock is 100kHz
    }
    else
    {
      TWBR=12;                                                                 // change the I²C clock to 400kHz
    } 
  }
  else
  {
    errorflag = errorflag | 128;                                               // incorrect i2cfreq given
  }
  
  mode=0;                                                                      // breaks out of Shutdown mode when I²C command is given
  Motors();                                                                    // update brake, speed and direction of motors
  Servos();                                                                    // update servo positions
}




