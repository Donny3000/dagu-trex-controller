void MasterSend(byte sbyte, byte pfreq, int lspeed, byte lbrake, int rspeed, byte rbrake, int sv0, int sv1, int sv2, int sv3, int sv4, int sv5, byte dev,int sens,int lowbat, byte i2caddr,byte i2cfreq)
{
  Wire.beginTransmission(I2Caddress); // transmit data to 7
  Wire.write(startbyte);              // start byte
  Wire.write(pfreq);                  // pwm frequency
  
  Wire.write(highByte(lspeed));       // MSB left  motor speed
  Wire.write( lowByte(lspeed));       // LSB left  motor speed
  Wire.write(lbrake);                 // left  motor brake
  
  Wire.write(highByte(rspeed));       // MSB right motor speed
  Wire.write( lowByte(rspeed));       // LSB right motor speed
  Wire.write(rbrake);                 // right motor brake
  
  Wire.write(highByte(sv0));          // MSB servo 0
  Wire.write( lowByte(sv0));          // LSB servo 0
  
  Wire.write(highByte(sv1));          // MSB servo 1
  Wire.write( lowByte(sv1));          // LSB servo 1
  
  Wire.write(highByte(sv2));          // MSB servo 2
  Wire.write( lowByte(sv2));          // LSB servo 2
  
  Wire.write(highByte(sv3));          // MSB servo 3
  Wire.write( lowByte(sv3));          // LSB servo 3
  
  Wire.write(highByte(sv4));          // MSB servo 4
  Wire.write( lowByte(sv4));          // LSB servo 4
  
  Wire.write(highByte(sv5));          // MSB servo 5
  Wire.write( lowByte(sv5));          // LSB servo 5
  
  Wire.write(dev);                    // devibrate
  Wire.write(highByte(sens));         // MSB impact sensitivity
  Wire.write( lowByte(sens));         // LSB impact sensitivity
  
  Wire.write(highByte(lowbat));       // MSB low battery voltage  550 to 30000 = 5.5V to 30V
  Wire.write( lowByte(lowbat));       // LSB low battery voltage
  
  Wire.write(i2caddr);                // I2C slave address for T'REX controller
  Wire.write(i2cfreq);                // I2C clock frequency:   0=100kHz   1=400kHz
  Wire.endTransmission();             // stop transmitting
  
  Serial.println("Master Command Data Packet Sent");
  
  
  //-------------------------------- Make sure Master and Slave I2C clock the same ------------------------------------------------
  
  if(i2cfreq==0)                                                               // thanks to Nick Gammon: http://gammon.com.au/i2c
  {
    TWBR=72;                                                                   // default I²C clock is 100kHz
  }
  else
  {
    TWBR=12;                                                                   // changes the I²C clock to 400kHz
  }
}



