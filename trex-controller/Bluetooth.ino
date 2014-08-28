void Bluetooth()
{
  //============================================================= Bluetooth Control ===================================================

  static byte d,e;                                             // data and error bytes
  if(Serial.available()>2)                                     // Command is 3 bytes in length
  {
    d=Serial.read();                                           // read byte from buffer
    if(d!=startbyte)                                           // if byte is not a start byte (0x0F)
    {
      lmspeed=0;                                               // bad data received
      rmspeed=0;                                               // set motor speeds to 0
      e=0;                                                     // error flag reset
    }
    else
    {
      lmspeed=(int(Serial.read())-127)*2-1;                    // good data received
      rmspeed=(int(Serial.read())-127)*2-1;                    // read values for left and right motors
    }
  }
  else                                                         // less than 3 bytes in buffer
  {
    e++;                                                       // count program loops with less than 3 bytes in buffer
    if(e>100)                                                  // assume lost signal if buffer less than 3 bytes for too long
    {
      lmspeed=0;                                               // stop left motor
      rmspeed=0;                                               // stop right motor
      e=0;                                                     // reset error counter
    }
  }
  Motors();                                                    // update motors 
}



void BluetoothConfig()                                         // This code intended for a DAGU bluetooth module - may not work with other brands
{
  long baud[]={9600,115200,57600,38400,19200,4800,2400,1200};  // try 9600 first as this is default setting then try other baud rates
  byte br=0,d;
  while(mode==0 && br<8)                                       // scan through different baud rates and attempt to configure bluetooth module
  {
    Serial.begin(baud[br]);                                    // enable T'REX serial at baud rate baud[br]
    Serial.print("AT");                                        // send "AT" to see if bluetooth module is connected
    delay(1500);                                               // wait for bluetooth module to respond
    
    if(Serial.available()>1)                                   // after 1 second the bluetooth module should respond
    {
      byte i=Serial.read();                                    // should be 79 "O"
      byte j=Serial.read();                                    // should be 75 "K"
      if(i==79 && j==75)                                       // if response is "OK" then cofigure bluetooth module
      {
        EmptyBuffer();                                         // clear buffer
        Serial.print("AT+NAMET'REX");                          // ensure name is set to "T'REX"
        delay(1500);                                           // wait for bluetooth module to respond
        EmptyBuffer();                                         // clear buffer
        Serial.print("AT+PIN1234");                            // ensure PIN is set to "1234"
        delay(1500);                                           // wait for bluetooth module to respond
        EmptyBuffer();                                         // clear buffer
        if(br!=0)                                              // if bluetooth baud rate was not 9600
        { 
          Serial.print("AT+BAUD4");                            // set bluetooth baud rate to 9600
          delay(1500);                                         // wait for bluetooth module to respond
          EmptyBuffer();                                       // clear buffer
          Serial.end();                                        // close serial communications at current baud rate
          Serial.begin(9600);                                  // set T'REX controller serial communications to 9600
        }
        mode=2;                                                // bluetooth module successfully detected and configured - change to bluetooth mode
      }        
    }
    if(mode==0)                                                // bad response - bluetooth module not communicating at current baud rate
    {
      EmptyBuffer();
      Serial.end();                                            // close serial communications at this baud rate
      br++;                                                    // prepare to try next baud rate
    }
  }
}

void EmptyBuffer()
{
  byte b;
  while(Serial.available())                                    // empty buffer once response is received
  {
    b=Serial.read();
  }
}



//=============================================================== Bluetooth AT+ commands ===========================================
/*
--------- Test Communications --------
 Send: AT            Receive: OK
 
 
 ---------- Change Baud Rate ----------
 Send: AT+BAUD1      Receive: OK1200
 Send: AT+BAUD2      Receive: OK2400
 Send: AT+BAUD3      Receive: OK4800
 Send: AT+BAUD4      Receive: OK9600
 Send: AT+BAUD5      Receive: OK19200
 Send: AT+BAUD6      Receive: OK38400
 Send: AT+BAUD7      Receive: OK57600
 Send: AT+BAUD8      Receive: OK115200
 
 
 ------------ Change Name -------------
 Send: AT+NAMEname   Receive: OKsetname
 
 
 --------- Change Pairing Code --------
 Send: AT+PIN1234    Receive: OKsetpin
 
 
 
 
 
 
 
 
 
 
 
 */

