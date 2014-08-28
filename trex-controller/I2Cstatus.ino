//------------------------------------------------- Report control status to I²C Master -------------------------------------------------------- 
void I2Cstatus()
{
  byte datapack[24];                             // array to store data packet in prior to transmission
  datapack[0]=startbyte;                         // each packet starts with startbyte
  datapack[1]=errorflag;                         // nonzero if bad data received - Master must wait until buffer has been flushed and send again
  
  datapack[2]=highByte(volts);                   // battery voltage      high byte
  datapack[3]= lowByte(volts);                   // battery voltage      low  byte
  
  datapack[4]=highByte(lmcur);                   // left  motor current  high byte
  datapack[5]= lowByte(lmcur);                   // left  motor current  low  byte
  
  datapack[6]=highByte(lmenc);                   // left  motor encoder  high byte 
  datapack[7]= lowByte(lmenc);                   // left  motor encoder  low  byte 
  
  datapack[8]=highByte(rmcur);                   // right motor current  high byte
  datapack[9]= lowByte(rmcur);                   // right motor current  low  byte
  
  datapack[10]=highByte(rmenc);                  // right motor encoder  high byte 
  datapack[11]= lowByte(rmenc);                  // right motor encoder  low  byte 
  
  datapack[12]=highByte(xaxis);                  // accelerometer X-axis high byte
  datapack[13]= lowByte(xaxis);                  // accelerometer X-axis low  byte
  
  datapack[14]=highByte(yaxis);                  // accelerometer Y-axis high byte
  datapack[15]= lowByte(yaxis);                  // accelerometer Y-axis low  byte
  
  datapack[16]=highByte(zaxis);                  // accelerometer Z-axis high byte
  datapack[17]= lowByte(zaxis);                  // accelerometer Z-axis low  byte
  
  datapack[18]=highByte(deltx);                  // X-axis impact data   high byte
  datapack[19]= lowByte(deltx);                  // X-axis impact data   low  byte
  
  datapack[20]=highByte(delty);                  // Y-axis impact data   high byte
  datapack[21]= lowByte(delty);                  // Y-axis impact data   low  byte
  
  datapack[22]=highByte(deltz);                  // Z-axis impact data   high byte
  datapack[23]= lowByte(deltz);                  // Z-axis impact data   low  byte
  
  Wire.write(datapack,24);                       // transmit data packet of 24 bytes
  errorflag=0;                                   // reset erroflag once error has been reported to I²C Master
  /*
  Serial.println("Status data packet sent to Master:");
  for(byte i=0;i<24;i++)
  {
    Serial.print(i,DEC);Serial.print("\t");Serial.println(datapack[i],DEC);
  }
  */
}


