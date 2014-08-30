void Encoders()
{
  static byte lencold,lencnew,rencold,rencnew;                      // old encoder values
  lencold=lencnew;                                                  // store previous value for left  encoder
  rencold=rencnew;                                                  // store previous value for right encoder
  lencnew=digitalRead(lmencpin);                                    // read new left  encoder value
  rencnew=digitalRead(rmencpin);                                    // read new right encoder value
  if (lencold!=lencnew) lmenc+=lmspeed/abs(lmspeed);                // if old does not equal new then add or subtract 1 depending on motor direction
  if (rencold!=rencnew) rmenc+=lmspeed/abs(lmspeed);                // if old does not equal new then add or subtract 1 depending on motor direction
}


