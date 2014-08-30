void RCmode()
{
  //------------------------------------------------------------ Code for RC inputs ---------------------------------------------------------

  int Speed=int(pulseIn(RCspeedpin,HIGH,25000));              // read throttle/left stick
  int Steer=int(pulseIn(RCsteerpin,HIGH,25000));              // read steering/right stick

  if (Speed==0) Speed=1500;                                   // if pulseIn times out (25mS) then set speed to stop
  if (Steer==0) Steer=1500;                                   // if pulseIn times out (25mS) then set steer to centre

  if (abs(Speed-1500)<RCdeadband) Speed=1500;                 // if Speed input is within deadband set to 1500 (1500uS=center position for most servos)
  if (abs(Steer-1500)<RCdeadband) Steer=1500;                 // if Steer input is within deadband set to 1500 (1500uS=center position for most servos)
  
  Steer=Steer-1500;
  lmspeed=(Speed-Steer-1500)*8/10;
  rmspeed=(Speed+Steer-1500)*8/10;
  
  if(lmspeed<-255) lmspeed=-255;
  if(lmspeed>255)  lmspeed=255;
  if(rmspeed<-255) rmspeed=-255;
  if(rmspeed>255)  rmspeed=255;
  
  Motors();
}


