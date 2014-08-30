void Motors()
{
  digitalWrite(lmbrkpin,lmbrake>0);                     // if left brake>0 then engage electronic braking for left motor
  digitalWrite(lmdirpin,lmspeed>0);                     // if left speed>0 then left motor direction is forward else reverse
  analogWrite (lmpwmpin,abs(lmspeed));                  // set left PWM to absolute value of left speed - if brake is engaged then PWM controls braking
  if(lmbrake>0 && lmspeed==0) lmenc=0;                  // if left brake is enabled and left speed=0 then reset left encoder counter
  
  digitalWrite(rmbrkpin,rmbrake>0);                     // if right brake>0 then engage electronic braking for right motor
  digitalWrite(rmdirpin,rmspeed>0);                     // if right speed>0 then right motor direction is forward else reverse
  analogWrite (rmpwmpin,abs(rmspeed));                  // set right PWM to absolute value of right speed - if brake is engaged then PWM controls braking
  if(rmbrake>0 && rmspeed==0) rmenc=0;                  // if right brake is enabled and right speed=0 then reset right encoder counter
}

void MotorBeep(byte beeps)                              
{
  digitalWrite(lmbrkpin,0);                             // ensure breaks are off
  digitalWrite(rmbrkpin,0);
  
  for(int b=0;b<beeps;b++)                              // loop to generate multiple beeps
  {
    for(int duration=0;duration<400;duration++)         // generate 2kHz tone for 200mS
    {
      digitalWrite(lmdirpin,1);                         // drive left  motor forward
      digitalWrite(rmdirpin,1);                         // drive right motor forward
      digitalWrite(lmpwmpin,1);                         // left  motor at 100%
      digitalWrite(rmpwmpin,1);                         // right motor at 100%
      delayMicroseconds(50);                            // limit full power to 50uS
      digitalWrite(lmpwmpin,0);                         // shutdown left  motor
      digitalWrite(rmpwmpin,0);                         // shutdown right motor
      delayMicroseconds(200);                           // wait aditional 200uS to generate 2kHz tone
      
      digitalWrite(lmdirpin,0);                         // drive left  motor backward
      digitalWrite(rmdirpin,0);                         // drive right motor backward
      digitalWrite(lmpwmpin,1);                         // left  motor at 100%
      digitalWrite(rmpwmpin,1);                         // right motor at 100%
      delayMicroseconds(50);                            // limit full power to 50uS
      digitalWrite(lmpwmpin,0);                         // shutdown left  motor
      digitalWrite(rmpwmpin,0);                         // shutdown right motor
      delayMicroseconds(200);                           // wait aditional 200uS to generate 2kHz tone
    }
    delay(200);                                         // pause for 200mS (1/5th of a second) between beeps
  }
}
  


