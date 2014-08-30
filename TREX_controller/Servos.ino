void Servos()
{
  for(byte i=0;i<6;i++)                                                        // up to 6 servos are supported by T'REX controller
  {
    if(servopos[i]!=0 && servo[i].attached()==0) servo[i].attach(servopin[i]); // if servopos is non zero and servo is not attached then attach the servo
    if(servopos[i]==0 && servo[i].attached()!=0)                               // if servo pos is 0 but servo is attached
    {
      servo[i].detach();                                                       // if servopos=0 and the servo is attached then detach the servo 
      pinMode(servopin[i],INPUT);                                              // set unused servo pin as input
    }
    
    if(servopos[i]>0) servo[i].writeMicroseconds(servopos[i]);                 // if servopos is >0 then move servo to that position
    if(servopos[i]<0) servo[i].writeMicroseconds(3000+servopos[i]);            // if servopos is <0 then move servo to that position but reverse sense of direction
  }
}

    
