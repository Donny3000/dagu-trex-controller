void Shutdown()
{
  //================ Shutdown motors and servos when battery is flat ==============
  
  lmspeed=0;      // set left  motor speed to 0 (off)
  rmspeed=0;      // set right motor speed to 0 (off)
  Motors();       // update H bridges
  
  servopos[0]=0;  // ensure servo 0 is detached
  servopos[1]=0;  // ensure servo 1 is detached
  servopos[2]=0;  // ensure servo 2 is detached
  servopos[3]=0;  // ensure servo 3 is detached
  servopos[4]=0;  // ensure servo 4 is detached
  servopos[5]=0;  // ensure servo 5 is detached
  Servos();       // update servo pins
}
