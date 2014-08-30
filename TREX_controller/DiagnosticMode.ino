void DiagnosticMode()
{
  //---------------------------------------- Diagnostic Mode ------------------------------------//
  //  This simple routine runs the motors forward / backward and brakes to test the "H" bridges  //
  //  Battery voltage, accelerometer data and motor current draw is displayed on serial monitor  //
  //             When LEDs are connected to servo outputs they will chase in sequence            //
  //---------------------------------------------------------------------------------------------//
  
  static int mdir,mpwm,brk,LED,div;
  if(mdir==0)                             // first time through the loop mdir=0 
  {                                       // initialize diagnostic routine
    mdir=5;                               // motor direction cannot start at 0 or motors will not move   
    for(byte i=0;i<6;i++)                 // scan through servo pins
    {
      pinMode(servopin[i],OUTPUT);        // set servo pin to OUTPUT
    }
  }
  
  mpwm+=mdir;                             // motor speed/direction is incremented/decremented by motor direction                   
  if(mpwm<-250 || mpwm>250)               // PWM value must be between -255 and +255
  {
    mdir=-mdir;                           // when limit is reached, reverse direction
    brk=1;                                // engage brake for quick slow down
  }
  if(mpwm==0) brk=0;                      // if motor PWM is 0 then disable the brake - motor can start again
  
  lmspeed=mpwm;                           // set left  motor speed
  rmspeed=mpwm;                           // set right motor speed
  lmbrake=brk;                            // enable / disable left  brake
  rmbrake=brk;                            // enable / disable right brake
  Motors();                               // update speed, direction and brake of left and right motors
  
  div++;                                  // divider used to slow down LED chasing
  if(div>20)
  {
    div=0;                                // reset divider
    LED++;                                // increment LED chase sequence
  }
  
  if(LED>5) LED=0;                        // cause chase sequence to repeat
  for(byte i=0;i<6;i++)                   // scan servo control pins
  {
    digitalWrite(servopin[i],LED==i);     // drive LEDs in chase sequence
  }
  
  Serial.print("Battery voltage: "); Serial.print(int(analogRead(voltspin)*10/3.357));Serial.print("\t");
  
  Serial.print("X: "); Serial.print(analogRead(axiszpin));Serial.print("\t");
  Serial.print("Y: "); Serial.print(analogRead(axisypin));Serial.print("\t");
  Serial.print("Z: "); Serial.print(analogRead(axisxpin));Serial.print("\t");
  
  lmcur=(analogRead(lmcurpin)-511)*48.83;
  rmcur=(analogRead(rmcurpin)-511)*48.83;
  Serial.print("Lmotor current: ");Serial.print(lmcur);Serial.print("mA\t");
  Serial.print("Rmotor current: ");Serial.print(rmcur);Serial.print("mA\t");
  Serial.print("PWM:");Serial.println(mpwm);
  delay(10);
}


