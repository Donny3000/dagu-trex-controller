void Accelerometer()
{
  static int oldx,oldy,oldz,vibration;                        // local variables used for impact detection
  
  oldx=xaxis;                                                 // store previous accelerometer readings for comparison
  oldy=yaxis;
  oldz=zaxis;
  
  vibration--;                                                // loop counter prevents false triggering cause by impact vibration
  if(vibration<0) vibration=0;                                // as robot vibrates due to impact
  
  xaxis=analogRead(axisxpin);                                 // read accelerometer - note analog read takes 260uS for each axis
  yaxis=analogRead(axisypin);
  zaxis=analogRead(axiszpin);
  if(vibration>0) return;                                     // until vibration has subsided no further calculations required
  
  deltx=xaxis-oldx;                                           // difference between old and new axis readings
  delty=yaxis-oldy;
  deltz=zaxis-oldz;
  magnitude=sqrt(sq(deltx)+sq(delty)+sq(deltz));              // magnitude of delta x,y,z using Pythagorus's Theorum
  
  if (magnitude>sensitivity)                                  // has a new impact occured
  {
    vibration=devibrate;                                      // reset anti-vibration counter
    return;
  }
  else
  {
    magnitude=0;                                              // no impact detected
    deltx=0;
    delty=0;
    deltz=0;
  }
}
  



