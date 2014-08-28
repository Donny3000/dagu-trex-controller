/*

The provided sample code requires external controllers to use the I²C data packet protocol listed below.
Each data packet starts with a start byte (0x0F, decimal 15) as a simple means of error checking.

The T'REX controller joins the I²C bus as a slave with the I²C address 0x07 (decimal address 7).
This address can be re-defined at the start of the sample code if required.
All integers are sent as high byte and then low byte.


===== Command data packet - 27 bytes =====
byte  Start 0x0F
byte  PWMfreq 1=31.25kHz    7=30.5Hz
int   lmspeed
byte  lmbrake
int   rmspeed
byte  rmbrake
int   servo 0
int   servo 1
int   servo 2
int   servo 3
int   servo 4
int   servo 5
byte  devibrate                       default=50 (100mS)   0-255
int   impact sensitivity  0 to 1023   default=50  
int   low battery  550 to 30000       5.5V to 30V
byte  I²C address
byte  I²C clock frequency: 0=100kHz   1=400kHz


===== Status data packet - 24 bytes =====
byte  Start 0x0F
byte  errorflag
int   battery voltage
int   left  motor current
int   left  motor encoder
int   right motor current
int   right motor encoder
int   X-axis
int   Y-axis
int   Z-axis
int   deltaX
int   deltaY
int   deltaZ
*/
