/*
The T'REX sample code expects 27 bytes of data to be sent to it in a specific order, this is the "command data packet"
If you do not send the correct number of bytes in the correct sequence then the T'REX will ignore the data and set the error flag
Your software can use this error flag to re-send datapackets that may have been corrupted due to electrical interferance

Master to Slave data packet - 27 bytes

byte  Start 0xF0
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
byte  devibrate 0-255   default=50 (100mS)
int   impact sensitivity  
int   low battery  550 to 30000       5.5V to 30V
byte  I²C address
byte  I²C clock frequency: 0=100kHz  1=400kHz


When requested, the T'REX sample code will send a data packet reporting it's status

Slave to Master data packet - 24 bytes

byte  Start
byte  errorflag
int   battery voltage
int   left  motor current
int   left  motor encoder
int   right motor current
int   right motor encoder

int   X-axis   raw data from accelerometer X-axis          
int   Y-axis   raw data from accelerometer Y-axis
int   Z-axis   raw data from accelerometer Z-axis

int   delta X  change in X-axis over a period of 2mS
int   delta Y  change in Y-axis over a period of 2mS
int   delta Z  change in Z-axis over a period of 2mS


If the T'REX receives faulty data (possibly due to electrical interference) it will report the problem using the error flag
Error Flag Bits

BIT0: wrong start byte or number of bytes received
BIT1: incorrect PWM frequency specified - must be a byte with a value of 1 to 7
BIT2: incorrect motor speed             - left and right motor speeds must be an integer from -255 to +255
BIT3: incorrect servo position given    - all servo positions must be an integer between -2400 to +2400, negative values reverse the sense of the servo. 0 = no servo 
BIT4: incorrect impact sensitivity      - must be an integer from 0 to 1023
BIT5: incorrect lowbat value            - minimum battery voltage must be an integer from 550 to 3000 (5.50V to 30.00V)
BIT6: incorrect I²C address given       - I²C address must be a byte with a value from 0 to 127 (7 bit address)
BIT7: incorrect I²C clock frequency     - must be a byte with a value of 0 or 1

Note: All integers must be sent MSB first






*/
