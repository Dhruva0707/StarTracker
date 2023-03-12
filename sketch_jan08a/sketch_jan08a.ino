/*
 *  Author: Dhruva Bhatia
 *  
 *  Description
 *  Turns a stepper motor through 360 degrees and then back
 * 
 *  Equipment:
 *  Arduino Uno 2021
 *  E3D 1703 motor (Nema 16 current rating 1.6Amps)
 *  TB6600 board. Config: 0101100 (1/8th microstep, 1.5-1.7A)
 */

 #include <Keypad.h>

const byte numRows = 5;
const byte numCols = 4;

char keymap[numRows][numCols]= 
{
  {'A','B','#','*'},
  {'1','2','3','U'},
  {'4','5','6','D'},
  {'7','8','9','C'},
  {'L','0','R','E'}
};

byte colPins[numCols] = {2, 3, 4, 5};
byte rowPins[numRows]= {10, 9, 8, 7, 6};

Keypad myKeypad= Keypad(makeKeymap(keymap), rowPins, colPins, numRows, numCols);

 // Define the pins to control the motor
 #define DIRECTION_PIN A1
 #define STEP_PIN A2
 #define ENABLE_PIN A0
 #define TRIP_PIN A3

 #define RESET_STEPS 8500  // Number of steps to move up after hitting the trip switch. Equivalent to 8 mm i.e. pitch of the lead screw
 #define SMALL_DELAY 1000 // Small delay in microseconds. To be used while moving fast
 #define SMALLEST_DELAY 400 // Delay to be used while moving at fastest speed

 #define NUMMICROSTEPS 1608
 #define D 581.0
 #define X 555.0
 #define Y 230.0
 #define PULLEYROUNDSTOZEROANGLE 14.4
 #define INCREMENT 4    // Number of steps by which we will change numOfMicroSteps to increase or decrease the tracking speed

 // Tracking Range in degrees
 double startAngle = 0.0;
 double endAngle   = 0.0;
 double currentAngle = 0.0;

 // Define the delays for each cycle
 long stepsToZeroAngle = 0;
 int numOfMicroSteps = NUMMICROSTEPS;


 void setup()
 {
  // Setup the pins for everything else
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  //pinMode(ledPin, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(TRIP_PIN, INPUT);

  //enable to True or False
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIRECTION_PIN, HIGH);

  stepsToZeroAngle = (long) (PULLEYROUNDSTOZEROANGLE*NUMMICROSTEPS*85/16);

  //startSerial
  Serial.begin(9600);
  Serial.println("Armed");
 }
 

 void move_forward(long totalSteps, long delay)
 {
  /*
   * Description: 
   * Used to move the motor forward
   * 
   * Input Parameters:
   * None
   * 
   * Output Parameters:
   * None
   */

   if( totalSteps < 1) return;
   
   // set the direction
   digitalWrite(DIRECTION_PIN, HIGH);
   //Serial.print(numRevolutions); Serial.print("    "); Serial.println(totalSteps);
   long currRound = 0;
   //long startMiliSec = millis();
   long half_delay = delay / 2;

   // Go through each step
   for(long i=0; i<=totalSteps;i++){
     digitalWrite(STEP_PIN, HIGH);
     //delayMicroseconds(half_delay);
     delayMicroseconds(100);          // Need just a small high pulse. Usually just a few tens of microseconds will suffice
     digitalWrite(STEP_PIN, LOW);
     delayMicroseconds(half_delay-100);
     delayMicroseconds(half_delay);
   } 
 }

 

 void move_reverse(long totalSteps, long delay)
 {
  /*
   * Description: 
   * Used to move the motor in reverse (Counter-Clockwise)
   * If the deadend trip trips, stop and move forward a bit to release the switch
   * 
   * Input Parameters:
   * None
   * 
   * Output Parameters:
   * None
   */

   // set the direction

   if( totalSteps < 1) return;
   
   digitalWrite(DIRECTION_PIN, LOW);
   Serial.println("Moving Down");
   Serial.print("totalSteps"); Serial.print("    "); Serial.println(totalSteps);
   
   long currRound = 0;
   long startMiliSec = millis();
   long currMiliSec = 0;
   long half_delay = delay / 2;
   int trip_status = HIGH;

   // Go through each step
   for(long i=0; i<=totalSteps;i++){
     digitalWrite(STEP_PIN, HIGH);
     delayMicroseconds(half_delay);
     digitalWrite(STEP_PIN, LOW);
     delayMicroseconds(half_delay);
     //trip_status = digitalRead(TRIP_PIN);
     //if( trip_status == LOW) break;
   }
   
   //Serial.println("End of move reverse cycle\n");

   if( trip_status == LOW) // Move forward a bit to release the switch
   {
      move_forward(RESET_STEPS, delay);
   }
 }



 double computeL( double d, double x, double y, double angle )
 {
    double rad_ang = radians(angle);
    double lSquare = d*d + x*x + y*y - 2*d*(x*cos(rad_ang) + y*sin(rad_ang));
    return ( sqrt(lSquare));
 }



 void doOneRound(double stAngle, double endAngle, long microStepsPerRound, int fast)
 {
  double d = D;
  double x = X;
  double y = Y;

  double angle = stAngle;
  double oldL = computeL( d, x, y, angle);
  double l = 0.0;

  long totalSteps = 0;

  Serial.print(stAngle); Serial.print(", "); Serial.println(endAngle);
  long startMiliSec = millis();
  angle = stAngle - 0.25;    // Breaking the movement into 0.25 degrees segments. This also means 1 minute intervals
  while( angle > endAngle - 0.25)
  {
    Serial.println(angle);
    l = computeL( d, x, y, angle);
    double deltaL = l - oldL;
    oldL = l;
    double nPulleyTurns = deltaL / 8;   // 8 is the pitch of the lead screw
    double nMotorTurns = nPulleyTurns * 85 / 16;  // Reduction ratio is 85:16
    long nSteps = (long) (nMotorTurns * microStepsPerRound);  // per minute. i.e. 60 Million microseconds
    long interval = (long) (60000000.0 / nSteps);     // interval between two steps in microseconds
    Serial.print(nSteps); Serial.print(", "); Serial.println(interval);

    if( fast == 0)
    {
      move_forward(nSteps, interval);
    }
    else
    {
      move_forward(nSteps, SMALL_DELAY);
    }

    totalSteps += nSteps;
    angle -= 0.25;
  }
  long endMiliSec = millis();
  Serial.print("Milli-seconds taken to complete one round was "); Serial.println(endMiliSec - startMiliSec);

  // Now move backwards to the original position. Wait a bit before and after this move to indicate this situation
  delay(4000);
  Serial.println("Moving down");
  move_reverse(totalSteps, SMALL_DELAY);
  delay(24000);
 }



 void initializePosition()
 {

    // Check if the TRIP_PINg circuit is working
    int initialized = 0;
    while(!!initialized)
    {
      int pin_status = digitalRead(TRIP_PIN);
      if (pin_status == LOW)
      {
        Serial.println("Tripped");
        while (pin_status == LOW)     // Wait till the button is released
        {
          pin_status = digitalRead(TRIP_PIN);
        }

        Serial.println("Finished consuming");
        delay(1000);  // Wait for a second. Just to be sure

        initialized = 1;
      }
    }

    // Reverse till the button trips on its own
    move_reverse((long)(2000000), SMALLEST_DELAY);  // Trying to move back at double the normal fast speed

    delay(1000);
    // Now go to Zero degree position
    long steps_to_move = stepsToZeroAngle;
    Serial.println(steps_to_move);
    move_forward(steps_to_move, SMALLEST_DELAY); // Move to initial position at double the normal fast speed
 }



void setAngleRange(int angle)
{
  startAngle = angle/2.0;
  endAngle   = -startAngle;
  Serial.print("Range set from "); Serial.print(startAngle); Serial.print(" to "); Serial.println(endAngle);
  Serial.println(currentAngle);

  double l1 = computeL( D, X, Y, currentAngle );
  double l2 = computeL( D, X, Y, startAngle );
  double nPulleyTurns;
  double nMotorTurns;
  long nSteps;

  Serial.print("l1 "); Serial.println(l1);
  Serial.print("l2 "); Serial.println(l2);
  if( l2 > l1 )
  {
    nPulleyTurns = (l2 - l1) / 8;   // 8 is the pitch of the lead screw
    nMotorTurns = nPulleyTurns * 85 / 16;  // Reduction ratio is 85:16
    nSteps = (long) (nMotorTurns * NUMMICROSTEPS);
    Serial.println(nPulleyTurns);
    Serial.println(nMotorTurns);
    Serial.println(nSteps);
    move_forward( nSteps, SMALLEST_DELAY);
  }
  else
  {
    nPulleyTurns = (l1 - l2) / 8;   // 8 is the pitch of the lead screw
    nMotorTurns = nPulleyTurns * 85 / 16;  // Reduction ratio is 85:16
    nSteps = (long) (nMotorTurns * NUMMICROSTEPS);
    
    Serial.println(nPulleyTurns);
    Serial.println(nMotorTurns);
    Serial.println(nSteps);
    move_reverse( nSteps, SMALLEST_DELAY);
  }

  currentAngle = startAngle;
}


 void loop()
 {
  
  char keypressed = myKeypad.getKey();
  if( keypressed == NO_KEY) return;
  Serial.println(keypressed);

  if( keypressed == '#') // go for verifiying the tripping button and position at 0 degree position
  {
    Serial.println("Press the Trip switch to initialize the tracker to zero degree position");
    initializePosition();
  }

  if( keypressed >= '0' && keypressed <= '9')
  {
    setAngleRange((int)(keypressed - '0'));
    return;
  }

  if( keypressed == 'A')  // Need to tracking round
  {
    Serial.println("Do normal tracking");
    doOneRound( startAngle, endAngle, numOfMicroSteps, 0);
    return;
  }

  if( keypressed == 'B') // Do a round at fast speed
  {
    Serial.println("Do a fast round");
    doOneRound( startAngle, endAngle, numOfMicroSteps, 1);
    return;
  }

  if( keypressed == 'U') // Increase the tracking speed a bit
  {
    numOfMicroSteps += INCREMENT;
    Serial.print("Number of microsteps "); Serial.println(numOfMicroSteps);
    return;
  }

  if( keypressed == 'D') // Decrease the tracking speed a bit
  {
    numOfMicroSteps -= INCREMENT;
    Serial.print("Number of microsteps "); Serial.println(numOfMicroSteps);
    return;
  }

  if( keypressed == '*') // Reset the tracking speed to default
  {
    numOfMicroSteps = NUMMICROSTEPS;
    Serial.print("Number of microsteps "); Serial.println(numOfMicroSteps);
    return;
  }

  //Serial.print("Incorrect input "); Serial.println(keypressed);
  delay(500);
 }
