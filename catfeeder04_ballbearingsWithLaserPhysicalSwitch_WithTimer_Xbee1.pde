/*
Sketch to control automatic cat feeding machine
Andres Leon
power must be set at 9 volts for vex motor to work properly.
laser and LDR schematic inspired from http://www.glacialwanderer.com/hobbyrobotics/?p=11
it is now able to respond to some basic commands from the serial interface.
*/
#include <Servo.h>
/* ************************************************************************************** */
const int versionNumber = 15;    //update this number with the file name so i know how to tell which
                                //version of the software this board is currently running.
/* ************************************************************************************** */
/* ************************************************************************************** */
//const float hoursBetweenFeedings = 0.015; //0.015 is 54 seconds
const float hoursBetweenFeedings = 4;  //number of hours for timer to run between 
                                        //feeds (1=1 hour, 0.25 = 15 mins)
/* ************************************************************************************** */
#define LDRPin 1      //pin connected to the Photoresistor
#define LASERPin 5    //pin connected to the LASER
#define SERVOPin 9    //pin connected to the servor motor
#define WAITAFTERCMD 5  //amount of millisecs to pause program after a command is received
                        //and response has been sent back to PC.
#define PUSHPin 7    //pin reading the input from pushpin that sets the feeding status to
                      //enable or disable.
#define LEDFeedStatusPin 13  //pin for LED that shows if feeder is enabled or not.

Servo myservo;  // create servo object to control a servo
const int servoFullStop = 94;          //signal required for the motor to STOP spinning (not really used)
const int servoFullStopForVexMotor = 94;  //signal required for the motor to STOP spinning
const int servoSpeedToOpen = 175;      //speed the servo runs at when feeding (cck direction)
const int servoSpeedToReposition = 145;  //speed the servo runs at when repositioning container
const int servoSpeedToClose = 1;   //speed the servo runs at when feeding (cw direction)
const int LDRThreshold = 950;      //the unit of resistance the photoresistor reports when
                                   //the laser beam is hitting it directly.
boolean allowFeedAndPos = true;  //indicates if controller should rotate container to feed

int mainLoopCounter = 0;      //counts how many times the main loop has run to determine if the
                              //command to feed needs to be sent. if more than 0, then feeding
                              //has been done. do not fed again.
int moveDoorLoopLen = 550;    //units of time that the container spins when feeding

char currentStatus = ' ';        //stores the status of what the feeder is doing at this moment.
                                //f=feeding,r=repositioning,s=straight
int pushButtonCurrVal = 0;    //current value that will store state of the button
int pushButtonPrevVal = 0;    //previous value of the button

float threshholdForTimer = 0.001;  //the gap that needs to be given for the program to readh this point
                                   //and still run the timer correctly.
unsigned long timerIncrement = hoursBetweenFeedings * 3600000; //(3600000 = 60min * 60sec * 1000milli)
unsigned long maxTimerLen = 4294967294; // this is about 50 days. when the millis is reset back to 0.
unsigned long baseTimeToFeed;

void setup()   //runs one time only
{ 
  //myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  //Serial.begin(9600);
  Serial.begin(19200);
  Serial.print("Cate feeder restated :: Software version ");
  Serial.println(versionNumber);
  pinMode(LASERPin, OUTPUT);
  pinMode(LEDFeedStatusPin, OUTPUT);
  pinMode(PUSHPin, INPUT);  //receive signal from button to enable or disable feeder
  baseTimeToFeed = timerIncrement;  //initilaize timer with minimum time to start feeding
} 

void loop() //loops forever
{
  int incomingSerialCmd = -1;  //when no data is read from serial default is -1  
  if (Serial.available() > 0) 
  {//command from computer is waiting to be processed
    incomingSerialCmd = Serial.read();
    switch(incomingSerialCmd)
    {
      case 48: //(48==0)
      {  //return the current status of the device
        //Serial.println(1, DEC);  //feeding now
        Serial.println(currentStatus);  //send current status
        break;
      }
      case 49: //(49==1)
      {  //reset the counter to initiate the container spin (49 is ascii for 1)        
        //Serial.println(1, DEC);  //feeding now
        Serial.println("<<FEEDCOMMAND>>");  //feeding now
        mainLoopCounter = 0;
        break;
      }
      case 50:  //(50==2)
      {  //prevent future feeding. Also prevent from checking container pos
        //Serial.println(2, DEC);  //not feeding
        Serial.println("<<STOPFEEDCOMMAND>>");  
        digitalWrite(LASERPin, LOW);    //turn laser light off
        allowFeedAndPos = false;
        break;
      }
      case 51:  //(51==3)
      {  //Resume feeding.
        //Serial.println(3, DEC);  //resume feeding
        Serial.println("<<RESUMEFEEDCOMMAND>>"); 
        //digitalWrite(LASERPin, LOW);    //turn laser light off
        allowFeedAndPos = true;
        break;
      }
      case 52:  //(52==4)
      {  //reposition container to correct position
        //Serial.println(4, DEC); //respoitioning container
        Serial.println("<<REPOSITIONCOMMAND>>");
        CheckContainerPos();
        break;
      }
      case 53:  //(53==5)
      {  //Show current software version number
        Serial.print("<<SOFTAWREVERSIONCOMMAND>>");
        Serial.print(versionNumber, DEC); //the version number
        Serial.println("");
        delay(WAITAFTERCMD);
        break;
      }
      case 54:  //(53==6)
      {  //Show timer times in millisecs separted by commas
        Serial.print("<<");
        Serial.print(millis());  //current time in millis
        Serial.print(",");
        Serial.print(baseTimeToFeed); //next time to feed in millis
        Serial.println(">>");
        delay(WAITAFTERCMD);
        break;
      }
    }
    //Serial.println("");
  }
  if (allowFeedAndPos)
  {//feeder is enabled and will allow container to turn...
    digitalWrite(LEDFeedStatusPin, HIGH);  //turn LED on to show feeder is enabled.
    mainLoopCounter = TimerTrigger(mainLoopCounter);
    if (mainLoopCounter == 0)
    {//proceed to feed
      myservo.attach(SERVOPin);
      moveDoor('o');
      //delay(2000);
      myservo.detach();
      mainLoopCounter++;
      //delay(100);    //comment this out when set to production. this is to show gap between feeding and readjustment
      digitalWrite(SERVOPin, LOW);
    }
    else
    {  //feeding done. just reposition container
      CheckContainerPos();
      //myservo.write(servoFullStop);
      //resetContainer();    
    }
  }
  else
  {  //feeder is disabled... turn LED off
    digitalWrite(LEDFeedStatusPin, LOW);  //turn LED off to show feeder is not enabled.
  }
  ReadPushButtonForPhyisicalFeedStatus();
} 

void moveDoor(char openOrClose)
{
  int loopCounter = 0;
  //Serial.println(loopCounter);
  if (openOrClose == 'o')
  {
    for (loopCounter=0; loopCounter<moveDoorLoopLen; loopCounter++)
    {
      //Serial.println(loopCounter);
      currentStatus = 'f';
      Serial.println(currentStatus);
      delay(WAITAFTERCMD);
      myservo.write(servoSpeedToOpen);
    }
  }
  else
  {
    for (loopCounter=0; loopCounter<moveDoorLoopLen; loopCounter++)
    {
      currentStatus = 'f';
      Serial.println(currentStatus);
      delay(WAITAFTERCMD);
      myservo.write(servoSpeedToClose);
    }
  }
  delay(100);
  myservo.write(servoFullStopForVexMotor);
  //myservo.detach();
  return;  
}

void CheckContainerPos()
{
  int val = 0;
  digitalWrite(LASERPin, HIGH);    //turn laser light on
  val = analogRead(LDRPin);
  //Serial.print(val);
  while (val >= LDRThreshold)
  {
    //container is not in correct position... Move until it is
    RepositionContainer();
    val = analogRead(LDRPin);
    //Serial.print(val);
  }
  currentStatus = 's';    //container is now facing up...
  //Serial.println(currentStatus);
  delay(WAITAFTERCMD);
  //Serial.println(val);
}

void RepositionContainer()
{
  myservo.attach(SERVOPin);
  int x = 0;
  int moveDoorLoopToAdjustLen = 4;
  for (x=0; x<moveDoorLoopToAdjustLen; x++)
  {
    //Serial.println(x);
    currentStatus = 'r';
    Serial.println(currentStatus);
    delay(WAITAFTERCMD);
    myservo.write(servoSpeedToReposition);    //this speed is more efficient for accurate movement.
  }
  myservo.detach();
}

void ReadPushButtonForPhyisicalFeedStatus()
{
  int state = 0;  
  pushButtonCurrVal = digitalRead(PUSHPin);
  if (pushButtonCurrVal == HIGH)
  {
    Serial.println("pushbutton ON!!!");
  }  
  if ((pushButtonCurrVal == HIGH) && (pushButtonPrevVal == LOW))
  {
    delay(10);
    state = 1 - state;
    allowFeedAndPos = 1 - allowFeedAndPos;
  }
  
  pushButtonPrevVal = pushButtonCurrVal;
  if (!allowFeedAndPos)
  {
    digitalWrite(LASERPin, LOW);    //turn laser light off
  }
}

int TimerTrigger(int currentMainLoopCounter)
{  //this will return a 0 or 1 depending on whether the timer needs to be triggered
   //0 will enable the feeder to run. 1 will means that it has already ran.

  if (currentMainLoopCounter == 0)
    return 0;  //allow it to feed now...
  else    //is 1
  {//must determine if now is time to feed... 
     unsigned long baseTimeBottomLimit =  baseTimeToFeed - (baseTimeToFeed * threshholdForTimer);
     unsigned long baseTimeUpperLimit =  baseTimeToFeed + (baseTimeToFeed * threshholdForTimer);
     //use print lines below to debug the timer process..
     /*
     Serial.print(millis());
     Serial.print("  ---  ");
     Serial.print(baseTimeToFeed);
     Serial.print("  ---  ");
     Serial.print(timerIncrement);
     Serial.print("  ---  ");
     Serial.print(baseTimeBottomLimit);
     Serial.print("  ---  ");
     Serial.print(baseTimeUpperLimit);      
     Serial.println("");
     */
     if ((millis() >= baseTimeBottomLimit) && (millis() < baseTimeUpperLimit))
     {  //within window
       Serial.println("within period to feed");
       //now increase the vars to allow the next timer to occur...
       //timerIncrement += hoursBetweenFeedings * 3600000; //(3600000 = 60min * 60sec * 1000milli)
       baseTimeToFeed += hoursBetweenFeedings * 3600000; //(3600000 = 60min * 60sec * 1000milli)
       return 0;  //allow feeder to run
     }
     else
     {
       return 1;  //prevent feeder from running
     }
  }
}
