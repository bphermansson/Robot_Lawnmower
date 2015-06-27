
/* Arduino based robot lawnmower
©Patrik Hermansson 2015

Controlled by an RC-transmitter
Uses a L298-based card for controlling two motors
Cutting motor controlled by Arduino, a BC547-based Mosfet driver and a IRF540. 


Uses a HC-SR04 distance sensor to avoid objects in the way. 
Two electrical motors are used for moving and turning, controlled by a L298N-based driver
(code inspired by http://www.instructables.com/id/Arduino-Modules-L298N-Dual-H-Bridge-Motor-Controll/?ALLSTEPS)


Also inspired by TemperatureWebPanel (in Examples/Bridge)
and
http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
*/

/*
HC-SR04 Ping distance sensor]
VCC to arduino 5v GND to arduino GND
Echo to Arduino pin 13 Trig to Arduino pin 12
Red POS to Arduino pin 11
Green POS to Arduino pin 10
560 ohm resistor to both LED NEG and GRD power rail
More info at: http://goo.gl/kJ8Gl
Original code improvements to the Ping sketch sourced from Trollmaker.com
Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
*/

// RC Receiver
#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>
// Assign your channel in pins
#define THROTTLE_IN_PIN 9
#define STEERING_IN_PIN 8

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_FRAME_SPACE 2
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define AUX_FLAG 4
volatile uint8_t bUpdateFlagsShared;
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unAuxInShared;
uint16_t unThrottleInStart;
uint16_t unSteeringInStart;
uint16_t unAuxInStart;
uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;


// Connections for driver card
// Motor 1
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 4; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 12;
int dir2PinB = 10;
int speedPinB = 11; // Needs to be a PWM pin to be able to control motor speed

// Connections for HC-SR04
#define trigPin A2
#define echoPin A3
// Led for detect close distance
#define lednear 7
#define ledstuck A1

//Measuring Current Using ACS712
const int analogIn = A0;
int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
int RawValue= 0;
int ACSoffset = 2500; 
double Voltage = 0; // Input value from current sensor
double Amps = 0;

// Battery voltage meter
// A resistor divider connected to the battery and ground. 
#define lowbattled D13
#define voltsens A4   // Mid point of divider connected to input A4. 
float vPow = 4.7; // Voltage at the Arduinos Vcc and Vref. 
float r1 = 1000000;  // "Top" resistor, 1Mohm.
float r2 = 470000;   // "Bottom" resistor (to ground), 470 kohm. 

#define cutmotor 5
#define currentsens A0
int dir;

void setup() {
  Serial.begin (9600);
  Serial.println("Welcome to Lawnmower15!"); 

  // Define pins for HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(lednear, OUTPUT);
  // Test led near
  digitalWrite(lednear, HIGH);
  delay(500);
  digitalWrite(lednear, LOW);
  
  pinMode(ledstuck, OUTPUT);
  // Test ledstuck
  digitalWrite(ledstuck, HIGH);
  delay(500);
  digitalWrite(ledstuck, LOW);
     
  //Define L298N Dual H-Bridge Motor Controller Pins
  pinMode(dir1PinA,OUTPUT);
  pinMode(dir2PinA,OUTPUT);
  pinMode(speedPinA,OUTPUT);
  pinMode(dir1PinB,OUTPUT);
  pinMode(dir2PinB,OUTPUT);
  pinMode(speedPinB,OUTPUT);
  
  // RC Receiver
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,7*2000);
  CRCArduinoFastServos::begin();
  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_IN_PIN, calcSteering,CHANGE);
  /*
  Serial.println("Start your engines");
  analogWrite(speedPinA, 25);//Sets speed variable via PWM 
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  Serial.println("25");
  delay(2500);
  analogWrite(speedPinA, 100);
  Serial.println("100");
  delay(2500);
  analogWrite(speedPinA, 200);
  Serial.println("200");
  delay(2500);
  analogWrite(speedPinA, 255);  
  Serial.println("255");
  */  
  Serial.println("Spin up cut motor");
  pinMode(cutmotor, OUTPUT);
  digitalWrite(cutmotor, LOW);
  delay(500);
  digitalWrite(cutmotor, HIGH);

  pinMode(lowbattled, OUTPUT);
  pinMode(voltsens, INPUT);
  // Check battery monitor
  Serial.println("--------------------");
  Serial.println("DC VOLTMETER");
  Serial.print("Maximum Voltage: ");
  Serial.print((int)(vPow / (r2 / (r1 + r2))));
  Serial.println("V");
  // Read AD and convert value
  float v = (analogRead(0) * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  Serial.print("Battery voltage: ");
  Serial.print(v2);
  Serial.println(" volt.");
  Serial.println("--------------------");
  
  Serial.println("Setup done");
}

void loop() {
  //Distance sensor
  long duration, distance;
  
  // RC
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint8_t bUpdateFlags;

  // Measure distance to sensor
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance < 10) {  
    // We are close to something
    digitalWrite(lednear,HIGH); 
    // Stop cutter
    digitalWrite(cutmotor, LOW);
    // Back off
    analogWrite(speedPinA, 255); // Full speed 
    analogWrite(speedPinB, 255);
    // Backwards
    digitalWrite(dir1PinA, HIGH);
    digitalWrite(dir2PinA, LOW);
    digitalWrite(dir1PinB, HIGH);
    digitalWrite(dir2PinB, LOW);
    delay (2000);
    // Turn right
    digitalWrite(dir1PinA, LOW);
    digitalWrite(dir2PinA, HIGH);
    digitalWrite(dir1PinB, HIGH);
    digitalWrite(dir2PinB, LOW);    
    delay (2000);
    digitalWrite(lednear,LOW); 
}
  else {
    digitalWrite(lednear,LOW);
  }
  /*
  if (distance >= 200 || distance <= 0){
    //Serial.println("Out of range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }
  */

  // Measure battery voltage
  // Read AD and convert value
  float v = (analogRead(0) * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  Serial.print("Battery voltage: ");
  Serial.print(v2);
  Serial.println(" volt.");
  if (v2<10.5) {  // Battery is low
      // Everything off
      digitalWrite(cutmotor, LOW);
      analogWrite(speedPinA, 0); 
      analogWrite(speedPinB, 0);
      digitalWrite(ledstuck, LOW);
      digitalWrite(lednear, LOW);
      while(1){   // Loop forever
        // Blink D13 to indicate low power
        digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(200);              // wait for a second
        digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
        delay(1500);              // wait for a second
      }
  }
  
  
  // Measure total current
  RawValue = analogRead(analogIn);
  Voltage = (RawValue / 1023.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  /*
  Serial.print("Raw Value = " ); // shows pre-scaled value 
  Serial.print(RawValue); 
  Serial.print("\t mV = "); // shows the voltage measured 
  Serial.print(Voltage,3); // the '3' after voltage allows you to display 3 digits after decimal point
  Serial.print("\t Amps = "); // shows the voltage measured 
  Serial.println(Amps,3); // the '3' after voltage allows you to display 3 digits after decimal point
  */
  double posamps = abs(Amps);
  Serial.println(posamps,3);
  delay(300); 
  if (posamps>3) {
    // The current drawn is to high, the lawnmower is stuck somehow.
    // Shut everything of and wait for a reset (manual action). 
    digitalWrite(cutmotor, LOW);
    analogWrite(speedPinA, 0); 
    analogWrite(speedPinB, 0);
    digitalWrite(ledstuck, HIGH);
    while(1){}
  }
  
 /*
 Any change on the RC-receiver?
 */
 // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); 
    bUpdateFlags = bUpdateFlagsShared;
    
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
  
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
    bUpdateFlagsShared = 0;
    interrupts(); 
  }
 if(bUpdateFlags & THROTTLE_FLAG)
  {
    //CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,unThrottleIn);
    if (unThrottleIn<1480 || unThrottleIn > 1530) {
      // The throttle has changed
      Serial.println(unThrottleIn);
    }

    
    // Throttle is pushed down
    // Max value is 1055
    if (unThrottleIn<1480 && unThrottleIn>1000) {
        analogWrite(speedPinA, 255);//Sets speed variable via PWM 
        analogWrite(speedPinB, 255);
        digitalWrite(dir1PinA, HIGH);
        digitalWrite(dir2PinA, LOW);
        digitalWrite(dir1PinB, HIGH);
        digitalWrite(dir2PinB, LOW);
        // Wait a second and start cutter
        delay(200);
        digitalWrite(cutmotor, HIGH);
    }
    // Throttle is pushed up
    // Min value is 1953
    else if (unThrottleIn>1530) {
        analogWrite(speedPinA, 255);//Sets speed variable via PWM 
        analogWrite(speedPinB, 255);
        digitalWrite(dir1PinA, LOW);
        digitalWrite(dir2PinA, HIGH);
        digitalWrite(dir1PinB, LOW);
        digitalWrite(dir2PinB, HIGH);
        // Wait a second and start cutter
        delay(200);
        digitalWrite(cutmotor, HIGH);
    }
    else {
      // Stop drivers and cutter
      analogWrite(speedPinA, 0);
      analogWrite(speedPinB, 0);
      digitalWrite(cutmotor, LOW);
    }
    // Dont react to distortion
    if (unThrottleIn<1000) {
      analogWrite(speedPinA, 0);
      analogWrite(speedPinB, 0);
    }
  }

  if(bUpdateFlags & STEERING_FLAG)
  {
    //CRCArduinoFastServos::writeMicroseconds(SERVO_STEERING,unSteeringIn);
    if (unSteeringIn<1480 || unSteeringIn > 1530) {
      // The steering has changed
      Serial.println(unSteeringIn);
    }
    // Steering is to the right
    // Min value is 1058
    if (unSteeringIn < 1480 && unSteeringIn > 1000) {
        analogWrite(speedPinA, 255);//Sets speed variable via PWM 
        analogWrite(speedPinB, 255);
        digitalWrite(dir1PinA, LOW);
        digitalWrite(dir2PinA, HIGH);
        delay(200);
        digitalWrite(dir1PinB, HIGH);
        digitalWrite(dir2PinB, LOW);
    }
    
    // Steering is to the left
    // Max value is 1982
    else if (unSteeringIn > 1530 && unSteeringIn <2000) {
      
        analogWrite(speedPinA, 255);//Sets speed variable via PWM 
        analogWrite(speedPinB, 255);
        digitalWrite(dir1PinA, HIGH);
        digitalWrite(dir2PinA, LOW);
        delay(200);
        digitalWrite(dir1PinB, LOW);
        digitalWrite(dir2PinB, HIGH);    
      
    }
    else {
      dir = 0;
    }
    
  }
  bUpdateFlags = 0;
 
 
  /* 
  Rest for a while
  */ 
  
  
  //delay(50); // Poll every 50ms
  
  
}

// simple interrupt service routine
void calcThrottle()
{
  if(PCintPort::pinState)
  {
    unThrottleInStart = TCNT1;
  }
  else
  {
    unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(PCintPort::pinState)
  {
    unSteeringInStart = TCNT1;
  }
  else
  {
    unSteeringInShared = (TCNT1 - unSteeringInStart)>>1;

    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

