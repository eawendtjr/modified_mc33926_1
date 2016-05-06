// A program to control the SAR Rover
// Written by: Group 26

/* The following program primarily serves to run the PID control system for the motors
  and to transmit data collected back to the controller. The PID control system automatically
  compensates for changing drive conditions to ensure that the RPM specified by the user is maintained 
  by the treads. There is a separate PID loop for each motor allowing for very precise control. PID also
  allows the rover to drive up severe slopes at very slow speeds which avoids stalling the motors. 
  This rover program also acquires slope angle data via an accelerometer and sends it to the user when promted. 
  Additionally, the data used to display a pressure vs. depth profile is acquired, processed and sent to the 
  controller in this program.
  
  The following forum post was referenced to learn the basics of PID control and arduino implementation: 
  http://forum.arduino.cc/index.php?topic=8652.0 
  Additionaly our friend Trevor Von Seggern provided advice regarding PID theory and tuning. The PID algorithim 
  was custom written by members of group 26 for our particular application. 
  
  This program has not been taken from another program, however,
  the method of converting integers to characters when sending data and vice versa when receiving data 
  was learned using the following thread: http://forum.arduino.cc/index.php?topic=195618.0.
  
  The equation to derive pressure from current into the probe module was experimentally determined for our
  particular servo and probe module design.
  
  The other reference information used can be found in the .h files of the included libraries.
 */

/****************************************************************************************************************************************************************************************************/

// Radio Communication definitions
#include <VirtualWire.h>
#define rx 5 // receive pin
#define tx 4 // transmit pin
char outgoing[20]; // outgoing data character array
char incoming[20]; // incoming data character array

// Incoming data storage variables
int LeftMotorValue;
int RightMotorValue;
int Mode; // probe mode or drive mode
int Cancel; // cancel probing

// Accelerometer Setup
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
// Uses pins A4 and A5

// Assign a unique ID to the accelerometer 
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// An integer to get tilt angle
int slopeangle;

// Test location variable setup
int testlocationvalue; 
int TestLocationState;

// Servo setup. Servo is controlled through the PIC16F88
#define ServoPower 8 // This allows power to run to servo
#define servopin1 9 // Two pins to control servo direction
#define servopin2 10

// Current Sensor
#define gains A0 // attach current sensor to measure servo torque
int noloadcurrent; // reference value for pressure measurment
int currentpressure; // pressure value during probe
// Photo interrupter variables
#define photo 13 // pin the photointerupter is attached to
int photostate_lower; // checks if counting photointerrupter is high or low on lowering
int photostate_raise; // checks if the photointerrupter is high or low on raising
int r; // a variable that is used to reverse the probe automatically

// Motor setup
#define motorL 6 // left motor speed
#define dirL 7 // left motor direction
#define motorR 11 // right motor speed
#define dirR 12 // right motor directions

// PID control setup
#define encoderR 2 //Interrupt pins are used to read the encoders
#define encoderL 3
#define PIDLOOPTIME 100 //The PID will update every 100ms

volatile long countR = 0; //Number of encoder triggers
volatile long countL = 0;

unsigned long last_ms = 0; //PID timer variable

int actual_speedR = 0; //tread rpm
int actual_speedL = 0;

int PWMR = 0; //initialize the 
int PWML = 0;

//PID Constants: These values reach the setpoint quickly and minimize overshoot
float kp_r = 0.4; // constants for right motor
float kd_r = 1.0;
float ki_r = 0.01;

float kp_l = 0.4; // constants for left motor (the same constants work well for both motors)
float kd_l = 1.0;
float ki_l= 0.01;

//define the integral active zone
//The integral term is very small for this project and is only used for very fine changes; hence the small active zone
const float integralActive = 5.0;//The integral term will not accumulate if the error is too large (integral is only used to make small corrections)

/****************************************************************************************************************************************************************************************************/

void setup() 
{
  // Setup serial interface
  Serial.begin(9600);
  Serial.print("Intializing... ");
  
  // radio communication setup
  vw_set_rx_pin(rx);
  vw_setup(2000); // data rate
  vw_rx_start(); // start receiving
  vw_set_tx_pin(tx);

  // checks if the lsm303
  if(!accel.begin())
  {
    //There was a problem detecting the ADXL345 ... check your connections 
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

 // servo motor initialization
 pinMode(servopin1, OUTPUT);
 pinMode(servopin2, OUTPUT);
 pinMode(ServoPower, OUTPUT);
 // initial conditions
 digitalWrite(servopin1, LOW);
 digitalWrite(servopin2, LOW);
 digitalWrite(ServoPower, LOW);
  

 // photo interupter initialization
 pinMode(photo, INPUT);
 photostate_lower = 0;
 photostate_raise = 0;

 // Motors
 pinMode(motorL, OUTPUT);
 pinMode(motorR, OUTPUT);
 pinMode(dirL, OUTPUT);
 pinMode(dirR, OUTPUT);
 pinMode(encoderR, INPUT);
 pinMode(encoderL, INPUT);
 
 //internal pullups for encoder pins
 digitalWrite(encoderR, HIGH); 
 digitalWrite(encoderL, HIGH);

 //initialize the motor directions to forward and the speed to zero
 digitalWrite(dirR, 1);
 digitalWrite(dirL, 1);
 analogWrite(motorR, PWMR);
 analogWrite(motorL, PWML);
 
 Serial.println("Initialized");
}

/****************************************************************************************************************************************************************************************************/

void loop() 
{
  IncomingData();
}

/****************************************************************************************************************************************************************************************************/

// A function to receive and decode the incoming data
void IncomingData()
{
  
  uint8_t buf[VW_MAX_MESSAGE_LEN]; // a buffer for incoming data
  uint8_t buflen = VW_MAX_MESSAGE_LEN; // length of the buffer

  
  if(vw_get_message(buf, &buflen)) // checks if good data is received
  {
    int i;

    for (i = 0; i < buflen; i++)
    {
      incoming[i] = buf[i]; // store incoming message to data array
    }

    LeftMotorValue = atoi(strtok(incoming, ",")); // convert values from characters to integers
    RightMotorValue = atoi(strtok(NULL, ","));    // has to gather data in between the commas
    Mode = atoi(strtok(NULL, ","));               // stop at a period
    TestLocationState = atoi(strtok(NULL, "."));
    //Shutoff = atoi(strtok(NULL, "."));

    // Print data to serial monitor
    Serial.print("Got: ");
    Serial.print(LeftMotorValue);
    Serial.print(", ");
    Serial.print(RightMotorValue);
    Serial.print(", ");
    Serial.print(Mode);
    Serial.print(",");
    Serial.println(TestLocationState);
    //Serial.print(", and ");
    //Serial.println(Shutoff);
    
    WriteData(); //send the requested data packet
    TestLoc(); //check if an angle measurment is requested
    
  }
  
}

/****************************************************************************************************************************************************************************************************/

// A function to write incoming data to the motors
void WriteData()
{
  if(Mode == 0)
  {
    //To ensure proper data transmission/reception, interrupts for encoder counting
    //are only available in drive mode
    attachInterrupt(digitalPinToInterrupt(encoderR), read_encoderR, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoderL), read_encoderL, FALLING);
    
    if((millis() - last_ms) >= PIDLOOPTIME) // The PID updates every 100ms
    {
      last_ms = millis(); //updates the time comparison variable
      
      if(RightMotorValue >= 0)
      {
        digitalWrite(dirR, 1); //forward
      }

      if(RightMotorValue < 0)
      {
        digitalWrite(dirR, 0); //reverse
      }

      if(LeftMotorValue >= 0)
      {
        digitalWrite(dirL, 1); //forward
      }
  
      if(LeftMotorValue < 0)
      {
        digitalWrite(dirL, 0); //reverse
      }

      // calculate the speed based on encoder measurements
      get_speedR();
      get_speedL();

      if(RightMotorValue == 0)
      {
        PWMR = 0; // make sure the right motor does not move when the user is sending a value of zero
      }
      else
      {
        PWMR = updatePIDR(PWMR, abs(RightMotorValue), actual_speedR); //set the PWM based on the PID
      }

      if(LeftMotorValue == 0)
      {
        PWML = 0; // make sure the right motor does not move when the user is sending a value of zero
      }
      else
      {
        PWML = updatePIDL(PWML, abs(LeftMotorValue), actual_speedL); //set the PWM based on the PID
      }

      //run the motors at the intended speed based on PID input
      analogWrite(motorR, PWMR);
      analogWrite(motorL, PWML);
    }
    
    //detach the interupts prior to acquiring controller commands
    detachInterrupt(digitalPinToInterrupt(encoderR));
    detachInterrupt(digitalPinToInterrupt(encoderL));
    
  }

  if(Mode == 1) //It is in probe mode
  {
    //run the servo forward to drive the probe into the snow
    digitalWrite(ServoPower, HIGH);
    digitalWrite(servopin1, HIGH);
    digitalWrite(servopin2, LOW);

    //make sure the rover is stationary during probing
    analogWrite(motorR, 0); 
    analogWrite(motorL, 0);

    delay(500); // allow servo transients to diminish
    
    int t = 0; //counter variable to determine depth while lowering
    r = 0;
    //int time1 = millis();
    while(t<=21)
    {
      photostate_lower = digitalRead(photo);
      //Serial.print("Photostate = ");
      //Serial.println(photostate_lower);
        if(photostate_lower == 1) //The photo interrupter has reached a positive edge
        { 
          
          if(t == 0)
          {
           noloadcurrent = GetCurrent(); //calibrate the current before the entering the snow
           Serial.print("No Load Current: ");
           Serial.println(noloadcurrent);
          }
          
          int pressure = GetPressure();
          slopeangle = GetPitch();
          if(pressure < 0)
          {
            pressure = 0; //prevents negative values which do not display on the screen and have no physical meaning
          }
          
          sprintf(outgoing, "%d,%d,%d;", pressure, slopeangle, t); //send data to controller

          Serial.print("Sending: ");
          Serial.print(outgoing);
          Serial.print(" and t = ");
          Serial.println(t); //display current count on serial port

          vw_send((uint8_t*)outgoing, strlen(outgoing)); //radio send protocol
          vw_wait_tx;
          t++;
          r = 2*t + 4;
         }
     
      uint8_t buf[VW_MAX_MESSAGE_LEN];
      uint8_t buflen = VW_MAX_MESSAGE_LEN;

      //check if the user has sent a cancle probe command
      if(vw_get_message(buf, &buflen))
      {
        for(int i = 0; i<buflen; i++)
        {
          Cancel = buf[i]; 
        }

        if(Cancel == 'S') 
        {
          break; //exit the probe loop prematuraly if desired 
        }
      }
      
    }
    //int time2 = millis();
    digitalWrite(ServoPower, LOW);
    digitalWrite(servopin1, LOW);
    digitalWrite(servopin2, LOW);
    delay(1000);
    
    //int reverse_time = (time2 - time1); //times how long it took to lower the probe
    //reverse_time = reverse_time - reverse_time*0.015;
    
    //reverse the direction of the servo
    digitalWrite(ServoPower, HIGH); //make sure the servo is on 
    digitalWrite(servopin1, LOW);
    digitalWrite(servopin2, HIGH);
    //delay(reverse_time);
    
    // return to the original probe position by counting the same amount backward as was done forward
    while(r >= 0) 
    {
      photostate_raise = digitalRead(photo);
      if(photostate_raise == 1)
      {
        r--; //decrement the reverse variable at every count until it goes to zero
        Serial.print("r = ");
        Serial.println(r);
        delay(1); //the pulse is 500 miroseconds (1 count per pulse)
      }
    }

    //turn off servo after probe is complete
    digitalWrite(ServoPower, LOW);
    digitalWrite(servopin1, LOW);
    digitalWrite(servopin2, LOW);
    
  }
}

/****************************************************************************************************************************************************************************************************/

// A function to calculate the pitch using the accelerometer
float GetPitch() 
{
  sensors_event_t event; //Create a sensor event to access the library functionality
  accel.getEvent(&event); //Access data

  float pi = 3.1415926; //Defining pi to concert radians to degrees
  float pitch; 
    
  float A_x = event.acceleration.x; //Finding and storing the current value of X direction accleration as a float variable
  float A_y = event.acceleration.y; //Finding and storing the current value of Y direction accleration as a float variable
  float A_z = event.acceleration.z; //Finding and storing the current value of Z direction accleration as a float variable
    
  float A_xn;
    
  //normalize x acceleration to get a directional vector from which pitch can be calculated
  A_xn = (A_x/sqrt((A_x*A_x)+(A_y*A_y)+(A_z*A_z)));
    
  //calculate pitch using the arcsine function
  pitch = asin(-A_xn)*(180/pi)*(-1);
    
  return pitch;
}

/****************************************************************************************************************************************************************************************************/

// A function for the test location
void TestLoc()
{

  //make sure probe is not running while taking angle data
  digitalWrite(servopin1, LOW);
  digitalWrite(servopin2, LOW);
  
  if(TestLocationState == 1)
  {
    vw_rx_stop(); //briefly stop receiving data
    int t = 0;
    while(t<= 2)
    {
        testlocationvalue = GetPitch(); //call angle calculation function

        sprintf(outgoing, "%d.", testlocationvalue); //send the current angle value
        Serial.print("Sent: ");
        Serial.println(outgoing);
        
        vw_send((uint8_t*)outgoing, strlen(outgoing));
        vw_wait_tx;
        t++;
    }
    vw_rx_start(); //begin receiving data once more
  }
}

/****************************************************************************************************************************************************************************************************/

// A function to get current measurements
int GetCurrent()
{
  float RawValue = 0.0;
  float SamplesRaw = 0.0;
  float AvgRaw = 0.0;
  float Amps = 0.0; 
  float SamplesAmps = 0.0; 
  int AvgAmps = 0;
  float mVpermA = 5.49; // calibration factor used in current sensor 
  float ACSoffset = 93.25; // mV offset for 0 current
  double Voltage = 0;

  //loop to sum up 1000 successive current readings
  for(int d = 0; d < 1000; d++)
  {
    RawValue = analogRead(gains);
    SamplesRaw = SamplesRaw + RawValue;
    Voltage = (RawValue/1023.0)*5000;
    Amps = ((Voltage - ACSoffset)/(mVpermA));
    SamplesAmps = SamplesAmps +  Amps;   
   }
  
  //Get rid of noisy data by averaging 1000 measurments
  AvgAmps=SamplesAmps/1000;
  
  return AvgAmps;
}

/****************************************************************************************************************************************************************************************************/

// A function to convert change in current to pressure using an experimentally determined relationship

int GetPressure()
{
  currentpressure = (GetCurrent() - noloadcurrent)*9.81/1.88; // the current corresponds to the pressure in kPa
  return currentpressure;
}

/****************************************************************************************************************************************************************************************************/

void get_speedR()
{
  static long last_countR = 0;
  actual_speedR = ((countR - last_countR)*(60*(1000/PIDLOOPTIME)))/(16*34); // 16 pulses per revolution and 34 gear ration to convert to rpm
  last_countR = countR;
}

/****************************************************************************************************************************************************************************************************/

void get_speedL()
{
  static long last_countL = 0;
  actual_speedL = ((countL - last_countL)*(60*(1000/PIDLOOPTIME)))/(16*34); // 16 pulses per revolution and 34 gear ration to convert to rpm
  last_countL = countL;
}

/****************************************************************************************************************************************************************************************************/

/******************PID Algorithim loops*******************/
//The following functions provide closed loop control over each DC motor

int updatePIDR(int commandR, int desiredValueR, int currentValueR){
  //Define PID terms
  float PID_floatR = 0;
  int PID_R = 0;

  //Define error terms
  int errorR = 0;
  static int previous_errorR = 0; //static indicates that it is only initialized once
  errorR = abs(desiredValueR) - abs(currentValueR); //calculate error
  
  float P_R = kp_r * errorR; //Proportional term
  
  static float integralRawR = 0; //This term accumulates over the course of operation
  if (errorR < integralActive && errorR != 0)
  {
   integralRawR = integralRawR + errorR; //Term that accumlates error (will only accumulate near the target) 
  }
  else
  {
    integralRawR = 0; //Do not accumulate error far away from target or if the error is equal to zero
  }
  float I_R = ki_r * integralRawR;
  //Limit the integral term to stop it from getting out of control
  if(I_R > 10)
  {
    I_R = 10;
  }
  if(I_R < -10)
  {
    I_R = -10;
  }
  
  float D_R = kd_r * (errorR - previous_errorR); //Derivative term
  if (errorR == 0)
  {
    D_R = 0; //make sure derivative term does not move the value once set point is reached
  }
  
  PID_floatR = P_R + I_R + D_R;
  PID_R = int(PID_floatR); //convert the PID value to an integer
  
  previous_errorR = errorR; //update the error term
  
  return constrain(commandR + PID_R, 0, 255); //constrain the PWM command to never exceed 255 (max possible PWM)  
}

/****************************************************************************************************************************************************************************************************/

int updatePIDL(int commandL, int desiredValueL, int currentValueL){
  //Define PID terms
  float PID_floatL = 0;
  int PID_L = 0;
  
  //Define error terms
  int errorL = 0;
  static int previous_errorL = 0; //static indicates that it is only initialized once
  errorL = abs(desiredValueL) - abs(currentValueL); //calculate error
  
  float P_L = kp_l * errorL; //Proportional term
  
  static float integralRawL = 0; //This term accumulates over the course of operation
  if (errorL < integralActive && errorL != 0)
  {
   integralRawL = integralRawL + errorL; //Term that accumlates error (will only accumulate near the target) 
  }
  else
  {
    integralRawL = 0; //Do not accumulate error far away from target or if the error is equal to zero
  }
  float I_L = ki_l * integralRawL;
  //Limit the integral term to stop it from getting out of control
  if(I_L > 10)
  {
    I_L = 10;
  }
  if(I_L < -10)
  {
    I_L = -10;
  }
  
  float D_L = kd_l * (errorL - previous_errorL); //Derivative term
  if (errorL == 0)
  {
    D_L = 0; //make sure derivative term does not move the value far from the set point
  }
  
  PID_floatL = P_L + I_L + D_L;
  PID_L = int(PID_floatL); //convert the PID value to an integer
  
  previous_errorL = errorL; //update the error term
  
  return constrain(commandL + PID_L, 0, 255); //constrain the PWM command to never exceed 255 (max possible PWM) 
}

/******************End PID Algorithim Loops*******************/

/****************************************************************************************************************************************************************************************************/

//The following two functions update the encoder count upon 
void read_encoderR()
{
  countR++;                
}

/****************************************************************************************************************************************************************************************************/

void read_encoderL()
{
  countL++;                
}

/****end****/
