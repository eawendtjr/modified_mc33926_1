// A program to control the SAR Controller
// Written by: Group 26

/* The controller's main functions are to send out driving data to the SAR Rover, tell the Rover to 
 *  test the snow, receive back snow pack data in live time, test the slope angle, and to shut off the 
 *  emergency system if needed. All data is displayed to a 1.8" TFT LCD Screen that has a customized 
 *  main screen to show the user all the button functions. The screen also displays data coming from
 *  the Rover in real time. This program has not been taken from some other controller program, however,
 *  the method of converting integers to characters when sending data and vice versa when receiving data 
 *  was learned using the following thread: http://forum.arduino.cc/index.php?topic=195618.0. Some of the 
 *  code may be found somewhere online, however, other than using the aforementioned thread for reference,
 *  the code was written by the group members.
 *  
 *  The .h files for the included libraries were refered to when implementing sensor and TFT funcitonality.
 *  Please see these files for more information.
 */

 /****************************************************************************************************************************************************************************************************/

// GLOBAL VARIABLES AND LIBRARIES

// Radio Communication
#include <VirtualWire.h>
#define tx 3 // transmit pin
#define rx 5 // receiver pin
char incoming[20]; // incoming data character array
char outgoing[20]; // outgoing data character array

// Joysticks
#define LeftJoystick A2 // left joystick is connected to A2
#define RightJoystick A0 // right joystick is connected to A0
int LeftValue; // stores the value of the left joystick
int RightValue; // stores the value of the right joystick

// Buttons
#define mode 2 // drive mode or probe mode button
int ModeState; // state of the mode button
#define testbutton 7 // test button
int TestState; // test button state
int testspotstate; // store value of incoming slope angle
int testarray[3]; // an array to store values of incoming slope angle
int maxangle; // max angle received back
#define probe_cancel 12 // cancel button 
int Cancel; // stores value of cancel button
#define TempPin 4 // shut off emergency system button
int TempDisplay; // the state of the Temperature display button

// TFT LCD Screen
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h> // SPI library
#define TFT_CS    10 // setting the hardware pins
#define TFT_RST   9 
#define TFT_DC    8
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST); // creating a TFT object
int y_pos = 11; // initial starting location for the plot
int drawWidth; // the width of each rectangle drawn

// Temperature Sensor
#include <Wire.h> // necessary 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
int temp; // stores temperature value
int altitude; // stores altitude value
int humidity; // stores relative humidity value

// Incoming data
int angle; // incoming angle data
int displayangle; // stores angle to display on screen
int load; // incoming load data

// Buzzer
#define buzzer 6 // piezoelectric buzzer hooked to pin 6

/****************************************************************************************************************************************************************************************************/

void setup() 
{
  // Setup serial interface
  Serial.begin(9600); // baud rate
  Serial.print("Initializing... ");

  // Radio communication
  vw_set_tx_pin(tx); // set tx pin
  vw_setup(2000); // baud rate
  vw_set_rx_pin(rx); // set rx pin
  
  // Button setup
  pinMode(mode, INPUT); // declaring buttons as inputs
  pinMode(testbutton, INPUT);
  pinMode(TempPin, INPUT);
  pinMode(probe_cancel, INPUT);

  // Buzzer Setup
  pinMode(buzzer, OUTPUT); // declare the buzzer as an output
  
  
  // Temp sensor Setup
  if (!bme.begin()) // checks if there is a temp sensor hooked up
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // TFT LCD Screen Setup
  tft.initR(INITR_BLACKTAB); // standard setup procedure
  delay(100); // pause to give the screen time to boot up
  CreateStartup(); // start up screen

  Serial.println("Initialized");
}

/****************************************************************************************************************************************************************************************************/

void loop() 
{
  GetData(); // gathers user data
  SendData(); // sends data to the SAR rover
  ReceiveData(); // receives data back if in probe mode
  TestLocation(); // receives data back if in test mode
  DisplayWeather(); //checks if weather measurments are desired
}

/****************************************************************************************************************************************************************************************************/

// A function to create an intro message
void CreateStartup()
{
  //Fill screen with black
  tft.fillScreen(ST7735_RED); // background is black

  //Set the orientation to horizontal
  tft.setRotation(3); 

  tft.setCursor(10,10);
  tft.setTextColor(ST7735_YELLOW); // red text color
  tft.setTextSize(2); // text size

  tft.print("Smart"); // print words
  tft.setCursor(10,30);
  tft.println("Avalanche");
  tft.setCursor(10,50);
  tft.print("Rover");
  tft.setCursor(10,70);
  tft.print("(SAR)");

  StartUpSong();
  
  delay(1000);
  GetTemp();
  delay(3000);
  MainMenu();
  
}

/****************************************************************************************************************************************************************************************************/

// A function to initially display temp data to provide initial weather gage for the user
void GetTemp()
{
  int inittemp = bme.readTemperature();
  
  if(inittemp >= 21)
  {
    tft.fillScreen(ST7735_RED);
    tft.setCursor(20, 15);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Temp = ");
    tft.print(inittemp);
    tft.print("C");
    tft.setCursor(20, 60);
    tft.setTextSize(1);
    tft.print("It's nice out today!");
  }

  if(inittemp < 21 && inittemp >= 15)
  {
    tft.fillScreen(ST7735_MAGENTA);
    tft.setCursor(20, 15);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Temp = ");
    tft.print(inittemp);
    tft.print("C");
    tft.setCursor(17, 60);
    tft.setTextSize(1);
    tft.print("It's not too bad out!");
  }

  if(inittemp < 15 && inittemp >=0)
  {
    tft.fillScreen(ST7735_GREEN);
    tft.setCursor(20, 15);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Temp = ");
    tft.print(inittemp);
    tft.print("C");
    tft.setCursor(15, 60);
    tft.setTextSize(1);
    tft.print("Better put on a jacket!");
  }

  if(inittemp< 0)
  {
    tft.fillScreen(ST7735_BLUE);
    tft.setCursor(25, 15);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.print(inittemp);
    tft.print("C");
    tft.setCursor(20, 60);
    tft.setTextSize(1);
    tft.print("It's chilly out here!");
  }
}

/****************************************************************************************************************************************************************************************************/

// A function to display a main menu
void MainMenu()
{
 
  tft.setTextSize(1.5);
  tft.setTextColor(ST7735_GREEN);
  delay(250);
  tft.fillRect(0,0,80,64, ST7735_BLUE); // fill 1/4 of scree
  tft.setCursor(10,20);
  tft.print("Check"); // print button name
  tft.setCursor(10, 30);
  tft.print("Weather");
  delay(330);

  tft.fillRect(80,64,160,128, ST7735_BLUE); // fill another 1/4 screen
  tft.setCursor(120, 95);
  tft.print("Test");
  tft.setCursor(120,105);  // print button name
  tft.print("Snow");
  delay(330);
 
  tft.fillRect(80,0,160,64, ST7735_BLUE); // fill another 1/4 screen
  tft.setCursor(120, 20);
  tft.print("Cancel");
  tft.setCursor(120, 30); // print buton name
  tft.print("Test");
  delay(330);
  
  tft.fillRect(0,64,80,128, ST7735_BLUE); // fill last 1/4 screen
  tft.setCursor(10, 95);
  tft.print("Test");
  tft.setCursor(10,105); // print button name
  tft.print("Location");
  
}

/****************************************************************************************************************************************************************************************************/

void StartUpSong()
{
  /*  note  frequency
  c     262 Hz
  d     294 Hz
  e     330 Hz
  f     349 Hz
  g     392 Hz
  a     440 Hz
  b     494 Hz
  C     523 Hz */
  delay(100);
  tone(buzzer, 262); // a tune to play at startup
  delay(150);
  tone(buzzer, 330);
  delay(150);
  tone(buzzer, 392);
  delay(150);
  tone(buzzer, 523);
  delay(300);
  tone(buzzer, 2*330);
  delay(150);
  tone(buzzer, 523);
  delay(500);
  noTone(buzzer);

}

/****************************************************************************************************************************************************************************************************/

// A function to gather values from user 
void GetData()
{

  LeftValue = analogRead(LeftJoystick); // analog read joystick values
  RightValue = analogRead(RightJoystick);

  LeftValue = constrain(LeftValue, 15, 989); // constrain the joystick ranges
  RightValue = constrain(RightValue, 15, 989);

  LeftValue = map(LeftValue, 15, 989, -150, 150); // remap the joystick values
  RightValue = map(RightValue, 15, 989, -150, 150);

  ModeState = digitalRead(mode); // read mode button

  TestState = digitalRead(testbutton); // read test button
 
  TempDisplay = digitalRead(TempPin); // read emergency override button

  //The following logic quantizes the motor speed values. Please note that the joystick
  //values are rpm which is why they don't go up to 255. The PID control system on the rover
  //converts the desired RPM values to varying PWM commands to keep the actual RPM near that
  //specified by the user.

  // Right Joystick - making discrete values for the joysticks
  if(RightValue <= 5 && RightValue >= -5) //sends 0 if the joystick value is close to zero
  {
    RightValue = 0;
  }

  if(RightValue <= 150 && RightValue > 140)
  {
    RightValue = 150;
  }

  if(RightValue < -140 && RightValue >= -150)
  {
    RightValue = -150;
  }

  // Left Joystick - making discrete values for the left joystick
  if(LeftValue <= 5 && LeftValue >=-5) //sends 0 if the joystick value is close to zero
  {
    LeftValue = 0;
  }

  if(LeftValue <= 150 && LeftValue > 140)
  {
    LeftValue = 150;
  }

  if(LeftValue < -140 && LeftValue >= -150)
  {
    LeftValue = -150;
  }

}

/****************************************************************************************************************************************************************************************************/

// A function to send the data to the rover
void SendData()
{
   sprintf(outgoing, "%d,%d,%d,%d.", LeftValue, RightValue, ModeState, TestState); // convert integers into a character array that is comma diliminated
   Serial.print("Sending: ");
   Serial.println(outgoing);

   vw_send((uint8_t*)outgoing, strlen(outgoing)); // send data
   vw_wait_tx(); // wait for data to send

}

/****************************************************************************************************************************************************************************************************/

// A funciton to receive data
void ReceiveData()
{
  
  if(ModeState == 1) // if in probe mode
  {
    SendData(); // send a pack of data
    vw_rx_start(); // start receiving data
    CreateBackground(); // fill screen with black
    FormatChart(); // create graph axes
    y_pos = 11; // starting point for graph
    int t = 0;
    while(t<20) 
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
        load = atoi(strtok(incoming, ",")); // convert values from characters to integers
        angle = atoi(strtok(NULL, ",")); // has to gather data in between the commas
        int tt = atoi(strtok(NULL, ";")); // stop at a semi colon and tt shows what packet is sent

        displayangle = angle;
        if(angle > displayangle)
        {
          displayangle = angle;
        }

        if(tt>=21)
        {
          tt=0; // this way t is always reset on another consecutive probe. 
        }
        t = tt; // in case one packet isn't recieved, it will skip to the correct t value
        
        Graph(); // graph data
        if(tt == 2)
        {
          DisplayData(); // only prints data once
        }

        // debugging comments
        Serial.print("Got: ");
        Serial.print(load);
        Serial.print(", ");
        Serial.print(angle);
        Serial.print(", and t = ");
        Serial.println(tt);
   
      }
      
      Cancel = digitalRead(probe_cancel); // read the cancel button
      Serial.print("cancel button state = ");
      Serial.println(Cancel);
      if(Cancel == 1) // if the cancel button is pressed
      {
        char* A = "S"; // a variable to send

        for(int i = 0; i<=9; i++) // send the data 9 times for redundancy
        {
          vw_send((uint8_t*)A, strlen(A));  // send data
          vw_wait_tx();  // wait for data to be sent
          delay(10); // short delay
          Serial.print("Probe canceled, sending ");
          Serial.println(A);
          
        }
        CreateBackground(); // create black background
        tft.setCursor(30,50);
        tft.setTextSize(3); // text size
        tft.setTextColor(ST7735_RED); // red font
        tft.print("Cancel"); // show user that it is canceling  
        delay(1000);
        MainMenu(); // go back to main menu
        break;
      }
    }
    tone(buzzer, 523); // tone to alert user
    delay(500);
    noTone(buzzer);
    vw_rx_stop(); // stop receiving
  }

}

/****************************************************************************************************************************************************************************************************/

void DisplayWeather() // A function to display the weather on command
{
  if(TempDisplay == 1)
  {
    tft.fillScreen(ST7735_CYAN);//Pretty background

    //get all the current weather data
    int current_pressure = bme.readPressure()/1000; // convert to kPa
    int current_humidity = bme.readHumidity(); // percent humidity
    int current_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // altitude
    int current_temp = bme.readTemperature(); // temperature

    tft.setTextSize(1);
    tft.setTextColor(ST7735_BLACK); //Set the text color to black

    tft.setCursor(15, 15); //top left corner
    tft.print("Pressure: "); tft.print(current_pressure); tft.print("kPa");
    tft.setCursor(15, 30);
    tft.print("Humidity: "); tft.print(current_humidity); tft.print("%");
    tft.setCursor(15,45);
    tft.print("Altitude: "); tft.print(current_altitude); tft.print("m");
    tft.setCursor(15,60);
    tft.print("Temperature: "); tft.print(current_temp); tft.print("C");
  }
  
}

/****************************************************************************************************************************************************************************************************/

void CreateBackground()
{
  //Fill screen with black
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(3); //Set the orientation to horizontal
}

// A function to create the axes for the LCD screen
void FormatChart()
{
  //create vertical axis
  tft.drawFastVLine(10, 10, tft.height()-20, ST7735_BLUE);

  //create horizonal axis
  tft.drawFastHLine(10, tft.height()-10, tft.width()-20, ST7735_BLUE);

  //title the chart
  tft.setCursor(30,0);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(1);
  tft.print("Depth vs Pressure");
  
  //title x axis
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(85, tft.height()-9);
  tft.print("200kPa");

  //title x axis
  tft.setCursor(0,0);
  tft.print("0");
  tft.print("m");
  tft.setCursor(0,tft.height() - 9);
  tft.print("0.5");
  tft.print("m");

 
}

/****************************************************************************************************************************************************************************************************/

// A function to graph the received values
void Graph()
{
  drawWidth = (abs(load))*0.5; // scale load readings
  if(drawWidth > 85)
  {
    drawWidth = 85;
  }
  tft.fillRect(11, y_pos, drawWidth, 5, ST7735_WHITE); // draw a rectangle
  y_pos=y_pos+5; // move cursor down 5 pixels
}

/****************************************************************************************************************************************************************************************************/

// A function to display slope and temp data
void DisplayData()
{
  tft.setCursor(100, 15);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_GREEN);

  tft.print("Temp:"); // prints temp data
  tft.setCursor(100, 25);
  tft.print(bme.readTemperature());
  tft.print("C");

  tft.setCursor(100, 40); // prints humidity data
  tft.print("Humidity:");
  tft.setCursor(100, 50);
  tft.print(bme.readHumidity());
  tft.print("%");

  tft.setCursor(100, 65); // prints altitude data
  tft.print("Altitude:");
  tft.setCursor(100, 75);
  tft.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  tft.print("m");

  tft.setCursor(100, 90); // prints slope angle data
  tft.print("Angle:");
  tft.setCursor(100, 100);
  tft.print(abs(angle));
}

/****************************************************************************************************************************************************************************************************/

// A function to test whether to probe or not
void TestLocation()
{
  if(TestState == 1) // is the test button pressed
  {
    SendData(); // send a packet of data
    vw_rx_start(); //start receiving
    int testarray[3] = {0, 0, 0}; // array to store values
    int t = 0;
    while(t<=2) 
    {
      
      uint8_t buf[VW_MAX_MESSAGE_LEN]; // data buffer
      uint8_t buflen = VW_MAX_MESSAGE_LEN; // buffer length
      word timeout_ms = 10000;
      
      if(vw_wait_rx_max(timeout_ms)) // wait for a message unless it times out
      {
      
        if(vw_get_message(buf, &buflen)) // if good data is received
        {
          int i;
  
          for (i = 0; i < buflen; i++)
          {
            incoming[i] = buf[i]; // store data to an array
          }
          testspotstate = atoi(strtok(incoming, ".")); // decode data and convert to integer
          t++;
        
          if(t == 1) // store values to the array
          {
            testarray[0] = testspotstate;
          }

          else if(t == 2)
          {
            testarray[1] = testspotstate;
          }

          else if(t == 3)
          {
            testarray[2] = testspotstate;
          } 

          // Getting absolute values of slope angles
          int firstvalue = abs(testarray[0]);
          int secondvalue = abs(testarray[1]);
          int thirdvalue = abs(testarray[2]);
        
          if(t = 3) // get max slope angle
          {
           maxangle = max(firstvalue, secondvalue); // max of first and second
           maxangle = max(maxangle, thirdvalue); // max of third and whatever was returned above
          }
          // debugging only
        
          Serial.print("Got: ");
          Serial.println(maxangle);
        
          Buzzer(); // display data and sounds buzzer according to angle
        }
      
      }

      else // if it times out
      {
        tone(buzzer, 262);
        delay(250);
        noTone(buzzer);
        break; // break out of while loop
      }
    }
    vw_rx_stop(); // stop receiving data
  }
}

/****************************************************************************************************************************************************************************************************/

// A function to alert user of the slope angle
void Buzzer()
{
  if(maxangle >= 20 && maxangle <30) // if angle is within 20 to 30 degrees
  {
    AngleDisplay(); // display angle
    for(int a = 0; a < 5; a++) // play a tone swequence 5 times
    {
      tone(buzzer, 500);
      delay(200);
      noTone(buzzer);
      delay(200);
    }
  }

  if(maxangle >= 30 && maxangle <= 45) // if it is within 30 and 45 degrees
  {
    AngleDisplay(); // display angle
    for(int b = 0; b < 5; b++) // play a tone 5 times
    {
      tone(buzzer, 750);
      delay(100);
      noTone(buzzer);
      delay(100);
    }
  }

  if(maxangle > 45) // if it is greater than 45 degrees
  {
    AngleDisplay(); // display angle
    for(int c = 0; c < 5; c++) // play tone 5 times
    {
      tone(buzzer, 1000);
      delay(50);
      noTone(buzzer);
      delay(50);
    }
  }

  else if(maxangle <20) // if anlge is less than 20 degrees
  {
    AngleDisplay(); // display angle
    tone(buzzer, 100);
    delay(200);
    noTone(buzzer);
  }
}

/****************************************************************************************************************************************************************************************************/

// A function to display angle on screen on command
//This function displays the numerical value in addition to drawing the angle with lines on the TFT screen
void AngleDisplay()
{
  tft.fillScreen(ST7735_RED);
  tft.setTextSize(3);
  const float pi = 3.14159;
  const int x_pos1 = 30;
  const int y_pos1 = 80;
  const int x_pos2 = 120;
  const int bottomlength = x_pos2 - x_pos1;
  int y_pos2;
  // calculations:
  float temp_y;//temporary y that will be later converted to an integer
  float slope = maxangle*pi/180; //must convert back to radians to use the arduinos tangent function
  
  //formulate the position of the of the hypotenuse based on the recorded angle
  temp_y = y_pos1 - bottomlength*tan(slope);
  y_pos2 = (int)temp_y;

  // draw a horizontal line as 0 reference
  tft.drawFastHLine(x_pos1, y_pos1, bottomlength, ST7735_BLUE);

  if(y_pos2 < y_pos1)
  {
    tft.drawLine(x_pos1, y_pos1, x_pos2, y_pos2, ST7735_BLUE);
  }

  else
  {
    y_pos2 = y_pos1; //only display the absolute value of angles
    tft.drawLine(x_pos1, y_pos1, x_pos2, y_pos2, ST7735_BLUE);
  }

  // Print angle values
  tft.setTextColor(ST7735_BLUE);
  tft.setCursor(55, 95);
  
  //The following logic puts a zero in front of numbers less than 10 so the
  //degree symbol lines up properly
  if(maxangle<10)
  {
    tft.print("0");
    tft.print(maxangle);
  }
  else
  {
    tft.print(maxangle);
  }
  
  // Print degree sign
  tft.setTextSize(1);
  tft.setCursor(90, 90);
  tft.print("o");
  
}

/****end****/
