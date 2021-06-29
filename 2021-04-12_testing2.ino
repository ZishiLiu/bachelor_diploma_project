/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   If you have other time consuming code running (i.e. a graphical LCD), consider calling update() from an interrupt routine,
   see example file "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/

#include <HX711_ADC.h>
#include <Wire.h>
#include <SPI.h>
#include <String.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include<SoftwareSerial.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//HX711 ADC---------------------------------------------
//pins:
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin
//define
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;

float weight=0;

//OLED---------------------------------------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 OLED(OLED_RESET);


//Buzzer---------------------------------------------------------
  int buzzPin=10;


//BlueTooth----------------------------------------------------------
//SoftwareSerial btSerial(2,3); // HC-06, RXD connect with pin2, TXD connect with pin3 through a voltage divider(5V to 3.3V)



//Button----------------------------------------------------------
  //Button 1: increase setValue, pin 3
  int but1Pin=9;  int but1State=0;
  int LED1=12;  //Led indicator for increase button (but1)
  
  //Button 2: increase setValue
  int but2Pin=8;  int but2State=0;
  int LED2=11;  //Led indicator for decrease button (but2)





void setup() { //---------------------------------------------------
 
//-----Local/BT serial monitor
  Serial.begin(9600); delay(10);
  Serial.println("Starting...");
  Serial.println();
  //btSerial.begin(9600); delay(10); //HC-06 default serial budrate is 9600
  

//-----HX711 ADC
  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 696.0; // uncomment this if you want to set the calibration value in the sketch
  #if defined(ESP8266)|| defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
  #endif
  //EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
  
//-----OLED
  OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0,0);
  OLED.println("Digital Scale");
  OLED.display();
  delay(1500);
  OLED.clearDisplay();

//-----Buzzer
  pinMode(buzzPin,OUTPUT);

//-----Button
  pinMode(but1Pin, INPUT);    pinMode(LED1,OUTPUT);
  pinMode(but2Pin, INPUT);    pinMode(LED2,OUTPUT);



}

float setValue=10.00; //value we set to be met

void loop() { //----------------------------------------------------------

//-----Bluetooth-----receive setValue from phone
  if (Serial.available())
  {
    String tmp;
    Serial.println("serial ava");
    tmp= (Serial.readString());
    setValue = tmp.toFloat();
  }


//-----HX711-----read values from loadcell
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      weight=i;
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      Serial.print("setValue: ");
      Serial.println(setValue);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

  
//-----Buzzer-----
  if ( abs(weight) > (setValue-5) )
  {
    digitalWrite(buzzPin, HIGH);
    delay(20);
    digitalWrite(buzzPin, LOW);

      if ( abs(weight) > (setValue-3) )
      {
      digitalWrite(buzzPin, HIGH);
      delay(200);
      digitalWrite(buzzPin, LOW);
        if ( abs(weight) > (setValue-1) )
        {
        digitalWrite(buzzPin, HIGH);
        delay(400);
        digitalWrite(buzzPin, LOW);
          if ( abs(weight) > (setValue-1) )
          {
          digitalWrite(buzzPin, HIGH);
          delay(100);
          digitalWrite(buzzPin, LOW);
          }
        }
      }  
  }
    

//-----Button-----
  but1State=digitalRead(but1Pin);
  but2State=digitalRead(but2Pin);
    
    //if button 1 is pressed, increase setValue by 1, LED1 lights up when but1 is pressed
    if(but1State != 0) 
    { digitalWrite(LED1, HIGH);  setValue++; }
    else digitalWrite(LED1, LOW);
    //if button 2 is pressed, decrease setValue by 1, LED2 lights up when but2 is pressed
    if(but2State != 0) 
    { digitalWrite(LED2, HIGH);  setValue--; }
    else digitalWrite(LED2, LOW);


//-----OLED
  OLED.clearDisplay();

    //display current reading
    OLED.setTextSize(1);
    OLED.setTextColor(WHITE);
      OLED.setCursor(0,0);
      OLED.print("Scale: ");
      OLED.print(weight);
      OLED.print(" [Unit]");
  
    //display setValue
    OLED.setTextSize(1);
    OLED.setTextColor(WHITE);
      OLED.setCursor(0,10);
      OLED.print("Set Value: ");
      OLED.print(setValue);
      OLED.print(" [Unit]");
  
  OLED.display();
  delay(300);
  OLED.clearDisplay();



//-----test----tmp


}
