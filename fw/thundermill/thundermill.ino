#define DEBUG // Please comment it if you are not debugging
String githash = "51832f3";
String FWversion = "TM";
#define ZERO 256  // ADC DC offset
#define RANGE 1000   // histogram range
#define EVENTS 500 // maximal number of recorded events

// Compiled with: Arduino 1.8.13

/*
  AORDOS_C for GeoDos

ISP
---
PD0     RX
PD1     TX
RESET#  through 50M capacitor to RST#

ANALOG
------
+      A0  PA0
-      A1  PA1
RESET  0   PB0

LED
---
LED1  13  PD5         
LED2  14  PD6         
LED3  15  PD7         



                     Mighty 1284p    
                      +---\/---+
           (D 0) PB0 1|        |40 PA0 (AI 0 / D24)
           (D 1) PB1 2|        |39 PA1 (AI 1 / D25)
      INT2 (D 2) PB2 3|        |38 PA2 (AI 2 / D26)
       PWM (D 3) PB3 4|        |37 PA3 (AI 3 / D27)
    PWM/SS (D 4) PB4 5|        |36 PA4 (AI 4 / D28)
      MOSI (D 5) PB5 6|        |35 PA5 (AI 5 / D29)
  PWM/MISO (D 6) PB6 7|        |34 PA6 (AI 6 / D30)
   PWM/SCK (D 7) PB7 8|        |33 PA7 (AI 7 / D31)
                 RST 9|        |32 AREF
                VCC 10|        |31 GND
                GND 11|        |30 AVCC
              XTAL2 12|        |29 PC7 (D 23)
              XTAL1 13|        |28 PC6 (D 22)
      RX0 (D 8) PD0 14|        |27 PC5 (D 21) TDI
      TX0 (D 9) PD1 15|        |26 PC4 (D 20) TDO
RX1/INT0 (D 10) PD2 16|        |25 PC3 (D 19) TMS
TX1/INT1 (D 11) PD3 17|        |24 PC2 (D 18) TCK
     PWM (D 12) PD4 18|        |23 PC1 (D 17) SDA
     PWM (D 13) PD5 19|        |22 PC0 (D 16) SCL
     PWM (D 14) PD6 20|        |21 PD7 (D 15) PWM
                      +--------+
*/

/*
// Compiled with: Arduino 1.8.9
// MightyCore 2.0.2 https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json
Fix old bug in Mighty SD library
~/.arduino15/packages/MightyCore/hardware/avr/2.0.2/libraries/SD/src/SD.cpp:
boolean SDClass::begin(uint32_t clock, uint8_t csPin) {
  if(root.isOpen()) root.close();
*/

#include <SD.h>             // Revised version from MightyCore
#include "wiring_private.h"
#include <Wire.h>           
#include <avr/wdt.h>

#define LED1  13  //PD5         
#define LED2  14  //PD6         
#define LED3  15  //PD7         
#define COUNT1_PCFO  0  //PB0       
#define COUNT2  20  //PC4         

uint16_t count = 0;
uint32_t serialhash = 0;
//uint16_t offset, base_offset;
uint8_t lo, hi;
uint16_t u_sensor, maximum;

// Read Analog Differential without gain (read datashet of ATMega1280 and ATMega2560 for refference)
// Use analogReadDiff(NUM)
//   NUM  | POS PIN             | NEG PIN           |   GAIN
//  0 | A0      | A1      | 1x
//  1 | A1      | A1      | 1x
//  2 | A2      | A1      | 1x
//  3 | A3      | A1      | 1x
//  4 | A4      | A1      | 1x
//  5 | A5      | A1      | 1x
//  6 | A6      | A1      | 1x
//  7 | A7      | A1      | 1x
//  8 | A8      | A9      | 1x
//  9 | A9      | A9      | 1x
//  10  | A10     | A9      | 1x
//  11  | A11     | A9      | 1x
//  12  | A12     | A9      | 1x
//  13  | A13     | A9      | 1x
//  14  | A14     | A9      | 1x
//  15  | A15     | A9      | 1x
#define PIN 0
uint8_t analog_reference = INTERNAL2V56; // DEFAULT, INTERNAL, INTERNAL1V1, INTERNAL2V56, or EXTERNAL

uint8_t bcdToDec(uint8_t b)
{
  return ( ((b >> 4)*10) + (b%16) );
}

void waitForSerial()
{
  while(Serial.available()) Serial.read();
  while (!Serial.available());
  while(Serial.available()) Serial.read();
}
  
void setup()
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(COUNT1_PCFO, INPUT);
  pinMode(COUNT2, INPUT);

  for(int i=0; i<3; i++)  
  {
    digitalWrite(LED1, LOW);  
    delay(100);
    digitalWrite(LED1, HIGH);  
    delay(100);
  }
  
  Wire.setClock(100000);

  // Open serial communications
  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println("#Cvak...");

  for(int i=0; i<3; i++)  
  {
    digitalWrite(LED2, LOW);  
    delay(100);
    digitalWrite(LED2, HIGH);  
    delay(100);
  }

  ADMUX = (analog_reference << 6) | ((PIN | 0x10) & 0x1F);  
  //ADCSRB = 0;               // Switching ADC to Free Running mode
  //sbi(ADCSRA, ADATE);       // ADC autotrigger enable (mandatory for free running mode)
  //sbi(ADCSRA, ADSC);        // ADC start the first conversions
  sbi(ADCSRA, 2);           // 0x100 = clock divided by 16, 1 MHz, 13 us for 13 cycles of one AD conversion, 24 us fo 1.5 cycle for sample-hold
  cbi(ADCSRA, 1);        
  cbi(ADCSRA, 0);        

  
  Serial.println("#Hmmm...");

  // make a string for device identification output
  String dataString = "$AIRDOS," + FWversion + "," + githash + ","; // FW version and Git hash

  if (digitalRead(17)) // Protection against sensor mallfunction 
  {
    Wire.beginTransmission(0x58);                   // request SN from EEPROM
    Wire.write((int)0x08); // MSB
    Wire.write((int)0x00); // LSB
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)0x58, (uint8_t)16);    
    for (int8_t reg=0; reg<16; reg++)
    { 
      uint8_t serialbyte = Wire.read(); // receive a byte
      if (serialbyte<0x10) dataString += "0";
      dataString += String(serialbyte,HEX);    
      serialhash += serialbyte;
    }
  }
  else
  {
    dataString += "NaN";    
  }


  for(int i=0; i<3; i++)  
  {
    digitalWrite(LED3, LOW);  
    delay(100);
    digitalWrite(LED3, HIGH);  
    delay(100);
  }
}

#define DECIMATION 1

void loop()
{
  int16_t buffer[RANGE];       // buffer for histogram

  int n=0;
  
  while (true)
  {
    int16_t sensor;

    if (digitalRead(COUNT1_PCFO)) {digitalWrite(LED1, HIGH);} else {digitalWrite(LED1, LOW);};
    if (digitalRead(COUNT2))
    {
 
      digitalWrite(LED2, !digitalRead(LED2));
      while (digitalRead(COUNT1_PCFO)); // wait for RPM hole

      sbi(ADCSRA, ADSC);        // ADC start conversions
      while (bit_is_clear(ADCSRA, ADIF)); // wait for end of conversion 
      lo = ADCL;
      hi = ADCH;
      sbi(ADCSRA, ADIF);            // reset interrupt flag from ADC
      sensor = (hi << 8) | lo;      // combine the two bytes
      if (bitRead(sensor,9)) sensor = -1*(1024-sensor);
      //buffer[n++] = sensor;
      Serial.println(sensor);
    }
  };

  //for(uint16_t i=0; i<(GPSdelay); i++)  // measurements between GPS aquisition
  {
    for(int n=0; n<RANGE; n++) // clear histogram
    {
      buffer[n]=0;
    }

    uint16_t hit_count = 0;

    // dosimeter integration
    for (uint32_t i=0; i<RANGE; i++)    
    {
      int16_t sensor;
      
      wdt_reset(); //Reset WDT

      while (bit_is_clear(ADCSRA, ADIF)); // wait for end of conversion 
      delayMicroseconds(8); // wait for 1.5 cycle of ADC clock for sample/hold for next conversion
      DDRB = 0b10011111;                  // Reset peak detector
      delayMicroseconds(2);              
      DDRB = 0b10011110;
  
      // we have to read ADCL first; doing so locks both ADCL
      // and ADCH until ADCH is read.  reading ADCL second would
      // cause the results of each conversion to be discarded,
      // as ADCL and ADCH would be locked when it completed.
      lo = ADCL;
      hi = ADCH;
      sbi(ADCSRA, ADIF);                  // reset interrupt flag from ADC
  
      // combine the two bytes
      //u_sensor = (hi << 7) | (lo >> 1);
      sensor = (hi << 8) | lo;
      if (bitRead(sensor,9)) sensor = -1*(1024-sensor);
      // manage negative values
      //if (u_sensor <= (CHANNELS/2)-1 ) {u_sensor += (CHANNELS/2);} else {u_sensor -= (CHANNELS/2);}

      //if (u_sensor < ZERO) {u_sensor = 0;} else {u_sensor -= ZERO;}
      
      buffer[i] = u_sensor;
      Serial.println(sensor); 
      delay(100);
    
    }  
    
    // Data out
    // Histogram out
    if (false)
    {
      int16_t max=0;
      int16_t min=1023;

      for(int n=0; n<RANGE; n++)  
      {
        if (buffer[n]>max) max=buffer[n];
        if (buffer[n]<min) min=buffer[n];
      }
       
      digitalWrite(LED1, HIGH);  // Blink for Dasa
      Serial.print(min); 
      Serial.print(","); 
      Serial.println(max); 
      digitalWrite(LED1, LOW);                
    }    
  }
}
