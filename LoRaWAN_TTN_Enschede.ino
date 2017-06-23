/*
* Lex      22-6-2017 - Added BME280 support
* Cor      07-6-2017 - Cayenne LPP datastructuur
* Cor      22-4-2017 - Uitgangspunt voor de TTN Enschede bouwavond
* Jeroen / 5-12-2016 - kale Ideetron versie genomen met kleine aanpassingen voor TTN Apeldoorn bouwavond
/******************************************************************************************
* Copyright 2015, 2016 Ideetron B.V.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************************/
/****************************************************************************************
* File:     LoRaWAN_V31.ino
* Author:   Gerben den Hartog
* Compagny: Ideetron B.V.
* Website:  http://www.ideetron.nl/LoRa
* E-mail:   info@ideetron.nl
****************************************************************************************/
/****************************************************************************************
* Created on:         04-02-2016
* Supported Hardware: ID150119-02 Nexus board with RFM95
* 
* Description
* 
* Minimal Uplink for LoRaWAN
* 
* This code demonstrates a LoRaWAN connection on a Nexus board. This code sends a messege every minute
* on chanell 0 (868.1 MHz) Spreading factor 7.
* On every message the frame counter is raised
* 
* This code does not include
* Receiving packets and handeling
* Channel switching
* MAC control messages
* Over the Air joining* 
*
* Firmware version: 1.0
* First version
* 
* Firmware version 2.0
* Working with own AES routine
* 
* Firmware version 3.0
* Listening to receive slot 2 SF9 125 KHz Bw
* Created seperate file for LoRaWAN functions
* 
* Firmware version 3.1
* Removed a bug from the Waitloop file
* Now using AppSkey in Encrypt payload function (Thanks to Willem4ever)
* Switching to standby at start of send package function
*
* Firmware version 3.2
* Merged I2C_ReadAllData.ino from 
* https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
* for interfacing with the BMP280
* 
* Firmware version 3.3
* Use Cayenne Low Power Payload datastructures
*   in order to use myDevices.com account
*   
*
* Firmware version 3.3b
*   Added BME280 support
****************************************************************************************/

/*
*****************************************************************************************
* INCLUDE FILES
*****************************************************************************************
*/
#include <SPI.h>
#include "AES-128_V10.h"
#include "Encrypt_V31.h"
#include "LoRaWAN_V31.h"
#include "RFM95_V21.h"
#include "LoRaMAC_V11.h"
#include "Waitloop_V11.h"

// added for the bme280 support
#include <Wire.h>
#include <Adafruit_Sensor.h>

#define USE_BME280

#ifdef USE_BME280
#include <Adafruit_BME280.h>
Adafruit_BME280 bmx; // I2C
#define BMx280_ADDRESS 0x76
#else
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmx; // I2C
#define BMx280_ADDRESS 0x76
#endif

#define USE_CAYENNE 1

#ifdef USE_CAYENNE
#include "CayenneLPP.h"
CayenneLPP lpp(51);                    // create a buffer of 51 bytes to store the payload
#endif


/* TTN payload decoder : 

function Decoder(bytes, port) 
{ 
  if (bytes.length >= 2)
  {
    var temperature = (((bytes[0] << 8) | bytes[1]) / 10.0) - 40.0;
  } 
  if (bytes.length >= 4)
  { 
    var pressure = ((bytes[2] << 8) | bytes[3]);
  }
  if (bytes.length >= 5)
  { 
    var humidity = bytes[4];
  }
  // Decoder  
  return { 
    temperature: temperature,
    pressure: pressure,
    humidity: humidity,
    bytes: bytes
  };
}
*/
 
/*
*****************************************************************************************
* GLOBAL VARIABLES
*****************************************************************************************
*/ 
// Registreren en eigen keys invullen
// Address en sleutels voor console.thethingsnetwork.org
unsigned char NwkSkey[16] = { 0x37, 0xAD, 0x2F, 0x38, 0x20, 0x94, 0xB1, 0x4D, 0xCE, 0xA5, 0x29, 0x30, 0x6E, 0x2F, 0x52, 0x71 };
unsigned char AppSkey[16] = { 0x44, 0xDD, 0x02, 0x2F, 0x4B, 0x16, 0x5C, 0x1E, 0x41, 0xFD, 0x20, 0x99, 0x4D, 0x2A, 0xFB, 0xAD };
unsigned char DevAddr[4] = { 0x26, 0x01, 0x11, 0xBC };
 
//WOUTER unsigned char NwkSkey[16] = { 0x7D, 0x0C, 0x5F, 0xEF, 0xDF, 0x90, 0x87, 0xC8, 0x84, 0x0D, 0xB5, 0xDB, 0xF8, 0xC5, 0xE2, 0xC7 };
//unsigned char AppSkey[16] = { 0xA3, 0x6F, 0x08, 0xE6, 0x38, 0x20, 0xB7, 0x20, 0xCE, 0x6F, 0xE4, 0xA4, 0xB8, 0x64, 0x7A, 0x14 };
//unsigned char DevAddr[4] = { 0x26, 0x01, 0x12, 0xA2 }; 

int FC = 0;
// SF to set in LoRaWAN)V31.h (default SF9 is set) - quick and dirty implemented (TBC)

void setup() 
{
  //Initialize the UART
  Serial.begin(115200);
  Serial.println("---");
  Serial.println("What: TTN Enschede - LoRa node bouwavond");

    
  if (!bmx.begin(BMx280_ADDRESS)) {
    Serial.println(F("Could not find a valid bme280 sensor, check wiring!"));
    while (1);
  }


  Serial.print(F("Temperature = "));
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0;
  Serial.print(temp);
  Serial.println(" *C");
  Serial.print(F("Pressure = "));
  Serial.print(pressure);
  Serial.println(" millibar"); 
  
  Serial.println("Setup: Initialized serial port");

   //Initialise the SPI port
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
  Serial.println("Setup: SPI initialized");
  
  //Initialize I/O pins
  pinMode(DIO0,INPUT);
  pinMode(DIO1,INPUT); 
  pinMode(DIO5,INPUT);
  pinMode(DIO2,INPUT);
  pinMode(CS,OUTPUT);
  
  digitalWrite(CS,HIGH);

  WaitLoop_Init();

  //Wait until RFM module is started
  WaitLoop(20);   
  Serial.println("Setup: Completed");
  Serial.println("---");
}

void loop() 
{
  unsigned char Test = 0x00;

  unsigned char Sleep_Sec = 0x00;
  unsigned char Sleep_Time = 0x01;

  unsigned char Data_Tx[256];
  unsigned char Data_Rx[64];
  unsigned char Data_Length_Tx;
  unsigned char Data_Length_Rx = 0x00;
  char msg[64];

  //Initialize RFM module
  Serial.println("Loop: Initializing RFM95");
  RFM_Init();
  delay(1000);

  // Here the sensor information should be retrieved 
  //  Pressure: 300...1100 hPa
  //  Temperature: -40…85°C

  float temp = bmx.readTemperature();
  float pressure = bmx.readPressure() / 100.0;    // from pascal to hPa (milliBar)
  float humidity = 0.0;

#ifdef USE_BME280
   humidity =   bmx.readHumidity();
#endif

  
  Serial.print(F("Temperature = "));
  Serial.print(temp);
  Serial.println(" *C");
  
  Serial.print(F("Pressure = "));
  Serial.print(pressure);
  Serial.println(" millibar"); 
  
  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");


  Serial.println("Loop: Sending data");

#ifdef USE_CAYENNE
    lpp.reset();                            // clear the buffer
    lpp.addTemperature(1, temp);            // on channel 1, add temperature, value 22.5°C
    lpp.addBarometricPressure(2, pressure); // channel 2, pressure
    lpp.addRelativeHumidity(3, humidity);            // channel 3, pressure
    Data_Length_Rx = LORA_Cycle(lpp.getBuffer(), Data_Rx, lpp.getSize());
#else
  int t = (int)((temp + 40.0) * 10.0);
  // t = t + 40; => t [-40..+85] => [0..125] => t = t * 10; => t [0..125] => [0..1250]
  int p = (int)(pressure);  // p [300..1100]
  int h = (int)(humidity);

  Data_Tx[0] = t >> 8;
  Data_Tx[1] = t & 0xFF;
  Data_Tx[2] = p >> 8;
  Data_Tx[3] = p & 0xFF;
  Data_Tx[4] = h & 0xFF;
  Data_Length_Tx = 5; // just 5 bytes

  Data_Length_Rx = LORA_Cycle(Data_Tx, Data_Rx, Data_Length_Tx);
#endif


  FC = FC + 1;
  
  //Delay of 1 minute 
  Serial.println("Loop: Start waiting loop (1 minutes)");
  delay(60000);
  Serial.println("---");
}
