
/* Cor      22-4-2017 - Uitgangspunt voor de TTN Enschede bouwavond
/* Jeroen / 5-12-2016 - kale Ideetron versie genomen met kleine aanpassingen voor TTN Apeldoorn bouwavond 
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
****************************************************************************************
* Modifications by Lex PH2LB : 
* 
* Added BMP280 sensor  
* payload function : 
*  
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

 */
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
#include <Adafruit_BMP280.h>

#include <Wire.h>

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
 

Adafruit_BMP280 bme; // I2C

int FC = 0;
// SF to set in LoRaWAN)V31.h (default SF9 is set) - quick and dirty implemented (TBC)

void setup() 
{
  //Initialize the UART
  Serial.begin(115200);
  Serial.println("---");
  Serial.println("What: TTN Enschede - LoRa node bouwavond BMP280 versie");
  Serial.println("Setup: Initialized serial port");

  
  if (!bme.begin(0x76)) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

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

  //Initialize RFM module
  Serial.println("Loop: Initializing RFM95");
  RFM_Init();
  delay(1000);

    // Here the sensor information should be retrieved 
  float t_float = bme.readTemperature();
  float p_float = bme.readPressure() / 100.0; // pa = mBar
  float h_float = 0.0;  // BMP280 doesn t have humidity
  Serial.print("Temperature = ");
  Serial.print(t_float);
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  Serial.print(p_float);
  Serial.println(" mBar");
 
  int h = (int)h_float;
  int p = (int)p_float;
  int t = (int)((t_float + 40.0) * 10.0);
  // t = t + 40; // t [-40..+80] => [0..120]
  // t = t * 10; // t [0..120] => [0..1200] 
    
  Data_Tx[0] = t >> 8;
  Data_Tx[1] = t & 0xFF;
  Data_Tx[2] = p >> 8;
  Data_Tx[3] = p & 0xFF;
  Data_Tx[4] = h & 0xFF;
  Data_Length_Tx = 5; 
 

  Serial.println("Loop: Sending data");
  Data_Length_Rx = LORA_Cycle(Data_Tx, Data_Rx, Data_Length_Tx);
  FC = FC + 1;
  
  //Delay of 1 minute 
  Serial.println("Loop: Start waiting loop (1 minutes)");
  delay(60000);
  Serial.println("---");
}
