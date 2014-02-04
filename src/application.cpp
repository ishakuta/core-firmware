/**
 ******************************************************************************
 * @file    application.cpp
 * @authors  Satish Nair, Zachary Crockett and Mohit Bhoite
 * @version V1.0.0
 * @date    05-November-2013
 * @brief   Tinker application
 * @hacked  DallasTemperature & OpenWire added 
 *          https://github.com/justinatomatic/DallasTemperature_SparkCore
 ******************************************************************************
  Copyright (c) 2013 Spark Labs, Inc.  All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/  
#include "application.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port D3 on SparkCore
#define ONE_WIRE_BUS D3
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature $
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress dsAddress;

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void initDsSensor()
{
        // start serial port
        Serial.begin(9600);
        Serial.println("Dallas Temperature IC Control Library Demo");

        // locate devices on the bus
        Serial.print("Locating devices...");
        sensors.begin();
        Serial.print("Found ");
        Serial.print(sensors.getDeviceCount(), DEC);
        Serial.println(" devices.");

        // report parasite power requirements
        Serial.print("Parasite power is: "); 
        if (sensors.isParasitePowerMode()) Serial.println("ON");
        else Serial.println("OFF");
  
        // assign address manually.  the addresses below will beed to be changed
        // to valid device addresses on your bus.  device address can be retrieved
        // by using either oneWire.search(deviceAddress) or individually via
        // sensors.getAddress(deviceAddress, index)
        //dsAddress = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

        if (!sensors.getAddress(dsAddress, 0)) 
                Serial.println("Unable to find address for Device 0");

        // show the addresses we found on the bus
        Serial.print("Device 0 Address: ");
        printAddress(dsAddress);
        Serial.println();

        // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
        sensors.setResolution(dsAddress, 9);
 
        Serial.print("Device 0 Resolution: ");
        Serial.print(sensors.getResolution(dsAddress), DEC); 
        Serial.println();
}

/* Function prototypes -------------------------------------------------------*/
int tinkerDigitalRead(String pin);
int tinkerDigitalWrite(String command);
int tinkerAnalogRead(String pin);
int tinkerAnalogWrite(String command);
int tinkerTempRead(String command);

int temp = 0;

/* This function is called once at start up ----------------------------------*/
void setup()
{
	Spark.variable("temp", &temp, INT);

	//Register all the Tinker functions
	Spark.function("digitalread", tinkerDigitalRead);
	Spark.function("digitalwrite", tinkerDigitalWrite);
	Spark.function("analogread", tinkerAnalogRead);
	Spark.function("analogwrite", tinkerAnalogWrite);

	Spark.function("tempread", tinkerTempRead);
	
	initDsSensor();
}

/* This function loops forever --------------------------------------------*/
void loop()
{
	//This will run in a loop
}

/*******************************************************************************
 * Function Name  : tempRead
 * Description    : Read temperature from DS18x20 sensors
 * Input          : Command
 * Output         : None
 * Return         : Temp in Celcius * 100
 *******************************************************************************/
int tinkerTempRead(String command)
{
        // issue a global temperature request to all devices on the bus
        sensors.requestTemperatures();

	//Serial.println(DallasTemperature::toFahrenheit(tempC));
	temp = sensors.getTempC(dsAddress) * 100;
        return temp;
}

/*******************************************************************************
 * Function Name  : tinkerDigitalRead
 * Description    : Reads the digital value of a given pin
 * Input          : Pin 
 * Output         : None.
 * Return         : Value of the pin (0 or 1) in INT type
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerDigitalRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		pinMode(pinNumber, INPUT_PULLDOWN);
		return digitalRead(pinNumber);
	}
	else if (pin.startsWith("A"))
	{
		pinMode(pinNumber+10, INPUT_PULLDOWN);
		return digitalRead(pinNumber+10);
	}
	return -2;
}

/*******************************************************************************
 * Function Name  : tinkerDigitalWrite
 * Description    : Sets the specified pin HIGH or LOW
 * Input          : Pin and value
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerDigitalWrite(String command)
{
	bool value = 0;
	//convert ascii to integer
	int pinNumber = command.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(command.substring(3,7) == "HIGH") value = 1;
	else if(command.substring(3,6) == "LOW") value = 0;
	else return -2;

	if(command.startsWith("D"))
	{
		pinMode(pinNumber, OUTPUT);
		digitalWrite(pinNumber, value);
		return 1;
	}
	else if(command.startsWith("A"))
	{
		pinMode(pinNumber+10, OUTPUT);
		digitalWrite(pinNumber+10, value);
		return 1;
	}
	else return -3;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogRead
 * Description    : Reads the analog value of a pin
 * Input          : Pin 
 * Output         : None.
 * Return         : Returns the analog value in INT type (0 to 4095)
                    Returns a negative number on failure
 *******************************************************************************/
int tinkerAnalogRead(String pin)
{
	//convert ascii to integer
	int pinNumber = pin.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	if(pin.startsWith("D"))
	{
		pinMode(pinNumber, INPUT);
		return analogRead(pinNumber);
	}
	else if (pin.startsWith("A"))
	{
		pinMode(pinNumber+10, INPUT);
		return analogRead(pinNumber+10);
	}
	return -2;
}

/*******************************************************************************
 * Function Name  : tinkerAnalogWrite
 * Description    : Writes an analog value (PWM) to the specified pin
 * Input          : Pin and Value (0 to 255)
 * Output         : None.
 * Return         : 1 on success and a negative number on failure
 *******************************************************************************/
int tinkerAnalogWrite(String command)
{
	//convert ascii to integer
	int pinNumber = command.charAt(1) - '0';
	//Sanity check to see if the pin numbers are within limits
	if (pinNumber< 0 || pinNumber >7) return -1;

	String value = command.substring(3);

	if(command.startsWith("D"))
	{
		pinMode(pinNumber, OUTPUT);
		analogWrite(pinNumber, value.toInt());
		return 1;
	}
	else if(command.startsWith("A"))
	{
		pinMode(pinNumber+10, OUTPUT);
		analogWrite(pinNumber+10, value.toInt());
		return 1;
	}
	else return -2;
}
