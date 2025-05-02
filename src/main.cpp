#include <Arduino.h>
#include "wiring.h"
#include <string>
#include <vector>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

#define SEALEVELPRESSURE_HPA (1013.25) // Dhaka's approximate sea-level pressure in hPa

Adafruit_BMP3XX bmp;

using namespace std;
#define xbeeSerial Serial8

// structs

struct TelemetryData
{
  String TEAM_ID;
  String MISSION_TIME;
  int PACKET_COUNT;
  String MODE;
  int STATE;
  float ALTITUDE;
  float TEMPERATURE;
  float PRESSURE;
  float VOLTAGE;
  float GYRO_R;
  float GYRO_P;
  float GYRO_Y;
  float ACCEL_R;
  float ACCEL_P;
  float ACCEL_Y;
  float MAG_R;
  float MAG_P;
  float MAG_Y;
  float AUTO_GYRO_ROTATION_RATE;
  String GPS_TIME;
  String GPS_ALTITUDE;
  String GPS_LATITUDE;
  String GPS_LONGITUDE;
  String GPS_SATS;
  String CMD_ECHO;
  vector<String> OPTIONAL_DATA;
};

// function prototypes

void setup();
void loop();
void publishTelemetryToXbee();
void checkXbeeHWAvailability();
void showXBEEWarning();
void initializeTelemetryData(TelemetryData &data);
String constructMessage();
String getStateName(int state);
void processReceivedXBeeData();
void handleGCSCommand(String cmd);
void confirmGroundStationConn();
void initBMP390L();

// flag variables
bool commEstablished = false;
TelemetryData telemetryData = {};
bool telemetryEnabled = false;

// Arduino setup and loop functions

void setup()
{
  Serial.begin(9600);  // Serial Monitor default baud rate
  Serial8.begin(9600); // XBee 900HP Pro default baud rate

  // initiate communication stack
  checkXbeeHWAvailability();
  confirmGroundStationConn();
  
  // initialize sensors
  initBMP390L();

  // initialize telemetry data
  initializeTelemetryData(telemetryData);
}

void initBMP390L()
{
  Wire1.begin();

  if (!bmp.begin_I2C(0x77, &Wire1))
  {
    Serial.println("Could not find a valid BMP390 sensor on Wire1, check wiring!");
    while (1)
      ;
  }

  // Optional: setup oversampling and filter
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
}

void loop()
{
  processReceivedXBeeData();

  // Read sensor data

  processSensorBMP();

  if (commEstablished && telemetryEnabled)
  {
    publishTelemetryToXbee();
    delay(1000); // Delay to avoid flooding the XBee with messages
  }
}

void processSensorBMP()
{
  if (bmp.performReading())
  {
    telemetryData.TEMPERATURE = bmp.temperature;
    telemetryData.PRESSURE = bmp.pressure / 100.0; // Convert to hPa
    telemetryData.ALTITUDE = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  else
  {
    Serial.println("Failed to read from BMP390 sensor!");
  }
}

// Custom functions

void processReceivedXBeeData()
{
  static String receivedData = ""; // Define a single buffer for received data

  while (xbeeSerial.available())
  {
    char character = xbeeSerial.read();
    receivedData += character;

    // Check if a complete payload is received
    if (receivedData.indexOf('<') != -1 && receivedData.indexOf('>') != -1)
    {
      int startIdx = receivedData.indexOf('<') + 1; // Get the index of '<'
      int endIdx = receivedData.indexOf('>');       // Get the index of '>'

      // Extract the data between '<' and '>'
      String extractedData = receivedData.substring(startIdx, endIdx);

      // Check the command type
      if (extractedData.charAt(0) == 'C')
      {
        handleGCSCommand(extractedData.substring(1));
      }
      else if (extractedData.charAt(0) == 'S')
      {
        // handleServoCommand(extractedData.substring(1));
      }
      else
      {
        Serial.println("Unknown command received: " + extractedData);
      }

      receivedData = ""; // Reset receivedData for the next payload
    }
  }
}

void handleGCSCommand(String cmd)
{
  int slashIndex = cmd.indexOf('/');
  String beforeSlash = cmd.substring(0, slashIndex);
  String afterSlash = cmd.substring(slashIndex + 1);
  telemetryData.CMD_ECHO = cmd;

  Serial.println("Received MQTT command from XBee: " + cmd);

  // if (beforeSlash.equals("PRESSURE"))
  // {
  //   float sim_pressure = afterSlash.toFloat();
  //   readSensorData(sim_pressure / 100);
  //   simulationFlightStatesLogic();
  //   publishSensorDataToXbee();
  //   Serial.println("Pressure value: " + String(afterSlash));
  // }
  // else if (beforeSlash.equals("UTC"))
  // {
  //   Serial.print(afterSlash);
  //   if (afterSlash.equals("GPS"))
  //   {
  //   }
  //   else
  //   {
  //     int firstColon = afterSlash.indexOf(':');
  //     int secondColon = afterSlash.indexOf(':', firstColon + 1);

  //     int hours = afterSlash.substring(0, firstColon).toInt();
  //     int minutes = afterSlash.substring(firstColon + 1, secondColon).toInt();
  //     int seconds = afterSlash.substring(secondColon + 1).toInt();

  //     setTime(hours, minutes, seconds, day(), month(), year());
  //     Teensy3Clock.set(now()); // set the RTC

  //     Serial.println("RTC has been set!");
  //   }
  //} else
  if (cmd.equals("CX/ON"))
  {
    telemetryEnabled = true;
  }
  else if (cmd.equals("CX/OFF"))
  {
    telemetryEnabled = false;
  }
  // else if (cmd.equals("CAL"))
  // {
  //   calibrateAltitude(3);
  //   String msg = "<RCAL, SUCCESS, Calibrated altitude = " + String(initial_altitude) + ">";
  //   xbeeSerial.print(msg);
  // }
  // else if (cmd.equals("BCN/ON"))
  // {
  //   turnOnBuzzer();
  //   xbeeSerial.print("<RBCN/ON, SUCCESS, Buzzer turned on successfully>");
  // }
  // else if (cmd.equals("BCN/OFF"))
  // {
  //   turnOffBuzzer();
  //   xbeeSerial.print("<RBCN/OFF, SUCCESS, Buzzer turned off successfully>");
  // }
  // else if (cmd.equals("SIM/ACTIVATE"))
  // {
  //   if (simulation_enable)
  //   {
  //     telemetry_is_on = false;
  //     simulation_activate = true;
  //     telemetryData.mode = 'S';
  //     xbeeSerial.print("<RSIM/ACTIVATE, SUCCESS, Simulation Activated>");
  //     Serial.println("simulation_activate");
  //   }
  //   else
  //   {
  //     xbeeSerial.print("<RSIM/ACTIVATE, FAILED, Simulation is not enabled>");
  //     Serial.println("simulation_activate failed because not enabled");
  //   }
  // }
  // else if (cmd.equals("SIM/ENABLE"))
  // {
  //   simulation_enable = true;
  //   xbeeSerial.print("<RSIM/ENABLE, SUCCESS, Simulation enabled>");
  //   Serial.println("simulation_enable");
  // }
  // else if (cmd.equals("SIM/DISABLE"))
  // {
  //   simulation_enable = false;
  //   xbeeSerial.print("<RSIM/DISABLE, SUCCESS, Simulation disabled>");
  //   Serial.println("simulation_disabled");
  // }
  // else if (cmd.equals("RESET/ALL"))
  // {
  //   resetMissionTime();
  //   resetPacketCount();
  //   initializeTelemetryData(telemetryData);
  //   xbeeSerial.print("<RRESET/ALL, SUCCESS, Reset succcessful>");
  // }
  // else if (cmd.equals("HS/ON"))
  // {
  //   moveServo(servo1, 90, 0);
  //   delay(200);
  //   moveServo(servo1, 0, 90);
  //   xbeeSerial.print("<RHS/ON, SUCCESS, Heatshield activated>");
  //   Serial.println("Heatshield activated");
  // }
  // else if (cmd.equals("HS/OFF"))
  // {
  //   moveServo(servo1, 90, 0);
  //   delay(200);
  //   moveServo(servo1, 0, 90);
  //   xbeeSerial.print("<RHS/OFF, SUCCESS, Heatshield deactivated>");
  //   Serial.println("Heatshield deactivated");
  // }
  // else if (cmd.equals("PC/ON"))
  // {
  //   moveServo(servo2, 90, 0);
  //   xbeeSerial.print("<RPC/ON, SUCCESS, Parachute deployed>");
  //   Serial.println("Parachute deployed");
  // }
  // else if (cmd.equals("PC/OFF"))
  // {
  //   moveServo(servo2, 0, 90);
  //   xbeeSerial.print("<RPC/OFF, SUCCESS, Parachute retracted>");
  //   Serial.println("Parachute retracted");
  // }
}

void checkXbeeHWAvailability()
{
  while (!Serial8) // Check if Serial8 is available
  {
    showXBEEWarning();
    delay(3000); // Wait for a second
    Serial8.println("Reinitializing Serial8...");
    Serial8.begin(9600); // Try to reinitialize Serial8
  }
}

void confirmGroundStationConn()
{
  Serial.println("Teensy 4.1 Transmitter started.");
  Serial8.println("<A3158GCS>");
  Serial.print("Waiting for GCS acknowledgement...");
  while (Serial8.available() == 0)
  {
    Serial8.println("<A3158GCS>");
    Serial.print(".");
    delay(2000); // Add a small delay to avoid flooding the XBee with messages
  }
  Serial.println("Acknowledgement received.");
  String ack = Serial8.readStringUntil('\n');
  if (ack.trim() == "<A3158ACK>")
  {
    Serial.println("Ground Station is ready.");
    commEstablished = true;
  }
}

void showXBEEWarning()
{
  Serial.println("Failed to initialize Serial8!");
  Serial.println("Check your wiring and try again.");
  Serial.println("Make sure the XBee is connected to the correct pins.");
  Serial.println("TX8 = Pin 35, RX8 = Pin 34");
  Serial.println("Make sure the XBee is powered on.");
  Serial.println("Make sure the XBee is configured correctly.");
  Serial.println("Retrying to initialize Serial8...");
}

// put function definitions here:
void publishTelemetryToXbee()
{
  telemetryData.PACKET_COUNT++;
  // EEPROM.put(PACKET_COUNT_ADDRESS, telemetryData.packet_count);

  String dataToSend = "<T" + constructMessage() + ">";

  // Convert String to const char* for length calculation
  const char *dataCharArray = dataToSend.c_str();
  size_t dataLength = strlen(dataCharArray);

  // Send data to XBee and check if the entire message was sent
  size_t bytesWritten = Serial8.write(dataCharArray, dataLength);

  // Only print to Serial if the data was successfully sent to XBee
  if (bytesWritten == dataLength)
  {
    Serial.println("Sent data to XBee: " + dataToSend);
  }
  else
  {
    Serial.println("Failed to send data to XBee: " + dataToSend);
  }
}

void initializeTelemetryData(TelemetryData &data)
{
  data.TEAM_ID = "3158";
  data.MODE = 'F';
  data.MISSION_TIME = "0:0:0";
  data.GPS_TIME = "0:0";
  data.STATE = 0;
  
  if (data.PACKET_COUNT == 0xFFFFFFFF)
  {
    data.PACKET_COUNT = 0;
  }

}

String constructMessage()
{
  // Construct message using telemetryData struct
  // String message = String(telemetryData.TEAM_ID) + ", " + String(telemetryData.MISSION_TIME) + ", " + String(telemetryData.PACKET_COUNT) + ", " + String(telemetryData.M) + ", " + getStateName(telemetryData.state) + ", " + String(telemetryData.altitude) + ", " + String(telemetryData.air_speed) + ", " + String(telemetryData.hs_deployed) + ", " + String(telemetryData.pc_deployed) + ", " + String(telemetryData.temperature) + ", " + String(telemetryData.voltage, 2) + ", " + String(telemetryData.pressure) + ", " + String(telemetryData.gps_time) + ", " + String(telemetryData.gps_altitude, 7) + ", " + String(telemetryData.gps_latitude, 7) + ", " + String(telemetryData.gps_longitude, 7) + ", " + String(telemetryData.gps_sats) + ", " + String(telemetryData.accelerometer_x) + ", " + String(telemetryData.accelerometer_y) + ", " + String(telemetryData.gyroscope_z) + ", " + String(telemetryData.cmd_echo);
  String message = telemetryData.TEAM_ID + ", " +
                   telemetryData.MISSION_TIME + ", " +
                   telemetryData.PACKET_COUNT + ", " +
                   telemetryData.MODE + ", " +
                   getStateName(telemetryData.STATE) + ", " +
                   telemetryData.ALTITUDE + ", " +
                   telemetryData.TEMPERATURE + ", " +
                   telemetryData.PRESSURE + ", " +
                   telemetryData.VOLTAGE + ", " +
                   telemetryData.GYRO_R + ", " +
                   telemetryData.GYRO_P + ", " +
                   telemetryData.GYRO_Y + ", " +
                   telemetryData.ACCEL_R + ", " +
                   telemetryData.ACCEL_P + ", " +
                   telemetryData.ACCEL_Y + ", " +
                   telemetryData.MAG_R + ", " +
                   telemetryData.MAG_P + ", " +
                   telemetryData.MAG_Y + ", " +
                   telemetryData.AUTO_GYRO_ROTATION_RATE + ", " +
                   telemetryData.GPS_TIME + ", " +
                   telemetryData.GPS_ALTITUDE + ", " +
                   telemetryData.GPS_LATITUDE + ", " +
                   telemetryData.GPS_LONGITUDE + ", " +
                   telemetryData.GPS_SATS + ", " +
                   telemetryData.CMD_ECHO + ", ";
  return message;
}

String getStateName(int state)
{
  switch (state)
  {
  case 0:
    return "IDLE";
  case 1:
    return "Takeoff";
  case 2:
    return "Cruise";
  case 3:
    return "Landing";
  default:
    return "Unknown";
  }
}