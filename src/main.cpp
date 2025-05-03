#include <Arduino.h>
#include "wiring.h"
// Add TeensyThreads library for multithreading
#include <TeensyThreads.h>
// Core sensors and communication
#include <Wire.h>
#include <HardwareSerial.h>
// Sensor libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
// Standard libraries
#include <string>
#include <vector>
#include <atomic>

// Constants
#define SEALEVELPRESSURE_HPA (1013.25) // Dhaka's approximate sea-level pressure in hPa
#define BMP_UPDATE_INTERVAL 40         // 25Hz update rate (ms)
#define GPS_UPDATE_INTERVAL 100        // 10Hz update rate (ms)
#define TELEMETRY_INTERVAL 1000        // 1Hz telemetry transmission (ms)

// Serial port definitions
#define xbeeSerial Serial8
// Use Serial5 on Teensy 4.1 for GPS (updated from Serial2)
HardwareSerial &gpsSerial = Serial5;

// Using standard namespace
using namespace std;

// Global objects
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;

// Define the State enum
enum State
{
  RECOVER,
  BOOT,
  IDLE,
  PRE_LAUNCH,
  ASCENDING,
  DESCENDING_1,
  DESCENDING_2,
  LANDING
};

// Thread-safe state management
Threads::Mutex stateMutex;
State currentState = BOOT;
State lastState = BOOT;

// Thread IDs
int altitudeThreadID = -1;
int gpsThreadID = -1;
int telemetryThreadID = -1;
int stateMonitorThreadID = -1;
int commandThreadID = -1;

// Thread-safe data storage with atomic variables
struct ThreadSafeData
{
  // Use std::atomic for thread-safe variables
  std::atomic<float> altitude{0};
  std::atomic<float> initialAltitude{0};
  std::atomic<float> highestAltitude{0};
  std::atomic<float> velocity{0};
  std::atomic<float> temperature{0};
  std::atomic<float> pressure{0};
  std::atomic<float> voltage{3.3}; // Default voltage
  std::atomic<bool> altitudeValid{false};
  std::atomic<bool> sensorStable{false};
  std::atomic<int> packetCount{0};
  // GPS data (protected by mutex when accessed)
  float latitude = 0;
  float longitude = 0;
  int satellites = 0;
  String gpsTime = "0:0";
};

ThreadSafeData safeData;

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
float getSmoothedAltitude();
void calculateAltitude();
void calculateGPS();
float changeOfAltitude();
String getStateString(State state);
void processBatteryVoltage();

// Thread function prototypes
void altitudeThread();
void gpsThread();
void telemetryThread();
void stateMonitorThread();
void commandThread();

// Mutex for GPS data protection
Threads::Mutex gpsMutex;
// Mutex for XBee communication
Threads::Mutex xbeeMutex;

// flag variables
bool commEstablished = false;
TelemetryData telemetryData = {};
bool telemetryEnabled = false;

// Averaging variables for precise altitude readings
const int AVG_SAMPLES = 3; // Reduced from 5 to 3 for faster readings
float altitudeBuffer[AVG_SAMPLES];
int bufferIndex = 0;

float lastAltitude = 0;
unsigned long lastAltitudeTime = 0;

float altitude = 0;
float initialAltitude = 0;
float highestAltitude = 0;
float changeAltitude = 0;
int counter = 0;
bool cxonCommandReceived = false;
float landingThresholdAltitude = 5.0;
float velocity = -1;
bool altitudeCalibrated = false;

// Add threshold constants
const float ALTITUDE_THRESHOLD = 200.0;         // 2 meters in cm
const int STABLE_READINGS_REQUIRED = 3;         // Number of stable readings required
const float STABILITY_THRESHOLD = 50.0;         // 0.5 meters in cm
const float DESCENT_THRESHOLD = -200.0;         // -2 meters in cm
const float DESCENT_RATE_THRESHOLD = -500.0;    // -5 m/s in cm/s
const float DESCENT_2_THRESHOLD = 0.75;         // 75% of highest altitude for DESCENDING_2
const int DESCENT_CONFIRMATION_COUNT = 3;       // Number of consecutive descent readings required
const float MIN_VALID_ALTITUDE = -10000.0;      // -100 meters in cm
const float MAX_VALID_ALTITUDE = 1000000.0;     // 10000 meters in cm
const int INITIAL_READINGS = 20;                // Number of initial readings to validate sensor
const float STATIONARY_THRESHOLD = 50.0;        // Maximum variation for stationary readings
const float ASCENT_THRESHOLD = 100.0;           // Minimum altitude change to consider as ascending
const int ASCENT_CONFIRMATION_COUNT = 5;        // Number of consecutive ascent readings required
const float CONSECUTIVE_DIFF_THRESHOLD = 3.0;   // Threshold for consecutive altitude differences when stationary
const float MOVING_DIFF_THRESHOLD = 2.0;        // Threshold for consecutive altitude differences when moving
const int DESCENT_DETECTION_COUNT = 4;          // Number of consecutive readings below peak to confirm descent
const float DESCENT_DETECTION_THRESHOLD = 50.0; // Minimum drop from peak to consider descent (in cm)
const float MAX_ALTITUDE_CHANGE = 100.0;        // Maximum valid altitude change between readings (in cm)
const int MAX_FAILED_READINGS = 5;              // Maximum number of consecutive failed readings before resetting

// Add variables for sensor validation
float lastReadings[5] = {0}; // Store last 5 readings
int readingIndex = 0;
bool sensorStable = false;
bool sensorValid = false;
float initialReadings[INITIAL_READINGS];
int validReadingCount = 0;
bool isStationary = true;
int ascentCount = 0;
float baselineAltitude = 0;
float lastHighestAltitude = 0;
int consecutiveDiffCount = 0;
int movingDiffCount = 0;

// Add variables for descent detection
int descentCount = 0;
bool descentConfirmed = false;
float lastDescentCheck = 0;
unsigned long lastDescentTime = 0;
int consecutiveBelowPeakCount = 0; // Counter for consecutive readings below peak
float peakAltitude = 0;            // Track the absolute peak altitude
float lastValidAltitude = 0;       // Store the last valid altitude reading
bool altitudeValid = false;        // Flag to indicate if current altitude is valid
int failedReadingsCount = 0;       // Counter for consecutive failed readings
float fallbackAltitude = 0;        // Fallback altitude to use when sensor fails
const int analogPin = 15;      // Pin 15 connected to the voltage divider 'S'
const float referenceVoltage = 3.3; // Teensy 4.1 analog reference (default is 3.3V)
const float multiplier = 5.0; 

// Arduino setup and loop functions

void setup()
{
  // Initialize serial communication with higher baud rates for improved performance
  Serial.begin(115200); // Increased from 9600 to 115200 for faster logging
  Serial8.begin(9600);  // XBee 900HP Pro default baud rate (maintain compatibility)

  analogReadResolution(12);

  Serial.println("Cansat Flight Software v2.0 - Multithreaded Edition");
  Serial.println("Initializing hardware and communication...");

  // Initialize Wire1 for I2C sensors
  Wire1.begin();
  Wire1.setClock(400000); // Set to 400kHz for faster I2C communication

  // Initiate communication stack (blocking operation)
  checkXbeeHWAvailability();
  confirmGroundStationConn();

  // Initialize telemetry data
  initializeTelemetryData(telemetryData);

  // Start threads with appropriate priorities (higher number = higher priority)
  // Process altitude data with highest priority
  altitudeThreadID = threads.addThread(altitudeThread, 0, 1024, 0);

  // GPS has second highest priority
  gpsThreadID = threads.addThread(gpsThread, 0, 1024, 0);

  // Command processing must be responsive
  commandThreadID = threads.addThread(commandThread, 0, 1024, 0);

  // State monitoring runs at normal priority
  stateMonitorThreadID = threads.addThread(stateMonitorThread, 0, 1024, 0);

  // Telemetry runs at lowest priority as it's not time-critical
  telemetryThreadID = threads.addThread(telemetryThread, 0, 1024, 0);

  

  Serial.println("All threads started successfully");
  Serial.println("System running in multithreaded mode");
}

// Simplified loop - main thread mostly idle since work is done in separate threads
void loop()
{
  // Print status every 5 seconds
  static unsigned long lastStatusTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastStatusTime >= 5000)
  {
    // Report thread health
    Serial.println("===== System Status =====");
    Serial.print(threads.threadsInfo());
    Serial.println("=========================");

    // Report current state
    State currentStateSnapshot;
    stateMutex.lock();
    currentStateSnapshot = currentState;
    stateMutex.unlock();

    Serial.print("Current State: ");
    Serial.println(getStateString(currentStateSnapshot));

    // Report sensor status
    Serial.print("Altitude: ");
    Serial.print(safeData.altitude.load() / 100.0);
    Serial.println(" m");

    Serial.print("Highest: ");
    Serial.print(safeData.highestAltitude.load() / 100.0);
    Serial.println(" m");

    Serial.print("Velocity: ");
    Serial.print(safeData.velocity.load());
    Serial.println(" m/s");

    Serial.print("Temperature: ");
    Serial.print(safeData.temperature.load());
    Serial.println(" C");

    Serial.print("Telemetry Enabled: ");
    Serial.println(telemetryEnabled ? "Yes" : "No");

    Serial.println("=======================");

    lastStatusTime = currentTime;
  }

  // Give most CPU time to the threads
  threads.yield();
  delay(50);
}

// Custom functions

void processBatteryVoltage() {
  int rawADC = analogRead(analogPin);                      // Read raw ADC value
  float vOut = (rawADC / 4095.0) * referenceVoltage;       // Convert to voltage
  float vIn = vOut * multiplier;                           // Calculate original input voltage
  // Serial.print("Battery Voltage ");
  // Serial.print(vIn, 3);
  // Serial.println(" V");
  telemetryData.VOLTAGE = vIn;                             // Store voltage in telemetry data
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

  if (cmd.equals("CX/ON"))
  {
    telemetryEnabled = true;
  }
  else if (cmd.equals("CX/OFF"))
  {
    telemetryEnabled = false;
  }
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
    return "BOOT";
  case 1:
    return "IDLE";
  case 2:
    return "PRE_LAUNCH";
  case 3:
    return "ASCENDING";
  case 4:
    return "DESCENDING_1";
  case 5:
    return "DESCENDING_2";
  case 6:
    return "LANDING";
  default:
    return "Unknown";
  }
}

// Helper function to convert state enum to string
String getStateString(State state)
{
  switch (state)
  {
  case BOOT:
    return "BOOT";
  case IDLE:
    return "IDLE";
  case RECOVER:
    return "RECOVER";
  case PRE_LAUNCH:
    return "PRE_LAUNCH";
  case ASCENDING:
    return "ASCENDING";
  case DESCENDING_1:
    return "DESCENDING_1";
  case DESCENDING_2:
    return "DESCENDING_2";
  case LANDING:
    return "LANDING";
  default:
    return "UNKNOWN";
  }
}

// New function to get smoothed altitude using moving average
float getSmoothedAltitude()
{
  if (!bmp.performReading())
  {
    failedReadingsCount++;
    if (failedReadingsCount >= MAX_FAILED_READINGS)
    {
      return fallbackAltitude;
    }
    return 0;
  }

  failedReadingsCount = 0;
  float rawAltitudeMeters = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  float rawAltitudeCm = rawAltitudeMeters * 100.0;

  altitudeBuffer[bufferIndex] = rawAltitudeCm;
  bufferIndex = (bufferIndex + 1) % AVG_SAMPLES;

  float avg = 0;
  for (int i = 0; i < AVG_SAMPLES; i++)
  {
    avg += altitudeBuffer[i];
  }
  return avg / AVG_SAMPLES;
}

void calculateAltitude()
{
  float rawAltitude = getSmoothedAltitude();

  if (failedReadingsCount >= MAX_FAILED_READINGS)
  {
    altitudeValid = false;
    return;
  }

  if (rawAltitude / 100.0 < MIN_VALID_ALTITUDE / 100.0 || rawAltitude / 100.0 > MAX_VALID_ALTITUDE / 100.0)
  {
    altitudeValid = false;
    return;
  }

  if (altitudeValid && abs(rawAltitude - lastValidAltitude) > MAX_ALTITUDE_CHANGE)
  {
    altitudeValid = false;
    return;
  }

  altitudeValid = true;
  lastValidAltitude = rawAltitude;
  fallbackAltitude = rawAltitude;
  lastReadings[readingIndex] = rawAltitude;
  readingIndex = (readingIndex + 1) % 5;

  if (!altitudeCalibrated)
  {
    float sumAltitude = 0;
    int validReadings = 0;

    for (int i = 0; i < 20; i++)
    {
      if (bmp.performReading())
      {
        float reading = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 100.0;
        if (reading >= -1000.0 && reading <= 1000.0)
        {
          sumAltitude += reading;
          validReadings++;
        }
        delay(50); // Reduced from 100 to 50
      }
    }

    if (validReadings >= 10)
    {
      initialAltitude = sumAltitude / validReadings;
      baselineAltitude = initialAltitude;
      lastAltitude = initialAltitude;
      lastHighestAltitude = 0;
      altitudeCalibrated = true;
    }
    else
    {
      return;
    }
  }

  altitude = rawAltitude - initialAltitude;

  float currentDiff = abs(altitude - lastAltitude);
  if (currentDiff > CONSECUTIVE_DIFF_THRESHOLD)
  {
    consecutiveDiffCount++;
  }
  else
  {
    consecutiveDiffCount = 0;
  }

  if (altitudeValid && altitude > highestAltitude)
  {
    highestAltitude = altitude;
    peakAltitude = highestAltitude;
    consecutiveBelowPeakCount = 0;

    float highestDiff = abs(highestAltitude - lastHighestAltitude);
    if (highestDiff > MOVING_DIFF_THRESHOLD)
    {
      movingDiffCount++;
    }
    else
    {
      movingDiffCount = 0;
    }

    lastHighestAltitude = highestAltitude;
  }
  else if (altitudeValid && altitude < (peakAltitude - DESCENT_DETECTION_THRESHOLD))
  {
    consecutiveBelowPeakCount++;
  }
  else if (altitudeValid)
  {
    consecutiveBelowPeakCount = 0;
  }

  lastAltitude = altitude;

  float maxDiff = 0;
  for (int i = 0; i < 5; i++)
  {
    for (int j = i + 1; j < 5; j++)
    {
      float diff = abs(lastReadings[i] - lastReadings[j]);
      if (diff > maxDiff)
        maxDiff = diff;
    }
  }
  sensorStable = (maxDiff <= STABILITY_THRESHOLD);
  isStationary = (maxDiff <= STATIONARY_THRESHOLD);

  if (lastDescentTime > 0)
  {
    float timeDiff = (millis() - lastDescentTime) / 1000.0;
    velocity = (altitude - lastDescentCheck) / (timeDiff * 100.0);
  }
  lastDescentCheck = altitude;
  lastDescentTime = millis();

  // Only print essential information at reduced frequency
  if (counter % 5 == 0)
  { // Changed from 10 to 5 for more frequent updates
    Serial.print("Current Altitude: ");
    Serial.print(altitude / 100.0);
    Serial.println(" m");

    Serial.print("Highest Altitude: ");
    Serial.print(highestAltitude / 100.0);
    Serial.println(" m");

    Serial.print("Velocity: ");
    Serial.print(velocity);
    Serial.println(" m/s");
  }
  counter++;
}

float changeOfAltitude()
{
  if (lastAltitudeTime == 0)
  {
    // First call - initialize values
    lastAltitude = altitude;
    lastAltitudeTime = millis();
    return 0;
  }

  // Calculate time difference in seconds
  float timeDiff = (millis() - lastAltitudeTime) / 1000.0;

  // Calculate altitude change rate in meters per second
  float altitudeChange = (altitude - lastAltitude) / timeDiff;

  // Update last values for next calculation
  lastAltitude = altitude;
  lastAltitudeTime = millis();

  return altitudeChange;
}

void calculateGPS()
{
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);

    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);

    if (gps.time.isValid())
    {
      Serial.print("Time (UTC): ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());
    }
    else
    {
      Serial.println("Time not available.");
    }

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());

    Serial.println("-----------");
  }
}

// Function for altitude and pressure sensor monitoring thread
void altitudeThread()
{
  // Initialize sensor locally in the thread
  if (!bmp.begin_I2C(0x77, &Wire1))
  {
    Serial.println("Failed to initialize BMP sensor in altitude thread!");
    return;
  }

  // Configure the sensor for optimal performance
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); // Increase to 50Hz for faster readings

  // Local variables for sensor processing
  float altitudeBuffer[AVG_SAMPLES] = {0};
  int bufferIndex = 0;
  float lastReading = 0;
  int failedReadings = 0;
  bool calibrated = false;
  float initialAlt = 0;
  float peakAlt = 0;

  // For calibration
  constexpr int CALIBRATION_SAMPLES = 10;
  float calibrationValues[CALIBRATION_SAMPLES];

  // Get and display current atmospheric pressure
  if (bmp.performReading()) {
    float currentPressure = bmp.pressure / 100.0;
    Serial.print("Current atmospheric pressure: ");
    Serial.print(currentPressure);
    Serial.println(" hPa");
    
    Serial.print("Current altitude with standard sea level pressure (");
    Serial.print(SEALEVELPRESSURE_HPA);
    Serial.print(" hPa): ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" meters");
    
    // Calculate the actual sea level pressure based on known altitude
    // Uncomment and set your known altitude if needed
    // float knownAltitude = 0.0; // Replace with your actual altitude in meters
    // float calculatedSeaLevel = bmp.seaLevelForAltitude(knownAltitude, currentPressure);
    // Serial.print("Calculated sea level pressure: ");
    // Serial.print(calculatedSeaLevel);
    // Serial.println(" hPa");
  }

  // Calibrate sensor
  int goodSamples = 0;
  Serial.println("Calibrating altitude sensor...");
  while (goodSamples < CALIBRATION_SAMPLES)
  {
    if (bmp.performReading())
    {
      float reading = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 100.0;
      if (reading > MIN_VALID_ALTITUDE && reading < MAX_VALID_ALTITUDE)
      {
        calibrationValues[goodSamples] = reading;
        Serial.print("Calibration sample ");
        Serial.print(goodSamples + 1);
        Serial.print(": ");
        Serial.print(reading/100.0);
        Serial.println(" meters");
        goodSamples++;
      }
    }
    threads.yield(); // Allow other threads to run during calibration
    threads.delay(20);
  }

  // Calculate average initial altitude
  float sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    sum += calibrationValues[i];
  }
  initialAlt = sum / CALIBRATION_SAMPLES;
  calibrated = true;

  Serial.print("Final calibration: Initial altitude set as zero reference point (");
  Serial.print(initialAlt/100.0);
  Serial.println(" meters)");

  // Store in thread-safe variable
  safeData.initialAltitude.store(initialAlt);

  unsigned long lastVelocityCalc = millis();
  float lastAltForVel = initialAlt;

  // Main sensor loop
  while (true)
  {
    // Read sensor
    bool validReading = false;
    float rawAltitude = 0;

    if (bmp.performReading())
    {
      // Get raw altitude in cm
      rawAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 100.0;

      // Update temperature and pressure in thread-safe variables
      safeData.temperature.store(bmp.temperature);
      safeData.pressure.store(bmp.pressure / 100.0); // Convert to hPa

      failedReadings = 0;
      validReading = true;
    }
    else
    {
      failedReadings++;
      if (failedReadings >= MAX_FAILED_READINGS)
      {
        safeData.altitudeValid.store(false);
        threads.delay(20);
        continue;
      }
    }

    // Skip invalid readings
    if (!validReading || rawAltitude < MIN_VALID_ALTITUDE || rawAltitude > MAX_VALID_ALTITUDE)
    {
      threads.delay(20);
      continue;
    }

    // Perform smoothing
    altitudeBuffer[bufferIndex] = rawAltitude;
    bufferIndex = (bufferIndex + 1) % AVG_SAMPLES;

    float smoothedAltitude = 0;
    for (int i = 0; i < AVG_SAMPLES; i++)
    {
      smoothedAltitude += altitudeBuffer[i];
    }
    smoothedAltitude /= AVG_SAMPLES;

    // Normalize to initial altitude if calibrated
    float relativeAltitude = calibrated ? (smoothedAltitude - initialAlt) : smoothedAltitude;

    // Calculate velocity every 100ms
    unsigned long currentTime = millis();
    if (currentTime - lastVelocityCalc >= 100)
    {                                                                              // 10Hz velocity updates
      float timeDiff = (currentTime - lastVelocityCalc) / 1000.0;                  // seconds
      float velocityMps = (relativeAltitude - lastAltForVel) / (timeDiff * 100.0); // Convert cm to m

      // Store in thread-safe variable
      safeData.velocity.store(velocityMps);

      lastAltForVel = relativeAltitude;
      lastVelocityCalc = currentTime;
    }

    // Track highest altitude
    if (relativeAltitude > safeData.highestAltitude.load())
    {
      safeData.highestAltitude.store(relativeAltitude);
      peakAlt = relativeAltitude;
    }

    // Store processed values in thread-safe variables
    safeData.altitude.store(relativeAltitude);
    safeData.altitudeValid.store(true);

    // Check stability
    if (abs(smoothedAltitude - lastReading) <= STABILITY_THRESHOLD)
    {
      safeData.sensorStable.store(true);
    }
    else
    {
      safeData.sensorStable.store(false);
    }

    lastReading = smoothedAltitude;
    threads.delay(BMP_UPDATE_INTERVAL); // ~25Hz update rate
  }
}

// Function for GPS sensor thread
void gpsThread()
{
  // Configure GPS on Serial5
  gpsSerial.begin(9600);
  
  Serial.println("GPS Thread started - using Serial5");
  uint8_t gpsRetryCount = 0;
  bool gpsInitialized = false;
  
  // Add a small delay to ensure serial port is ready
  threads.delay(500);
  
  while (!gpsInitialized && gpsRetryCount < 5) {
    if (gpsSerial) {
      Serial.println("GPS Serial5 initialized successfully");
      gpsInitialized = true;
    } else {
      gpsRetryCount++;
      Serial.print("GPS Serial5 initialization attempt ");
      Serial.print(gpsRetryCount);
      Serial.println(" failed, retrying...");
      gpsSerial.begin(9600);
      threads.delay(500);
    }
  }
  
  if (!gpsInitialized) {
    Serial.println("WARNING: Failed to initialize GPS on Serial5!");
  }
  
  // Main GPS processing loop
  unsigned long lastSatelliteReport = 0;
  
  while (true)
  {
    // Process all available GPS data
    while (gpsSerial.available() > 0)
    {
      char c = gpsSerial.read();
      gps.encode(c);
    }

    // Update GPS data with mutex protection when we have a new position
    if (gps.location.isUpdated())
    {
      gpsMutex.lock();
      safeData.latitude = gps.location.lat();
      safeData.longitude = gps.location.lng();
      safeData.satellites = gps.satellites.value();

      // Update GPS time if valid
      if (gps.time.isValid())
      {
        char timeStr[10];
        sprintf(timeStr, "%02d:%02d", gps.time.hour(), gps.time.minute());
        safeData.gpsTime = String(timeStr);
      }
      
      // Log updated GPS data for debugging
      Serial.print("GPS update - Lat: ");
      Serial.print(safeData.latitude, 6);
      Serial.print(", Long: ");
      Serial.print(safeData.longitude, 6);
      Serial.print(", Sats: ");
      Serial.println(safeData.satellites);
      
      gpsMutex.unlock();
    }
    
    // Periodically report satellite count even if position hasn't been updated
    unsigned long currentTime = millis();
    if (currentTime - lastSatelliteReport > 5000) { // Every 5 seconds
      Serial.print("GPS satellites in view: ");
      Serial.println(gps.satellites.value());
      lastSatelliteReport = currentTime;
    }

    threads.delay(GPS_UPDATE_INTERVAL); // 10Hz GPS processing
  }
}

// Function for telemetry thread
void telemetryThread()
{
  // Initialize telemetry data
  TelemetryData threadTelemetry = {};
  initializeTelemetryData(threadTelemetry);

  // Wait for communication to establish
  while (!commEstablished)
  {
    threads.delay(500);
  }

  // Main telemetry loop
  while (true)
  {
    if (telemetryEnabled)
    {
      // Capture the current state with mutex protection
      State currentStateSnapshot;
      {
        stateMutex.lock();
        currentStateSnapshot = currentState;
        stateMutex.unlock();
      }

      // Update packet count thread-safely
      threadTelemetry.PACKET_COUNT = safeData.packetCount.fetch_add(1);

      // Get current time (could be improved with RTC)
      unsigned long uptime = millis() / 1000; // Convert to seconds
      int hours = uptime / 3600;
      int minutes = (uptime % 3600) / 60;
      int seconds = uptime % 60;
      char timeStr[10];
      sprintf(timeStr, "%d:%d:%d", hours, minutes, seconds);
      threadTelemetry.MISSION_TIME = String(timeStr);

      // Copy current sensor values
      threadTelemetry.ALTITUDE = safeData.altitude.load() / 100.0; // Convert cm to m
      threadTelemetry.TEMPERATURE = safeData.temperature.load();
      threadTelemetry.PRESSURE = safeData.pressure.load();
      threadTelemetry.VOLTAGE = safeData.voltage.load();
      threadTelemetry.STATE = static_cast<int>(currentStateSnapshot);

      // Get GPS data with mutex protection
      {
        gpsMutex.lock();
        threadTelemetry.GPS_LATITUDE = String(safeData.latitude, 6);
        threadTelemetry.GPS_LONGITUDE = String(safeData.longitude, 6);
        threadTelemetry.GPS_SATS = String(safeData.satellites);
        threadTelemetry.GPS_TIME = safeData.gpsTime;
        threadTelemetry.GPS_ALTITUDE = String(safeData.altitude.load() / 100.0); // Use BMP altitude
        gpsMutex.unlock();
      }

      // IMPORTANT: Update the global telemetryData (needed for constructMessage)
      xbeeMutex.lock();
      telemetryData = threadTelemetry;
      
      // Construct message using updated telemetryData
      String dataToSend = "<T" + constructMessage() + ">";
      
      // Send data to XBee
      Serial8.println(dataToSend);
      
      // Debug output
      Serial.println("Sending telemetry packet #" + String(threadTelemetry.PACKET_COUNT));
      Serial.println("Altitude: " + String(threadTelemetry.ALTITUDE) + " m");
      Serial.println("Temperature: " + String(threadTelemetry.TEMPERATURE) + " C");
      Serial.println("Pressure: " + String(threadTelemetry.PRESSURE) + " hPa");
      
      // Add GPS debug output
      Serial.println("GPS Latitude: " + threadTelemetry.GPS_LATITUDE);
      Serial.println("GPS Longitude: " + threadTelemetry.GPS_LONGITUDE);
      Serial.println("GPS Satellites: " + threadTelemetry.GPS_SATS);
      Serial.println("GPS Time: " + threadTelemetry.GPS_TIME);

      // Serial.println("Voltage: " + threadTelemetry.VOLTAGE + " V");
      
      xbeeMutex.unlock();
    }

    threads.delay(TELEMETRY_INTERVAL); // 1Hz telemetry updates
  }
}

// Function for state monitoring thread
void stateMonitorThread()
{
  // Wait for sensors to initialize
  threads.delay(2000);

  State currentStateLocal = BOOT;
  State lastStateLocal = BOOT;

  // Local variables for state monitoring
  int consecutiveBelowPeakCount = 0;
  bool cxonReceived = false;

  // Main state monitoring loop
  while (true)
  {
    // Get current altitude and status
    float altitude = safeData.altitude.load();
    float highestAltitude = safeData.highestAltitude.load();
    float velocity = safeData.velocity.load();
    bool altitudeValid = safeData.altitudeValid.load();
    bool sensorStable = safeData.sensorStable.load();

    // Update current state (with mutex protection)
    {
      stateMutex.lock();
      currentStateLocal = currentState;
      stateMutex.unlock();
    }

    // State transition logic
    State nextState = currentStateLocal;

    switch (currentStateLocal)
    {
    case RECOVER:
      nextState = IDLE;
      break;

    case BOOT:
      nextState = IDLE;
      break;

    case IDLE:
      if (cxonReceived || cxonCommandReceived)
      {
        nextState = PRE_LAUNCH;
      }
      break;

    case PRE_LAUNCH:
      if (sensorStable && altitudeValid)
      {
        nextState = ASCENDING;
      }
      break;

    case ASCENDING:
      if (altitudeValid && highestAltitude > 0)
      {
        // Check for descent
        if (altitude < (highestAltitude - DESCENT_DETECTION_THRESHOLD))
        {
          consecutiveBelowPeakCount++;

          // Confirm descent with multiple readings
          if (consecutiveBelowPeakCount >= DESCENT_DETECTION_COUNT)
          {
            nextState = DESCENDING_1;
          }
        }
        else
        {
          consecutiveBelowPeakCount = 0;
        }
      }
      break;

    case DESCENDING_1:
      if (sensorStable && altitude <= (highestAltitude * DESCENT_2_THRESHOLD))
      {
        nextState = DESCENDING_2;
      }
      break;

    case DESCENDING_2:
      if (sensorStable && altitude < landingThresholdAltitude && abs(velocity) < 0.5)
      {
        nextState = LANDING;
      }
      break;

    case LANDING:
      // Terminal state, no transition
      break;
    }

    // Update state if changed (with mutex protection)
    if (nextState != currentStateLocal)
    {
      stateMutex.lock();
      lastState = currentState;
      currentState = nextState;
      stateMutex.unlock();

      // Log state transition
      Serial.print("State transition: ");
      Serial.print(getStateString(currentStateLocal));
      Serial.print(" -> ");
      Serial.println(getStateString(nextState));

      // Update local state
      lastStateLocal = currentStateLocal;
      currentStateLocal = nextState;
    }

    threads.delay(100); // 10Hz state monitoring
  }
}

// Function for command processing thread
void commandThread() {
  static String receivedData = "";

  while (true) {
    // Check for XBee commands with mutex protection
    xbeeMutex.lock();
    bool dataAvailable = xbeeSerial.available() > 0;
    xbeeMutex.unlock();
    
    if (dataAvailable) {
      xbeeMutex.lock();
      char character = xbeeSerial.read();
      xbeeMutex.unlock();
      
      receivedData += character;
      
      // Debug print to verify data reception
      if (character == '>' || character == '\n') {
        Serial.print("Raw XBee data received: ");
        Serial.println(receivedData);
      }
      
      // Process complete commands
      if (receivedData.indexOf('<') != -1 && receivedData.indexOf('>') != -1) {
        int startIdx = receivedData.indexOf('<') + 1;
        int endIdx = receivedData.indexOf('>');
        
        if (startIdx < endIdx) {
          String extractedData = receivedData.substring(startIdx, endIdx);
          
          // Log the received command for debugging
          Serial.print("Command received: ");
          Serial.println(extractedData);
          
          // Process the command based on type
          if (extractedData.length() > 0) {
            char cmdType = extractedData.charAt(0);
            
            switch (cmdType) {
              case 'C': // Control commands
                handleGCSCommand(extractedData.substring(1));
                break;
                
              default:
                Serial.print("Unknown command type: ");
                Serial.println(cmdType);
                break;
            }
          }
        }
        
        receivedData = ""; // Reset for next command
      }
      
      // Reset buffer if it gets too long without finding a complete command
      if (receivedData.length() > 100) {
        Serial.println("Command buffer overflow - resetting");
        receivedData = "";
      }
    }
    
    threads.delay(20); // 50Hz command checking
  }
}

// Timer interrupt for precision control (if needed)
IntervalTimer stateTimer;