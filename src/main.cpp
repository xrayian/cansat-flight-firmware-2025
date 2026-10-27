/*
 * CanSat Flight Software - Multithreaded Reference Implementation
 * Teensy 4.1 + TeensyThreads
 *
 * Reference code for CanSat teams. This is NOT the software flown at
 * CanSat 2025. See README.md for the architecture and the threading rules.
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
#include <atomic>
#include <math.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define SEALEVELPRESSURE_HPA (1013.25) // Standard sea-level pressure (hPa)
#define BMP_UPDATE_INTERVAL 40         // 25 Hz (ms)
#define GPS_UPDATE_INTERVAL 100        // 10 Hz (ms)
#define TELEMETRY_INTERVAL 1000        // 1 Hz (ms)
#define STATE_UPDATE_INTERVAL 100      // 10 Hz (ms)
#define COMMAND_POLL_INTERVAL 20       // 50 Hz (ms)
#define THREAD_STACK_SIZE 4096         // Bytes per thread

#define xbeeSerial Serial8             // XBee: TX8 = pin 35, RX8 = pin 34
HardwareSerial &gpsSerial = Serial5;

const char *TEAM_ID = "3158";
const char *HANDSHAKE_REQUEST = "<A3158GCS>";
const char *HANDSHAKE_ACK = "<A3158ACK>";

// Altitudes are in centimetres unless the name says otherwise.
const int AVG_SAMPLES = 3;
const int CALIBRATION_SAMPLES = 10;
const int MAX_FAILED_READINGS = 5;
const float MIN_VALID_ALTITUDE = -10000.0;      // -100 m
const float MAX_VALID_ALTITUDE = 1000000.0;     // 10 km
const float STABILITY_THRESHOLD = 50.0;         // 0.5 m between readings
const float ASCENT_THRESHOLD = 100.0;           // 1 m above the pad
const int ASCENT_CONFIRMATION_COUNT = 5;
const float DESCENT_DETECTION_THRESHOLD = 50.0; // 0.5 m below peak
const int DESCENT_DETECTION_COUNT = 4;
const float DESCENT_2_RATIO = 0.75;             // Below 75% of peak
const float LANDING_THRESHOLD = 500.0;          // 5 m
const float LANDING_VELOCITY_MPS = 0.5;         // m/s

const int BATTERY_PIN = 15;          // Voltage divider output
const float ADC_REFERENCE = 3.3;     // Teensy 4.1 analog reference (V)
const float DIVIDER_MULTIPLIER = 5.0;

// ---------------------------------------------------------------------------
// Hardware objects, each owned by exactly one thread
// ---------------------------------------------------------------------------
Adafruit_BMP3XX bmp; // altitudeThread only. Add a Wire1 lock if another
                     // thread (for example an IMU thread) shares the bus.
TinyGPSPlus gps;     // gpsThread only.

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum State {
    RECOVER,
    BOOT,
    IDLE,
    PRE_LAUNCH,
    ASCENDING,
    DESCENDING_1,
    DESCENDING_2,
    LANDING
};

std::atomic<State> currentState{BOOT}; // Written only by stateMonitorThread.

int altitudeThreadID = -1;
int gpsThreadID = -1;
int commandThreadID = -1;
int stateMonitorThreadID = -1;
int telemetryThreadID = -1;

// ---------------------------------------------------------------------------
// Locks: one per shared resource
// ---------------------------------------------------------------------------
Threads::Mutex serialLock; // USB Serial (debug log)
Threads::Mutex xbeeLock;   // Serial8 (XBee radio)
Threads::Mutex gpsLock;    // gpsData
Threads::Mutex cmdLock;    // lastCmdEcho

// ---------------------------------------------------------------------------
// Shared data
// ---------------------------------------------------------------------------
struct SensorData {
    std::atomic<float> altitude{0};        // cm, relative to the pad
    std::atomic<float> initialAltitude{0}; // cm, absolute
    std::atomic<float> highestAltitude{0}; // cm, relative
    std::atomic<float> velocity{0};        // m/s, positive is up
    std::atomic<float> temperature{0};     // C
    std::atomic<float> pressure{0};        // hPa
    std::atomic<float> voltage{0};         // V
    std::atomic<bool> altitudeValid{false};
    std::atomic<bool> sensorStable{false};
    std::atomic<int> packetCount{0};
};
SensorData sensors;

struct GpsSnapshot {
    double latitude = 0;
    double longitude = 0;
    float altitude = 0; // m, above mean sea level
    int satellites = 0;
    String time = "0:0";
};
GpsSnapshot gpsData; // Guarded by gpsLock. Access through readGps()/writeGps().

GpsSnapshot readGps() {
    Threads::Scope lock(gpsLock);
    return gpsData;
}

void writeGps(const GpsSnapshot &g) {
    Threads::Scope lock(gpsLock);
    gpsData = g;
}

String lastCmdEcho = ""; // Guarded by cmdLock. Access through the helpers.

String readCmdEcho() {
    Threads::Scope lock(cmdLock);
    return lastCmdEcho;
}

void writeCmdEcho(const String &cmd) {
    Threads::Scope lock(cmdLock);
    lastCmdEcho = cmd;
}

std::atomic<bool> telemetryEnabled{false};
std::atomic<bool> cxonCommandReceived{false};

// ---------------------------------------------------------------------------
// Telemetry packet (built locally by telemetryThread, never shared)
// ---------------------------------------------------------------------------
struct TelemetryData {
    String MISSION_TIME = "0:0:0";
    int PACKET_COUNT = 0;
    String MODE = "F";
    State STATE = BOOT;
    float ALTITUDE = 0;
    float TEMPERATURE = 0;
    float PRESSURE = 0;
    float VOLTAGE = 0;
    float GYRO_R = 0, GYRO_P = 0, GYRO_Y = 0;
    float ACCEL_R = 0, ACCEL_P = 0, ACCEL_Y = 0;
    float MAG_R = 0, MAG_P = 0, MAG_Y = 0;
    float AUTO_GYRO_ROTATION_RATE = 0;
    String GPS_TIME = "0:0";
    String GPS_ALTITUDE = "0";
    String GPS_LATITUDE = "0";
    String GPS_LONGITUDE = "0";
    String GPS_SATS = "0";
    String CMD_ECHO = "";
};

// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------
String getStateString(State state);
String constructMessage(const TelemetryData &t);
void handleGCSCommand(const String &cmd);
void processBatteryVoltage();
void confirmGroundStationConn();
void logLine(const String &msg);

void altitudeThread();
void gpsThread();
void commandThread();
void stateMonitorThread();
void telemetryThread();

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
// Single lines go through logLine(). For a multi-line block, copy the data
// first, then hold serialLock once for the whole block.
void logLine(const String &msg) {
    Threads::Scope lock(serialLock);
    Serial.println(msg);
}

// ---------------------------------------------------------------------------
// setup() and loop()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    xbeeSerial.begin(9600); // XBee 900HP default baud rate
    analogReadResolution(12);
    Wire1.begin();

    // No threads are running yet, so direct Serial use is safe here.
    Serial.println("CanSat Flight Software - Multithreaded Reference");
    Serial.println("Waiting for the ground station...");

    confirmGroundStationConn();

    altitudeThreadID     = threads.addThread(altitudeThread, 0, THREAD_STACK_SIZE);
    gpsThreadID          = threads.addThread(gpsThread, 0, THREAD_STACK_SIZE);
    commandThreadID      = threads.addThread(commandThread, 0, THREAD_STACK_SIZE);
    stateMonitorThreadID = threads.addThread(stateMonitorThread, 0, THREAD_STACK_SIZE);
    telemetryThreadID    = threads.addThread(telemetryThread, 0, THREAD_STACK_SIZE);

    // TeensyThreads schedules round-robin; start order is not priority.
    // To give a thread more CPU time: threads.setTimeSlice(threadID, ticks);

    logLine("All threads started.");
}

void loop() {
    static unsigned long lastStatusTime = 0;
    unsigned long now = millis();

    if (now - lastStatusTime >= 5000) {
        State state = currentState.load();
        float alt = sensors.altitude.load() / 100.0f;
        float peak = sensors.highestAltitude.load() / 100.0f;
        float vel = sensors.velocity.load();
        float temp = sensors.temperature.load();
        bool txOn = telemetryEnabled.load();

        Threads::Scope lock(serialLock);
        Serial.println("===== System Status =====");
        Serial.print(threads.threadsInfo());
        Serial.print("State: ");       Serial.println(getStateString(state));
        Serial.print("Altitude: ");    Serial.print(alt);  Serial.println(" m");
        Serial.print("Peak: ");        Serial.print(peak); Serial.println(" m");
        Serial.print("Velocity: ");    Serial.print(vel);  Serial.println(" m/s");
        Serial.print("Temperature: "); Serial.print(temp); Serial.println(" C");
        Serial.print("Telemetry: ");   Serial.println(txOn ? "ON" : "OFF");
        Serial.println("=========================");

        lastStatusTime = now;
    }

    threads.delay(50);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void confirmGroundStationConn() {
    // Runs in setup() before any thread starts.
    while (true) {
        xbeeSerial.println(HANDSHAKE_REQUEST);
        Serial.print(".");

        unsigned long start = millis();
        while (millis() - start < 2000) {
            if (xbeeSerial.available() > 0) {
                String reply = xbeeSerial.readStringUntil('\n');
                reply.trim();
                if (reply == HANDSHAKE_ACK) {
                    Serial.println("\nGround station connected.");
                    return;
                }
                Serial.println("\nUnexpected reply: " + reply);
            }
        }
    }
}

void processBatteryVoltage() {
    int raw = analogRead(BATTERY_PIN);
    float vOut = (raw / 4095.0f) * ADC_REFERENCE;
    sensors.voltage.store(vOut * DIVIDER_MULTIPLIER);
}

void handleGCSCommand(const String &cmd) {
    writeCmdEcho(cmd);
    logLine("Ground station command: " + cmd);

    if (cmd.equals("CX/ON")) {
        telemetryEnabled.store(true);
        cxonCommandReceived.store(true);
    } else if (cmd.equals("CX/OFF")) {
        telemetryEnabled.store(false);
    } else {
        logLine("Unknown command: " + cmd);
    }
}

String getStateString(State state) {
    switch (state) {
        case RECOVER:      return "RECOVER";
        case BOOT:         return "BOOT";
        case IDLE:         return "IDLE";
        case PRE_LAUNCH:   return "PRE_LAUNCH";
        case ASCENDING:    return "ASCENDING";
        case DESCENDING_1: return "DESCENDING_1";
        case DESCENDING_2: return "DESCENDING_2";
        case LANDING:      return "LANDING";
        default:           return "UNKNOWN";
    }
}

String constructMessage(const TelemetryData &t) {
    return String(TEAM_ID) + ", " +
           t.MISSION_TIME + ", " +
           t.PACKET_COUNT + ", " +
           t.MODE + ", " +
           getStateString(t.STATE) + ", " +
           t.ALTITUDE + ", " +
           t.TEMPERATURE + ", " +
           t.PRESSURE + ", " +
           t.VOLTAGE + ", " +
           t.GYRO_R + ", " + t.GYRO_P + ", " + t.GYRO_Y + ", " +
           t.ACCEL_R + ", " + t.ACCEL_P + ", " + t.ACCEL_Y + ", " +
           t.MAG_R + ", " + t.MAG_P + ", " + t.MAG_Y + ", " +
           t.AUTO_GYRO_ROTATION_RATE + ", " +
           t.GPS_TIME + ", " +
           t.GPS_ALTITUDE + ", " +
           t.GPS_LATITUDE + ", " +
           t.GPS_LONGITUDE + ", " +
           t.GPS_SATS + ", " +
           t.CMD_ECHO + ", ";
}

// ---------------------------------------------------------------------------
// Altitude thread: sole writer of altitude, peak, velocity, temperature,
// pressure, altitudeValid and sensorStable
// ---------------------------------------------------------------------------
void altitudeThread() {
    if (!bmp.begin_I2C(0x77, &Wire1)) {
        logLine("BMP388 not found on Wire1. Altitude thread stopped.");
        sensors.altitudeValid.store(false);
        return;
    }

    // Set after begin_I2C(), which can reset the bus to its default speed.
    Wire1.setClock(400000);

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Calibrate: the average of the first valid readings becomes zero.
    logLine("Calibrating altitude sensor...");
    float sum = 0;
    int goodSamples = 0;
    while (goodSamples < CALIBRATION_SAMPLES) {
        if (bmp.performReading()) {
            float reading = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 100.0f;
            if (reading > MIN_VALID_ALTITUDE && reading < MAX_VALID_ALTITUDE) {
                sum += reading;
                goodSamples++;
            }
        }
        threads.delay(20);
    }
    const float initialAlt = sum / CALIBRATION_SAMPLES;
    sensors.initialAltitude.store(initialAlt);
    logLine("Calibration done. Pad altitude: " + String(initialAlt / 100.0f) + " m");

    float buffer[AVG_SAMPLES];
    for (int i = 0; i < AVG_SAMPLES; i++) buffer[i] = initialAlt;
    int bufferIndex = 0;
    float lastSmoothed = initialAlt;
    int failedReadings = 0;
    float peak = 0;

    unsigned long lastVelocityCalc = millis();
    float lastAltForVel = 0;

    while (true) {
        if (!bmp.performReading()) {
            if (failedReadings < MAX_FAILED_READINGS) failedReadings++;
            if (failedReadings >= MAX_FAILED_READINGS) sensors.altitudeValid.store(false);
            threads.delay(20);
            continue;
        }
        failedReadings = 0;

        float rawAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) * 100.0f;
        sensors.temperature.store(bmp.temperature);
        sensors.pressure.store(bmp.pressure / 100.0f);

        if (rawAltitude < MIN_VALID_ALTITUDE || rawAltitude > MAX_VALID_ALTITUDE) {
            threads.delay(20);
            continue;
        }

        // Moving average
        buffer[bufferIndex] = rawAltitude;
        bufferIndex = (bufferIndex + 1) % AVG_SAMPLES;
        float smoothed = 0;
        for (int i = 0; i < AVG_SAMPLES; i++) smoothed += buffer[i];
        smoothed /= AVG_SAMPLES;

        float relativeAltitude = smoothed - initialAlt;

        // Vertical velocity at 10 Hz
        unsigned long now = millis();
        if (now - lastVelocityCalc >= 100) {
            float dt = (now - lastVelocityCalc) / 1000.0f;
            sensors.velocity.store((relativeAltitude - lastAltForVel) / (dt * 100.0f));
            lastAltForVel = relativeAltitude;
            lastVelocityCalc = now;
        }

        if (relativeAltitude > peak) {
            peak = relativeAltitude;
            sensors.highestAltitude.store(peak);
        }

        sensors.altitude.store(relativeAltitude);
        sensors.altitudeValid.store(true);
        sensors.sensorStable.store(fabsf(smoothed - lastSmoothed) <= STABILITY_THRESHOLD);
        lastSmoothed = smoothed;

        threads.delay(BMP_UPDATE_INTERVAL);
    }
}

// ---------------------------------------------------------------------------
// GPS thread: sole owner of the gps object and Serial5
// ---------------------------------------------------------------------------
void gpsThread() {
    gpsSerial.begin(9600);
    logLine("GPS thread started on Serial5.");

    unsigned long lastSatelliteReport = 0;

    while (true) {
        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }

        if (gps.location.isUpdated()) {
            GpsSnapshot g = readGps();
            g.latitude = gps.location.lat();
            g.longitude = gps.location.lng();
            g.satellites = gps.satellites.value();
            if (gps.altitude.isValid()) {
                g.altitude = gps.altitude.meters();
            }
            if (gps.time.isValid()) {
                char timeStr[10];
                snprintf(timeStr, sizeof(timeStr), "%02d:%02d",
                         gps.time.hour(), gps.time.minute());
                g.time = String(timeStr);
            }
            writeGps(g);

            logLine("GPS fix: " + String(g.latitude, 6) + ", " + String(g.longitude, 6) +
                    " | Sats: " + String(g.satellites));
        }

        unsigned long now = millis();
        if (now - lastSatelliteReport > 5000) {
            logLine("GPS satellites in view: " + String(gps.satellites.value()));
            lastSatelliteReport = now;
        }

        threads.delay(GPS_UPDATE_INTERVAL);
    }
}

// ---------------------------------------------------------------------------
// Command thread: owns XBee reads
// Commands arrive framed as <C + command + >, for example <CCX/ON>.
// ---------------------------------------------------------------------------
void commandThread() {
    String received = "";

    while (true) {
        // Drain everything waiting in one short locked section, parse outside it.
        char chunk[64];
        int n = 0;
        {
            Threads::Scope lock(xbeeLock);
            while (n < (int)sizeof(chunk) && xbeeSerial.available() > 0) {
                chunk[n++] = (char)xbeeSerial.read();
            }
        }

        for (int i = 0; i < n; i++) {
            received += chunk[i];

            int start = received.indexOf('<');
            int end = (start == -1) ? -1 : received.indexOf('>', start + 1);
            if (end != -1) {
                String payload = received.substring(start + 1, end);
                received = received.substring(end + 1);

                if (payload.length() > 1 && payload.charAt(0) == 'C') {
                    handleGCSCommand(payload.substring(1));
                } else {
                    logLine("Ignored frame: <" + payload + ">");
                }
            }

            if (received.length() > 100) {
                logLine("Command buffer overflow, resetting.");
                received = "";
            }
        }

        threads.delay(COMMAND_POLL_INTERVAL);
    }
}

// ---------------------------------------------------------------------------
// State monitor thread: sole writer of currentState
// ---------------------------------------------------------------------------
void stateMonitorThread() {
    threads.delay(2000); // Let the sensors settle.

    int ascentCount = 0;
    int belowPeakCount = 0;

    while (true) {
        float altitude = sensors.altitude.load();       // cm
        float highest = sensors.highestAltitude.load(); // cm
        float velocity = sensors.velocity.load();       // m/s
        bool altitudeValid = sensors.altitudeValid.load();
        bool sensorStable = sensors.sensorStable.load();

        State state = currentState.load();
        State next = state;

        switch (state) {
            case RECOVER:
            case BOOT:
                next = IDLE;
                break;

            case IDLE:
                if (cxonCommandReceived.load()) next = PRE_LAUNCH;
                break;

            case PRE_LAUNCH:
                // Climbing past 1 m for several consecutive readings means launch.
                if (altitudeValid && altitude > ASCENT_THRESHOLD) {
                    if (++ascentCount >= ASCENT_CONFIRMATION_COUNT) next = ASCENDING;
                } else {
                    ascentCount = 0;
                }
                break;

            case ASCENDING:
                // Several consecutive readings below the peak means apogee passed.
                if (altitudeValid && highest > 0) {
                    if (altitude < highest - DESCENT_DETECTION_THRESHOLD) {
                        if (++belowPeakCount >= DESCENT_DETECTION_COUNT) next = DESCENDING_1;
                    } else {
                        belowPeakCount = 
