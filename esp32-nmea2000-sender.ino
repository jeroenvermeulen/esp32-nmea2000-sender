/*
 Inspired by https://github.com/AK-Homberger/NMEA2000-Data-Sender
*/

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4
#define VOLTAGE_PIN GPIO_NUM_35      // Voltage measure is connected GPIO 35 (Analog ADC1_CH7)
#define VAPOR_PIN GPIO_NUM_34        // Voltage measure is connected GPIO 34 (Analog ADC1_CH6)

#include <Arduino.h>
#include <Preferences.h>      // ESP32
#include <WiFi.h>             // ESP32
#include <Wire.h>             // ESP32
#include <esp_mac.h>          // ESP32
#include <NMEA2000_CAN.h>     // https://github.com/ttlappalainen/NMEA2000
#include <N2kMessages.h>      // https://github.com/ttlappalainen/NMEA2000
#include "Adafruit_SHT4x.h"   // Library Man: Adafruit SHT4x by Adafruit
#include <Adafruit_BNO055.h>  // Library Man: Adafruit BNO055 by Adafruit
//// also needed:
// mcp_can.h                         // https://github.com/ttlappalainen/CAN_BUS_Shield
// NMEA2000_mcp.h                    // https://github.com/ttlappalainen/NMEA2000_mcp

#define ENABLE_DEBUG_LOG 1           // Debug log on serial
#define NMEA_DEBUG_LOG 0             // NMEA message log on serial
#define ADC_Calibration_Value2 19.0  // The real value depends on the true resistor values for the ADC input (100K / 27 K).

int NodeAddress;          // To store last Node Address
Preferences preferences;  // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {  // 126993 // Heartbeat
  // 126996
  127257L,  // Attitude: Yaw,Pitch,Roll
  127501L,  // Universal Binary Status Report
  127505L,  // Fluid Level
  127508L,  // Battery Status
  130313L,  // Humidity
  130316L,  // Temperature extended range
  // 60928
  0
};

// Create schedulers, disabled at the beginning
//                               enabled  period offset
tN2kSyncScheduler AttitudeScheduler(false, 500, 50);
tN2kSyncScheduler TemperatureScheduler(false, 5000, 500);
tN2kSyncScheduler HumidityScheduler(false, 5000, 600);
tN2kSyncScheduler DCStatusScheduler(false, 5000, 700);
tN2kSyncScheduler VaporAlarmScheduler(false, 5000, 800);
tN2kSyncScheduler EngineDynamicScheduler(false, 5000, 900);

// SHT40
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// BNO055
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);


// Global Data
float gBusVoltage = NAN;
float gTemperature = NAN;
float gHumidity = NAN;
float gVaporPercent = NAN;
unsigned long uptimeSec = 0;

// No value passed, just a message
void debugLog(const char* str) {
#if ENABLE_DEBUG_LOG == 1
  unsigned long milli = millis();
  Serial.printf("%8d  ", milli);
  Serial.println(str);
#endif
}

// Overload for 1 value passed
template<typename T>
void debugLog(char* str, T value) {
#if ENABLE_DEBUG_LOG == 1
  unsigned long milli = millis();
  Serial.printf("%8d  ", milli);
  Serial.printf(str, value);
  Serial.println("");
#endif
}

// Overload for 2 values passed
template<typename TA, typename TB>
void debugLog(const char* str, TA valueA, TB valueB) {
#if ENABLE_DEBUG_LOG == 1
  unsigned long milli = millis();
  Serial.printf("%8d  ", milli);
  Serial.printf(str, valueA, valueB);
  Serial.println("");
#endif
}

// Overload for 3 values passed
template<typename TX, typename TY, typename TZ>
void debugLog(const char* str, TX valueX, TY valueY, TZ valueZ) {
#if ENABLE_DEBUG_LOG == 1
  unsigned long milli = millis();
  Serial.printf("%8d  ", milli);
  Serial.printf(str, valueX, valueY, valueZ);
  Serial.println("");
#endif
}

void OnN2kOpen() {
  debugLog("NMEA2000 is Open.");
  // Start schedulers now.
  TemperatureScheduler.UpdateNextTime();
  HumidityScheduler.UpdateNextTime();
  DCStatusScheduler.UpdateNextTime();
  VaporAlarmScheduler.UpdateNextTime();
  EngineDynamicScheduler.UpdateNextTime();
  AttitudeScheduler.UpdateNextTime();
}

int readIntFromStorage(const char* key, int defaultValue) {
  preferences.begin("nvs", false);
  int value = preferences.getInt(key, defaultValue);
  preferences.end();
  debugLog("Received %s from storage: %d", key, value);
  return value;
}

void writeIntToStorage(const char* key, int value) {
  preferences.begin("nvs", false);
  preferences.putInt(key, value);
  preferences.end();
  debugLog("Written %s to storage: %d", key, value);
}

unsigned long getUniqueID() {
  uint8_t chipid[6];
  unsigned long id = 0;
  esp_efuse_mac_get_default(chipid);
  for (int i = 0; i < 6; i++) id += (chipid[i] << (7 * i));
  return id;
}

void i2cScan() {
  Wire.begin();
  byte error, address;
  int nDevices;
  debugLog("Scanning I2C bus...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      debugLog("I2C device found at address 0x%02x", address);
      nDevices++;
    } else if (error == 4) {
      debugLog("Unknown error at address 0x%02x", address);
    }
  }
  if (nDevices == 0) {
    debugLog("No I2C devices found");
  }
}

void setupSHT40() {
  while (!sht4.begin()) {
    debugLog("Couldn't find SHT40 - waiting 5 sec");
    delay(5);
  }
  debugLog("Found SHT40 sensor");
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
}

void setupBNO055() {
  while (!bno.begin()) {
    Serial.print("Couldn't find BNO055  - waiting 5 sec");
    delay(5);
  }
  debugLog("Found BNO055 sensor");
}

void setupN2K() {
  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  // Set product information
  NMEA2000.SetProductInformation("1",                      // Manufacturer's Model serial code
                                 100,                      // Manufacturer's product code
                                 "ESP32 NMEA2000 Sender",  // Manufacturer's Model ID
                                 "0.0.2 (2024-06-04)",     // Manufacturer's Software version code
                                 "1.0.0 (2024-06-04)"      // Manufacturer's Model version
  );
  // Set device information
  NMEA2000.SetDeviceInformation(getUniqueID(),  // Unique number. Use e.g. Serial number.
                                132,            // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25,             // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046            // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
#if NMEA_DEBUG_LOG == 1
  NMEA2000.SetForwardStream(&Serial);
#endif
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);            // Show in clear text. Leave uncommented for default Actisense format.
  NodeAddress = readIntFromStorage("LastNodeAddress", 15);  // Read stored last NodeAddress
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();
}

void setup() {
  Serial.begin(115200);
  debugLog("");
  debugLog("Starting setup...");

  // Disable WiFi
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

  Wire.begin();
  i2cScan();

  setupSHT40();
  setupBNO055();
  setupN2K();

  debugLog("Setup complete.");
  delay(200);
}

// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
double ReadVoltage() {
  double reading = analogRead(VOLTAGE_PIN);  // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) {
    return 0;
  }
  // Polynomial https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function/blob/master/ESP32_ADC_Read_Voltage_Accurate.ino
  reading = (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1000;
  return reading * ADC_Calibration_Value2 / 4096;
}

void SendN2kBattery() {
  if (DCStatusScheduler.IsTime()) {
    DCStatusScheduler.UpdateNextTime();
    double voltageReading = ReadVoltage();
    if (isnan(voltageReading) || 0 >= voltageReading || 100 < voltageReading) {
      debugLog("ERROR Invalid voltage reading: %.02f", voltageReading);
      gBusVoltage = NAN;
    } else {
      if (isnan(gBusVoltage)) {
        gBusVoltage = voltageReading;
      } else {
        // Filter to eliminate spike for ADC readings
        gBusVoltage = ((gBusVoltage * 15) + voltageReading) / 16;
      }
    }
    double voltageSend = (isnan(gBusVoltage)) ? N2kDoubleNA : gBusVoltage;
    debugLog("Volt:         %6.02f V", gBusVoltage);
    tN2kMsg N2kMsg;
    SetN2kDCBatStatus(N2kMsg, 1, voltageSend, N2kDoubleNA, N2kDoubleNA, 1);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kgTemperature() {
  if (TemperatureScheduler.IsTime()) {
    TemperatureScheduler.UpdateNextTime();
    // SHT40
    sensors_event_t humidity, temp;
    sht4.getEvent(&humidity, &temp);
    gTemperature = temp.temperature;
    gHumidity = humidity.relative_humidity;
    tN2kMsg N2kMsg;
    int instance = 2;
    debugLog("Temperature:  %6.02f °C", gTemperature);
    SetN2kTemperatureExt(N2kMsg, 0xff, instance, N2kts_InsideTemperature, CToKelvin(gTemperature));  // PGN 130316
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kgHumidity() {
  if (HumidityScheduler.IsTime()) {
    HumidityScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    int instance = 2;
    debugLog("Humidity:     %6.02f %% RH", gHumidity);
    SetN2kHumidity(N2kMsg, 0xff, instance, N2khs_InsideHumidity, gHumidity);  // PGN 130313
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kVaporAlarm() {
  if (VaporAlarmScheduler.IsTime()) {
    VaporAlarmScheduler.UpdateNextTime();
    // MQ-2 Vapor Sensor
    uptimeSec = esp_timer_get_time() / 1000 / 1000;
    if (uptimeSec < 300) {
      debugLog("Vapor: warming up MQ-2 sensor, uptime %.02f Sec", uptimeSec);
    } else {
      double vaporReading = analogRead(VAPOR_PIN);
      if (vaporReading >= 0 || vaporReading < 4096) {
        gVaporPercent = vaporReading * 100 / 4096;
      } else {
        debugLog("ERROR Invalid reading from Vapor: %.06f", vaporReading);
        gVaporPercent = NAN;
      }
    }
    tN2kOnOff vaporStatus = N2kOnOff_Unavailable;
    if (!isnan(gVaporPercent)) {
      if (gVaporPercent > 10) {
        debugLog("Vapor ALARM");
        vaporStatus = N2kOnOff_On;
      } else {
        vaporStatus = N2kOnOff_Off;
      }
    }
    debugLog("Vapor         %6.02f %%", gVaporPercent);
    tN2kMsg N2kMsg;
    SetN2kBinaryStatus(N2kMsg, 15, vaporStatus);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kEngineDynamicParam() {
  if (EngineDynamicScheduler.IsTime()) {
    EngineDynamicScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    double fluidSend = N2kDoubleNA;
    if (!isnan(gVaporPercent)) {
      fluidSend = gVaporPercent;
    }
    SetN2kFluidLevel(N2kMsg, 0, N2kft_FuelGasoline, fluidSend, N2kDoubleNA);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kAttitude() {
  if (AttitudeScheduler.IsTime()) {
    AttitudeScheduler.UpdateNextTime();
    // BNO055
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    debugLog("Yaw: %6.02f°    Pitch: %6.02f°    Roll: %6.02f°", orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z);
    tN2kMsg N2kMsg;
    SetN2kAttitude(N2kMsg, 0xff, orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z);  // PGN 127257
    NMEA2000.SendMsg(N2kMsg);
  }
}

void loop() {
  SendN2kgTemperature();
  SendN2kgHumidity();
  SendN2kBattery();
  SendN2kVaporAlarm();
  SendN2kEngineDynamicParam();
  SendN2kAttitude();
  NMEA2000.ParseMessages();

  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) {  // Save potentially changed Source Address to NVS memory
    debugLog("Address Change: Old Address=%d", NodeAddress);
    debugLog("                New Address=%d", SourceAddress);
    NodeAddress = SourceAddress;  // Set new Node Address (to save only once)
    writeIntToStorage("LastNodeAddress", NodeAddress);
  }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if (Serial.available()) {
    Serial.read();
  }
}
