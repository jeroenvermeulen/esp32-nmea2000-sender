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
  127501L,  // Universal Binary Status Report
  127505L,  // Fluid Level
  127508L,  // Battery Status
  130310L,  // Environmental Parameters - deprecated
  130313L,  // Humidity
  130315L,  // Pressure
  130316L,  // Temperature extended range
  // 60928
  0
};

// Create schedulers, disabled at the beginning
//                                  enabled  period offset
tN2kSyncScheduler BmeTemperatureScheduler(false, 1500, 510);
tN2kSyncScheduler BmeHumidityScheduler(false, 1500, 520);
tN2kSyncScheduler BmePressureScheduler(false, 1500, 530);
tN2kSyncScheduler EnvironmentScheduler(false, 1500, 540);
tN2kSyncScheduler DCStatusScheduler(false, 1500, 550);
tN2kSyncScheduler VaporAlarmScheduler(false, 1500, 560);
tN2kSyncScheduler EngineDynamicScheduler(false, 1500, 570);

// SHT40
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

// BNO055
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);


// Global Data
float busVoltage = NAN;
float bmeTemperature = NAN;
float bmeHumidity = NAN;
float bmePressure = NAN;
float bmeGas = NAN;
float vaporPercent = NAN;
unsigned long uptimeSec = 0;

// Task handle for OneWire read (Core 0 on ESP32)
TaskHandle_t readSensorsTask;

void debugLog(char* str) {
#if ENABLE_DEBUG_LOG == 1
  Serial.println(str);
#endif
}

void debugInt(char* str, int value) {
#if ENABLE_DEBUG_LOG == 1
  Serial.printf(str, value);
  Serial.println("");
#endif
}

void debugDouble(char* str, double value) {
#if ENABLE_DEBUG_LOG == 1
  Serial.printf(str, value);
  Serial.println("");
#endif
}

void OnN2kOpen() {
  debugLog("NMEA2000 is Open.");
  // Start schedulers now.
  BmeTemperatureScheduler.UpdateNextTime();
  BmeHumidityScheduler.UpdateNextTime();
  BmePressureScheduler.UpdateNextTime();
  EnvironmentScheduler.UpdateNextTime();
  DCStatusScheduler.UpdateNextTime();
  VaporAlarmScheduler.UpdateNextTime();
  EngineDynamicScheduler.UpdateNextTime();
}

int readIntFromStorage(const char* key, int defaultValue) {
  preferences.begin("nvs", false);
  int value = preferences.getInt(key, defaultValue);
  preferences.end();
#if ENABLE_DEBUG_LOG == 1
  Serial.printf("Received %s from storage: %d\n", key, value);
#endif
  return value;
}

void writeIntToStorage(const char* key, int value) {
  preferences.begin("nvs", false);
  preferences.putInt(key, value);
  preferences.end();
#if ENABLE_DEBUG_LOG == 1
  Serial.printf("Written %s to storage: %d\n", key, value);
#endif
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
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
}

void setupSHT40() {
  Serial.println("Adafruit SHT4x test");
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
    case SHT4X_HIGH_PRECISION:
      Serial.println("High precision");
      break;
    case SHT4X_MED_PRECISION:
      Serial.println("Med precision");
      break;
    case SHT4X_LOW_PRECISION:
      Serial.println("Low precision");
      break;
  }
  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_LOW_HEATER_100MS);
  switch (sht4.getHeater()) {
    case SHT4X_NO_HEATER:
      Serial.println("No heater");
      break;
    case SHT4X_HIGH_HEATER_1S:
      Serial.println("High heat for 1 second");
      break;
    case SHT4X_HIGH_HEATER_100MS:
      Serial.println("High heat for 0.1 second");
      break;
    case SHT4X_MED_HEATER_1S:
      Serial.println("Medium heat for 1 second");
      break;
    case SHT4X_MED_HEATER_100MS:
      Serial.println("Medium heat for 0.1 second");
      break;
    case SHT4X_LOW_HEATER_1S:
      Serial.println("Low heat for 1 second");
      break;
    case SHT4X_LOW_HEATER_100MS:
      Serial.println("Low heat for 0.1 second");
      break;
  }
}

void setupBNO055() {
  /* Initialize the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
}

void setup() {
  // Init USB serial port
  Serial.begin(115200);
  debugLog("");
  debugLog("Starting setup...");

  // Disable WiFi
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

  Wire.begin();
  i2cScan();

  // while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
  //   debugLog("-  Unable to find BME680. Trying again in 5 seconds.");
  //   delay(5000);
  // }  // of loop until device is located
  // BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  // BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  // BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  // Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  // BME680.setIIRFilter(IIR4);  // Use enumerated type values
  // Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  // BME680.setGas(320, 150);  // 320�c for 150 milliseconds

  setupSHT40();
  setupBNO055();

  // // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  // Set product information
  NMEA2000.SetProductInformation("1",                      // Manufacturer's Model serial code
                                 100,                      // Manufacturer's product code
                                 "ESP32 NMEA2000 Sender",  // Manufacturer's Model ID
                                 "0.0.2 (2024-07-04)",     // Manufacturer's Software version code
                                 "1.0.0 (2024-07-04)"      // Manufacturer's Model version
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

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);  // Show in clear text. Leave uncommented for default Actisense format.

  NodeAddress = readIntFromStorage("LastNodeAddress", 15);  // Read stored last NodeAddress

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();

  // Create task for core 0, loop() runs on core 1
  xTaskCreatePinnedToCore(
    ReadSensors,       /* Function to implement the task */
    "readSensorsTask", /* Name of the task */
    10000,             /* Stack size in words */
    NULL,              /* Task input parameter */
    0,                 /* Priority of the task */
    &readSensorsTask,  /* Task handle. */
    0);                /* Core where the task should run */

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

void ReadSensors(void* parameter) {
  for (;;) {
    vTaskDelay(100);

    sensors_event_t humidity, temp;

    uint32_t timestamp = millis();
    sht4.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
    timestamp = millis() - timestamp;
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degrees C");
    Serial.print("Humidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.println("% rH");

    Serial.print("Read duration (ms): ");
    Serial.println(timestamp);


    sensors_event_t event;
    bno.getEvent(&event);
    /* Display the floating point data */
    Serial.print("Yaw: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tPitch: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tRoll: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");


    // Voltage
    double voltageReading = ReadVoltage();
    if (isnan(voltageReading) || 0 >= voltageReading || 100 < voltageReading) {
      debugDouble("ERROR Invalid voltage reading: %.02f", voltageReading);
      busVoltage = NAN;
    } else {
      if (isnan(busVoltage)) {
        busVoltage = voltageReading;
      } else {
        // Filter to eliminate spike for ADC readings
        busVoltage = ((busVoltage * 15) + voltageReading) / 16;
      }
    }

    // Vapor Sensor
    uptimeSec = esp_timer_get_time() / 1000 / 1000;
    if (uptimeSec < 300) {
      debugDouble("Vapor: warming up MQ-2 sensor, uptime %.02f Sec", uptimeSec);
    } else {
      double vaporReading = analogRead(VAPOR_PIN);
      if (vaporReading >= 0 || vaporReading < 4096) {
        vaporPercent = vaporReading * 100 / 4096;
      } else {
        debugDouble("ERROR Invalid reading from Vapor: %.06f", vaporReading);
        vaporPercent = NAN;
      }
    }
  }
}

void SendN2kBattery() {
  if (DCStatusScheduler.IsTime()) {
    tN2kMsg N2kMsg;
    DCStatusScheduler.UpdateNextTime();
    double voltageSend = N2kDoubleNA;
    if (!isnan(busVoltage)) {
      voltageSend = busVoltage;
    }
    debugDouble("Volt:         %4.02f V", busVoltage);
    SetN2kDCBatStatus(N2kMsg, 1, voltageSend, N2kDoubleNA, N2kDoubleNA, 1);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kBmeHumidity() {
  if (BmeHumidityScheduler.IsTime()) {
    BmeHumidityScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    int instance = 2;
    debugDouble("BME Humidity: %4.02f %% RH", bmeHumidity);
    SetN2kHumidity(N2kMsg, 0xff, instance, N2khs_InsideHumidity, bmeHumidity);  // PGN 130313
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kBmeTemperature() {
  if (BmeTemperatureScheduler.IsTime()) {
    BmeTemperatureScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    int instance = 2;
    debugDouble("BME Temp:     %4.02f °C", bmeTemperature);
    SetN2kTemperatureExt(N2kMsg, 0xff, instance, N2kts_InsideTemperature, CToKelvin(bmeTemperature));  // PGN 130316
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kBmePressure() {
  if (BmePressureScheduler.IsTime()) {
    BmePressureScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    int instance = 2;
    debugDouble("BME Pressure: %4.02f hPa", bmePressure);
    SetN2kSetPressure(N2kMsg, 0xff, instance, N2kps_Atmospheric, hPAToPascal(bmePressure));  // PGN 130315
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kEnvironment() {
  if (EnvironmentScheduler.IsTime()) {
    EnvironmentScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    SetN2kOutsideEnvironmentalParameters(N2kMsg, 0xff, N2kDoubleNA, N2kDoubleNA, hPAToPascal(bmePressure));  // PGN 130310
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kVaporAlarm() {
  if (VaporAlarmScheduler.IsTime()) {
    VaporAlarmScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    tN2kOnOff vaporStatus = N2kOnOff_Unavailable;
    if (!isnan(vaporPercent)) {
      if (vaporPercent > 10) {
        debugLog("Vapor ALARM");
        vaporStatus = N2kOnOff_On;
      } else {
        vaporStatus = N2kOnOff_Off;
      }
    }
    debugDouble("Vapor         %4.02f %%", vaporPercent);
    SetN2kBinaryStatus(N2kMsg, 15, vaporStatus);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kEngineDynamicParam() {
  if (EngineDynamicScheduler.IsTime()) {
    EngineDynamicScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    double fluidSend = N2kDoubleNA;
    if (!isnan(vaporPercent)) {
      fluidSend = vaporPercent;
    }
    SetN2kFluidLevel(N2kMsg, 0, N2kft_FuelGasoline, fluidSend, N2kDoubleNA);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void loop() {
  SendN2kBmeTemperature();
  SendN2kBmeHumidity();
  SendN2kBmePressure();
  SendN2kEnvironment();
  SendN2kBattery();
  SendN2kVaporAlarm();
  SendN2kEngineDynamicParam();
  NMEA2000.ParseMessages();

  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) {  // Save potentially changed Source Address to NVS memory
    debugInt("Address Change: Old Address=%d", NodeAddress);
    debugInt("                New Address=%d", SourceAddress);
    NodeAddress = SourceAddress;  // Set new Node Address (to save only once)
    writeIntToStorage("LastNodeAddress", NodeAddress);
  }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if (Serial.available()) {
    Serial.read();
  }
}
