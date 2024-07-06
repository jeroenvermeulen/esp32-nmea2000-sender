/*
 Inspired by https://github.com/AK-Homberger/NMEA2000-Data-Sender
*/

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4
#define ONE_WIRE_PIN GPIO_NUM_13     // Data wire for teperature (Dallas DS18B20)
#define VOLTAGE_PIN GPIO_NUM_35      // Voltage measure is connected GPIO 35 (Analog ADC1_CH7)
#define VAPOR_PIN GPIO_NUM_34        // Voltage measure is connected GPIO 34 (Analog ADC1_CH6)

#include <Arduino.h>
#include <Preferences.h>        // ESP32
#include <WiFi.h>               // ESP32
#include <Wire.h>               // ESP32
#include <esp_mac.h>            // ESP32
#include <NMEA2000_CAN.h>       // https://github.com/ttlappalainen/NMEA2000
#include <N2kMessages.h>        // https://github.com/ttlappalainen/NMEA2000
#include <DallasTemperature.h>  // Library: DallasTemperature by milesburton  https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <BME280I2C.h>          // Library: BME280 by Tyler Glenn https://github.com/finitespace/BME280/
//// also needed:
// mcp_can.h                         // https://github.com/ttlappalainen/CAN_BUS_Shield
// NMEA2000_mcp.h                    // https://github.com/ttlappalainen/NMEA2000_mcp

#define ENABLE_DEBUG_LOG 1           // Debug log on serial
#define NMEA_DEBUG_LOG 1             // NMEA message log on serial
#define ADC_Calibration_Value2 19.0  // The real value depends on the true resistor values for the ADC input (100K / 27 K).

int NodeAddress;          // To store last Node Address
Preferences preferences;  // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = { 130313L,  // Humidity
                                                   130315L,  // Pressure
                                                   130316L,  // Temperature extended range
                                                   127508L,  // Battery Status
                                                   127501L,  // Universal Binary Status Report
                                                   0 };

// Create schedulers, disabled at the beginning
//                                  enabled  period offset
tN2kSyncScheduler DallasTemperatureScheduler(false, 1500, 500);
tN2kSyncScheduler BmeClimateScheduler(false, 1500, 510);
tN2kSyncScheduler BmeTemperatureScheduler(false, 1500, 520);
tN2kSyncScheduler DCStatusScheduler(false, 1500, 540);
tN2kSyncScheduler VaporAlarmScheduler(false, 1500, 550);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_PIN);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature dallasSensor(&oneWire);
// BME280
BME280I2C bme;

// Global Data
float dallasTemp = 0.0;
float BatteryVolt = 0.0;

// Task handle for OneWire read (Core 0 on ESP32)
TaskHandle_t DallasTask;

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
  DallasTemperatureScheduler.UpdateNextTime();
  BmeClimateScheduler.UpdateNextTime();
  BmeTemperatureScheduler.UpdateNextTime();
  DCStatusScheduler.UpdateNextTime();
  VaporAlarmScheduler.UpdateNextTime();
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

void setup() {
  // Init USB serial port
  Serial.begin(115200);
  debugLog("");
  debugLog("Starting setup...");

  // Disable WiFi
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

  // Start OneWire
  dallasSensor.begin();

  Wire.begin();
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

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

  NodeAddress = readIntFromStorage("LastNodeAddress", 33);  // Read stored last NodeAddress, default 33

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();

  // Create task for core 0, loop() runs on core 1
  xTaskCreatePinnedToCore(
    GetTemperature, /* Function to implement the task */
    "DallasTask",   /* Name of the task */
    10000,          /* Stack size in words */
    NULL,           /* Task input parameter */
    0,              /* Priority of the task */
    &DallasTask,    /* Task handle. */
    0);             /* Core where the task should run */

  debugLog("Setup complete.");
  delay(200);
}

// This task runs isolated on core 0 because dallasSensor.requestTemperatures() is slow and blocking for about 750 ms
void GetTemperature(void* parameter) {
  float tmp = 0;
  for (;;) {
    dallasSensor.requestTemperatures();  // Send the command to get temperatures
    vTaskDelay(100);
    tmp = dallasSensor.getTempCByIndex(0);
    if (tmp != -127) dallasTemp = tmp;
    vTaskDelay(100);
  }
}

void SendN2kBattery(double BatteryVoltage) {
  if (DCStatusScheduler.IsTime()) {
    tN2kMsg N2kMsg;
    DCStatusScheduler.UpdateNextTime();
    if (0 >= BatteryVoltage) {
      debugDouble("ERROR Invalid voltage reading: %.02f", BatteryVoltage);
      BatteryVoltage = N2kDoubleNA;
    } else {
      debugDouble("Volt:         %4.02f V", BatteryVoltage);
    }
    SetN2kDCBatStatus(N2kMsg, 1, BatteryVoltage, N2kDoubleNA, N2kDoubleNA, 1);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kDallasTemperature() {
  if (DallasTemperatureScheduler.IsTime()) {
    DallasTemperatureScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;

    int instance = 0;
    debugDouble("Dallas Temp:  %4.02f °C", dallasTemp);
    SetN2kTemperatureExt(N2kMsg, 0xff, instance, N2kts_SeaTemperature, CToKelvin(dallasTemp));  // PGN 130316

    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kBmeClimate() {
  if (BmeClimateScheduler.IsTime()) {
    BmeClimateScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;

    float bmeTemp(NAN), bmeHum(NAN), bmePres(NAN);
    bme.read(bmePres, bmeTemp, bmeHum);

    int instance = 0;
    //debugDouble("BME Temp:     %4.02f °C", bmeTemp);
    //SetN2kTemperatureExt(N2kMsg, 0xff, instance, N2kts_InsideTemperature, CToKelvin(bmeTemp)); // PGN 130316

    debugDouble("BME Humidity: %4.02f %% RH", bmeHum);
    SetN2kHumidity(N2kMsg, 0xff, instance, N2khs_InsideHumidity, bmeHum); // PGN 130313

    // SignalK can't handle this somehow
    // debugDouble("BME Pressure: %4.02f hPa", bmePres);
    // SetN2kSetPressure(N2kMsg, 0xff, instance, N2kps_Atmospheric, hPAToPascal(bmePres)); // PGN 130315

    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kBmeTemperature() {
  if (BmeTemperatureScheduler.IsTime()) {
    BmeTemperatureScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;

    float bmeTemp(NAN), bmeHum(NAN), bmePres(NAN);
    bme.read(bmePres, bmeTemp, bmeHum);

    int instance = 0;
    debugDouble("BME Temp:     %4.02f °C", bmeTemp);
    SetN2kTemperatureExt(N2kMsg, 0xff, instance, N2kts_InsideTemperature, CToKelvin(bmeTemp)); // PGN 130316

    //debugDouble("BME Humidity: %4.02f %% RH", bmeHum);
    //SetN2kHumidity(N2kMsg, 0xff, instance, N2khs_InsideHumidity, bmeHum); // PGN 130313

    // SignalK can't handle this somehow
    // debugDouble("BME Pressure: %4.02f hPa", bmePres);
    // SetN2kSetPressure(N2kMsg, 0xff, instance, N2kps_Atmospheric, hPAToPascal(bmePres)); // PGN 130315

    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kVaporAlarm() {
  if (VaporAlarmScheduler.IsTime()) {
    VaporAlarmScheduler.UpdateNextTime();
    tN2kMsg N2kMsg;
    tN2kOnOff vaporStatus;

    double reading = analogRead(VAPOR_PIN);  // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
    if (reading < 0 || reading > 4095) {
      debugDouble("ERROR Invalid reading from Vapor: %.06f", reading);
      vaporStatus = N2kOnOff_Unavailable;
    } else {
      double vaporPercent = reading * 100 / 4096;
      debugDouble("Vapor         %4.02f %%", vaporPercent);
      double uptimeSec = esp_timer_get_time() / 1000 / 1000;
      if (uptimeSec < 3) {  // TODO 300
        debugDouble("Vapor: warming up MQ-2 sensor, uptime %.02f Sec", uptimeSec);
        vaporStatus = N2kOnOff_Unavailable;
      } else {
        if (vaporPercent > 1) {
          debugLog("Vapor ALARM");
          vaporStatus = N2kOnOff_On;
        } else {
          vaporStatus = N2kOnOff_Off;
        }
      }
    }
    SetN2kBinaryStatus(N2kMsg, 33, vaporStatus);
    NMEA2000.SendMsg(N2kMsg);
  }
}

// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
double ReadVoltage() {
  double reading = analogRead(VOLTAGE_PIN);  // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) {
    return 0;
  }
  // Polynomial https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function/blob/master/ESP32_ADC_Read_Voltage_Accurate.ino
  reading = (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1000;
  return reading;
}

void loop() {
  // Filter to eliminate spike for ADC readings
  BatteryVolt = ((BatteryVolt * 15) + (ReadVoltage() * ADC_Calibration_Value2 / 4096)) / 16;

  SendN2kDallasTemperature();
  SendN2kBmeClimate();
  SendN2kBmeTemperature();
  SendN2kBattery(BatteryVolt);
  SendN2kVaporAlarm();
  NMEA2000.ParseMessages();

  // int SourceAddress = NMEA2000.GetN2kSource();
  // if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
  //   NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
  //   writeIntToStorage("LastNodeAddress", SourceAddress);
  //   debugInt("Address Change: New Address=%d\n", NodeAddress);
  // }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if (Serial.available()) {
    Serial.read();
  }
}
