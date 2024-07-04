/*
 Inspired by https://github.com/AK-Homberger/NMEA2000-Data-Sender
*/

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4
#define ONE_WIRE_PIN GPIO_NUM_13     // Data wire for teperature (Dallas DS18B20)
#define VOLTAGE_PIN GPIO_NUM_35      // Voltage measure is connected GPIO 35 (Analog ADC1_CH7)

#include <Arduino.h>
#include <Preferences.h>             // ESP32
#include <WiFi.h>                    // ESP32
#include "esp_mac.h"                 // ESP32
#include "NMEA2000_CAN.h"            // https://github.com/ttlappalainen/NMEA2000
#include "N2kMessages.h"             // https://github.com/ttlappalainen/NMEA2000
#include "DallasTemperature.h"       // From Library Manager: by milesburton  https://github.com/milesburton/Arduino-Temperature-Control-Library
//// also needed:
// mcp_can.h                         // https://github.com/ttlappalainen/CAN_BUS_Shield
// NMEA2000_mcp.h                    // https://github.com/ttlappalainen/NMEA2000_mcp

#define ENABLE_DEBUG_LOG 1           // Debug log on serial
#define NMEA_DEBUG_LOG 1             // NMEA message log on serial
#define ADC_Calibration_Value2 19.0  // The real value depends on the true resistor values for the ADC input (100K / 27 K).

int NodeAddress;                     // To store last Node Address
Preferences preferences;             // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {130316L, // Temperature
                                                  127508L, // Battery Status
                                                  0
                                                 };

// Create schedulers, disabled at the beginning
//                                  enabled  period offset
tN2kSyncScheduler TemperatureScheduler(false, 1500,  500);
tN2kSyncScheduler DCStatusScheduler(false, 1500,  510);

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_PIN);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature dallasSensor(&oneWire);

// Send time offsets
#define TempSendOffset 0
#define BatterySendOffset 100
#define SlowDataUpdatePeriod 1000  // Time between CAN Messages sent

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
  TemperatureScheduler.UpdateNextTime();
  DCStatusScheduler.UpdateNextTime();
}

int readIntFromStorage(const char* key, int defaultValue) {
  preferences.begin("nvs", false);
  int value = preferences.getInt(key, defaultValue);
  preferences.end();
#if ENABLE_DEBUG_LOG == 1
  Serial.printf("Received %s from storage: %d", key, value);
#endif
  return value;
}

void writeIntToStorage(const char* key, int value) {
  preferences.begin("nvs", false);
  preferences.putInt(key, value);
  preferences.end();
#if ENABLE_DEBUG_LOG == 1
  Serial.printf("Written %s to storage: %d", key, value);
#endif
}

unsigned long getUniqueID() {
    uint8_t chipid[6];
    unsigned long id = 0;
    esp_efuse_mac_get_default(chipid);
    for (int i = 0; i < 6; i++) id += (chipid[i] << (7 * i));
    return id;
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

  // // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  // Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "ESP32 NMEA2000 Sender",  // Manufacturer's Model ID
                                 "0.0.2 (2024-07-04)",  // Manufacturer's Software version code
                                 "1.0.0 (2024-07-04)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(getUniqueID(), // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25,  // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

#if NMEA_DEBUG_LOG == 1
  NMEA2000.SetForwardStream(&Serial);
#endif

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  NodeAddress = readIntFromStorage("LastNodeAddress", 33);  // Read stored last NodeAddress, default 33
  debugInt("NodeAddress: %d", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();

  // Create task for core 0, loop() runs on core 1
  xTaskCreatePinnedToCore(
    GetTemperature, /* Function to implement the task */
    "DallasTask", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &DallasTask,  /* Task handle. */
    0); /* Core where the task should run */

  debugLog("Setup complete.");
  delay(200);
}

// This task runs isolated on core 0 because dallasSensor.requestTemperatures() is slow and blocking for about 750 ms
void GetTemperature( void * parameter) {
  float tmp = 0;
  for (;;) {
    dallasSensor.requestTemperatures(); // Send the command to get temperatures
    vTaskDelay(100);
    tmp = dallasSensor.getTempCByIndex(0);
    if (tmp != -127) dallasTemp = tmp;
    vTaskDelay(100);
  }
}

void SendN2kBattery(double BatteryVoltage) {
  tN2kMsg N2kMsg;
  if ( DCStatusScheduler.IsTime() ) {
    DCStatusScheduler.UpdateNextTime();
    debugDouble("Volt: %3.02f V", BatteryVoltage);
    SetN2kDCBatStatus(N2kMsg, 0, BatteryVoltage, N2kDoubleNA, N2kDoubleNA, 1);
    NMEA2000.SendMsg(N2kMsg);
  }
}

void SendN2kTemperature(double temp) {
  tN2kMsg N2kMsg;
  if ( TemperatureScheduler.IsTime() ) {
    TemperatureScheduler.UpdateNextTime();
    debugDouble("Temp: %3.02f °C", temp);
    SetN2kTemperatureExt(N2kMsg, 0, 0, N2kts_InsideTemperature, CToKelvin(temp), N2kDoubleNA); // PGN130316
    NMEA2000.SendMsg(N2kMsg);
  }
}

// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
double ReadVoltage(byte pin) {
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  // polynomial
  reading = (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1000;
  return reading;
}

void loop() {
  // Filter to eliminate spike for ADC readings
  BatteryVolt = ((BatteryVolt * 15) + (ReadVoltage(VOLTAGE_PIN) * ADC_Calibration_Value2 / 4096)) / 16;

  SendN2kTemperature(dallasTemp);
  SendN2kBattery(BatteryVolt);
  NMEA2000.ParseMessages();

  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    writeIntToStorage("LastNodeAddress", SourceAddress);
    debugInt("Address Change: New Address=%d\n", NodeAddress);
  }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) {
    Serial.read();
  }
}
