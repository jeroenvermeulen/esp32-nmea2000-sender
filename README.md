Inspired by https://github.com/AK-Homberger/NMEA2000-Data-Sender

Hardware:
- ESP32-WROOM-32, 4MB
- TJA1050 CAN Controller with [120K resistor removed](https://github.com/AK-Homberger/NMEA2000-Data-Sender/tree/master?tab=readme-ov-file#remove-the-120-ohm-resistor-from-the-transceiver)
- BNO055 9-axis IMU for heading, pitch and roll
- SHT40 temperature & humidity sensor I2C probe cable
- MQ-2 smoke & gas sensor on a sensor board providing analog output
- 3K3 + 1K8 resistors for scaling MQ-2 analog output 0-5V to 0-3.3V
- 100K + 27K resistors for measuring 12V bus voltage
- A yellow LED + 47 Ohm resistor
- 3.5A DC-DC Step-Down Voltage Regulator Board with 5V output
