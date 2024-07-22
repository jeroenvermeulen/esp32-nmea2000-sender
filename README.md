Inspired by https://github.com/AK-Homberger/NMEA2000-Data-Sender

Hardware:
- ESP32-WROOM-32, 4MB
- TJA1050 CAN Controller with [120K resistor removed](https://github.com/AK-Homberger/NMEA2000-Data-Sender/tree/master?tab=readme-ov-file#remove-the-120-ohm-resistor-from-the-transceiver)
- BNO055 9-axis IMU for heading, pitch and roll
- SHT40 temperature & humidity sensor I2C probe cable
- MQ-2 smoke & gas sensor on a sensor board providing analog output
- 100K & 27K resistors for measuring 12V bus voltage
- A yellow LED + 48 Ohm resistor
- 3.5A DC-DC Step-Down Voltage Regulator Board with 5V output
