/*
  Check the version number of the IM19 IMU module
  By: Nathan Seidle
  SparkFun Electronics
  Date: June 24, 2026
  License: This code is public domain but you buy me a drink if you use this
  and we meet someday (Drinkware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/sparkfun-9dof-imu-breakout-im19-qwiic.html

  This example demonstrates how to read the version number of the IM19 IMU module.

  Hardware Connections:
  Connect the ESP32 pin 14 to the IMU RX pin and pin 17 to the IMU TX pin.
  Load this sketch
  Open output window at 115200bps
*/

#include "SparkFun_IM19_IMU_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_IM19_IMU

IM19 tiltSensor;

int pin_IMU_RX = 14;
int pin_IMU_TX = 17;

HardwareSerial IMU(1);

void setup()
{
    Serial.begin(115200);
    delay(250);

    Serial.println();
    Serial.println("IM19 Example - Version Check");

    IMU.begin(115200, SERIAL_8N1, pin_IMU_RX, pin_IMU_TX);

    if (tiltSensor.begin(IMU) == false)
    {
        Serial.println("IM19 not detected. Please check wiring. Freezing...");
        while (1)
            ;
    }
    Serial.println("IM19 Detected");

    char versionString[32] = {'\0'};
    bool gotVersion = tiltSensor.getVersion(versionString, sizeof(versionString));
    if (gotVersion)
    {
        Serial.print("Version string: ");
        Serial.println(versionString);
    }
    else
        Serial.println("Could not get version string.");

    int hardwareVersion = 0;
    bool gotHardware = tiltSensor.getHardwareVersion(hardwareVersion);
    if (gotHardware)
    {
        Serial.print("Hardware version: ");
        Serial.println(hardwareVersion);
    }
    else
        Serial.println("Could not parse hardware version.");

    int bootVersion = 0;
    bool gotBoot = tiltSensor.getBootVersion(bootVersion);
    if (gotBoot)
    {
        Serial.print("Bootloader version: ");
        Serial.println(bootVersion);
    }
    else
        Serial.println("Could not parse bootloader version.");

    int appVersion = 0;
    bool gotApp = tiltSensor.getAppVersion(appVersion);
    if (gotApp)
    {
        Serial.print("App version: ");
        Serial.println(appVersion);
    }
    else
        Serial.println("Could not parse app version.");
}

void loop()
{
    if (Serial.available())
    {
        char incoming = (char)Serial.read();
        if (incoming == 'r')
            ESP.restart();
    }
}
