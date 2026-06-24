/*
  Configure the IMU for use with a GNSS receiver
  By: Nathan Seidle
  SparkFun Electronics
  Date: June 24, 2026
  License: This code is public domain but you buy me a drink if you use this
  and we meet someday (Drinkware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/sparkfun-9dof-imu-breakout-im19-qwiic.html

  This example demonstrates how to configure the IMU for use with a GNSS receiver.

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

    tiltSensor.enableDebugging();

    if (tiltSensor.begin(IMU) == false)
    {
        Serial.println("IM19 not detected. Please check wiring. Freezing...");
        while (1)
            ;
    }
    Serial.println("IM19 Detected");

    bool overallResult = true;
    int imuAppVersionInt = 0;

    if (tiltSensor.getAppVersion(imuAppVersionInt) == true)
    {
        Serial.printf("IM19 App Version: %d\r\n", imuAppVersionInt);
        char versionString[20] = {0};
        tiltSensor.getVersion(versionString, sizeof(versionString));
        Serial.printf("IM19 Version: %s\r\n", versionString);
    }

    // The filter has a set of default parameters, which can be loaded when setting an error.
    if (tiltSensor.sendCommand("LOAD_DEFAULT") == false)
    {
        Serial.println("Failed to load default parameters");
        overallResult = false;
    }

    // Use serial port 2 as the serial port for communication with GNSS
    if (tiltSensor.sendCommand("GNSS_PORT=PHYSICAL_UART2") == false)
    {
        Serial.println("Failed to set GNSS port");
        overallResult = false;
    }

    // Use serial port 1 as the main output with combined navigation data output
    if (tiltSensor.sendCommand("NAVI_OUTPUT=UART1,ON") == false)
    {
        Serial.println("Failed to set NAVI output");
        overallResult = false;
    }

    // The following commands take time to output their full response. Delay to allow the serial to arrive, and then be
    // flushed by the next sendCommand().

    // If defined, set the IMU installation angle - before LEVER_ARM2
    // "the AT+INSTALL_ANGLE command must be sent firstly"
    if (tiltSensor.sendCommand("INSTALL_ANGLE=0,180,0") == false)
    {
        Serial.println("Failed to set IMU installation angle");
        overallResult = false;
    }

    delay(25);

    // Set the LEVER_ARM(2) distance of the antenna ARP from the IMU
    if (tiltSensor.sendCommand("LEVER_ARM2=0.03391,0.00272,0.02370") == false)
    {
        Serial.println("Failed to set LEVER_ARM2");
        overallResult = false;
    }

    delay(25);

    // Set the overall length of the GNSS setup in meters: rod length 1800mm + internal length 96.45mm + antenna
    // POC 19.25mm = 1915.7mm
    if (tiltSensor.sendCommand("CLUB_VECTOR=0,0,1.858") == false)
    {
        Serial.println("Failed to set club vector");
        overallResult = false;
    }

    delay(25);

    // Configure interface type
    if (tiltSensor.sendCommand("GNSS_CARD=OEM") == false)
    {
        Serial.println("Failed to configure interface type");
        overallResult = false;
    }

    // Configure as tilt measurement mode
    if (tiltSensor.sendCommand("WORK_MODE=408") == false) // From stock firmware
    {
        Serial.println("Failed to configure tilt measurement mode");
        overallResult = false;
    }

    // AT+HIGH_RATE=[ENABLE | DISABLE] - try to slow down NAVI
    if (tiltSensor.sendCommand("HIGH_RATE=DISABLE") == false)
    {
        Serial.println("Failed to configure high rate");
        overallResult = false;
    }

    // Turn off MEMS output.
    // result &= tiltSensor.sendCommand("MEMS_OUTPUT=UART1,ON"); //Stock firmware enables MEMS
    if (tiltSensor.sendCommand("MEMS_OUTPUT=UART1,OFF") == false)
    {
        Serial.println("Failed to configure MEMS output");
        overallResult = false;
    }

    // The 'CORRECT_HOLDER' command is not supported on app versions 11.1 and later.
    // The command *is* supported on older 6.1 firmware. The command is not documented in the IM19 datasheet, but was
    // found in the Torch v2 example firmware.
    if (imuAppVersionInt == 610)
    {
        if (tiltSensor.sendCommand("CORRECT_HOLDER=ENABLE") == false)
        {
            Serial.println("Failed to enable correct holder");
            overallResult = false;
        }
    }

    // Trigger IMU on PPS from GNSS
    if (tiltSensor.sendCommand("SET_PPS_EDGE=RISING") == false)
    {
        Serial.println("Failed to configure PPS edge");
        overallResult = false;
    }

    // Enable magnetic field mode
    // 'it is recommended to use the magnetic field initialization mode to speed up the initialization process'
    if (tiltSensor.sendCommand("AHRS=ENABLE") == false)
    {
        Serial.println("Failed to enable magnetic field mode");
        overallResult = false;
    }

    if (tiltSensor.sendCommand("MAG_AUTO_SAVE=ENABLE") == false)
    {
        Serial.println("Failed to enable magnetic field auto save");
        overallResult = false;
    }

    if (overallResult == true)
    {
        if (tiltSensor.saveConfiguration() == true)
            Serial.println("Tilt sensor configuration complete");
    }
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
