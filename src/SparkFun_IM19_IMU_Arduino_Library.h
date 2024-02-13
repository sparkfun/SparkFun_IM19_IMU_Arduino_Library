/*
  This is a library to read/write to external I2C EEPROMs.
  It uses the same template system found in the Arduino
  EEPROM library so you can use the same get() and put() functions.

  https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library
  Best used with the Qwiic EEPROM: https://www.sparkfun.com/products/14764

  Various external EEPROMs have various interface specs
  (overall size, page size, write times, etc). This library works with
  all types and allows the various settings to be set at runtime.

  All read and write restrictions associated with pages are taken care of.
  You can access the external memory as if it was contiguous.

  Development environment specifics:
  Arduino IDE 1.8.x

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

*/

/*
  Commands covered in v1.1.10 doc
  AT+SYSTEM_RESET - Reset system
  AT+BOOT - Used for bootloading new firmware
  AT+SAVE_ALL - Save current config to NVM
  AT+GNSS_CARD=[HEMI | OEM | NOVTEL | UNICORE]
  AT+AHRS=[ENABLE | DISABLE]
  AT+ANT2=[ENABLE | DISABLE]
  AT+READ_PARA=[SYSTEM | ALL]
  AT+LOAD_DEFAULT
  AT+AUTO_FIX=[ENABLE | DISABLE]
  AT+MAG_AUTO_SAVE=[ENABLE | DISABLE]
  AT+GYR_AUTO_SAVE=[ENABLE | DISABLE]
  AT+ACC_AUTO_SAVE=[ENABLE | DISABLE]
  AT+CLUB_VECTOR=[VALUE1],[VALUE2],[VALUE3] - Pole length
  AT+NAVI_OUTPUT=[UART1 | UART 3], [ON | OFF] - Enable/disable tilt compensation
  AT+MEMS_OUTPUT=[UART1 | UART 3], [ON | OFF]
  AT+GNSS_OUTPUT=[UART1 | UART 3], [ON | OFF]
  AT+LEVER_ARM=X,Y,Z - Product specific location of the IMU.
  AT+CHECK_SYNC
  AT+HIGH_RATE=[ENABLE | DISABLE]
  AT+SET_ALL_PARA
  AT+ACTIVATE_KEY=[KEY]
  AT+ALIGN_VEL=1.0

  Not in document
  AT+GNSS_PORT=PHYSICAL_UART3   //Uses serial port 3 as the serial port for communication with GNSS
  AT+WORK_MODE=152  //Configured as tilt measurement mode
  AT+SET_PPS_EDGE=RISING/FALLING

  Typical config:
  AT+MAG_AUTO_SAVE=ENABLE
  AT+NAVI_OUTPUT=UART1,ON
  AT+LEVER_ARM=0.007,-0.035,-0.025
  AT+CLUB_VECTOR=0.00,0.00,1.855
  AT+GNSS_CARD=UNICORE
  AT+HIGH_RATE=DISABLE
  AT+SAVE_ALL
*/
#ifndef _SPARKFUN_IM19_IMU_ARDUINO_LIBRARY_H
#define _SPARKFUN_IM19_IMU_ARDUINO_LIBRARY_H

#include "Arduino.h"

#if __has_include("SoftwareSerial.h")
#include <SoftwareSerial.h>
#endif

#define IM19_CHECK_POINTER_BOOL(packetPointer, initPointer)                                                                 \
    {                                                                                                                  \
        if (packetPointer == nullptr)                                                                                  \
            initPointer();                                                                                             \
        if (packetPointer == nullptr)                                                                                  \
            return false;                                                                                              \
    }

#define IM19_CHECK_POINTER_VOID(packetPointer, initPointer)                                                                 \
    {                                                                                                                  \
        if (packetPointer == nullptr)                                                                                  \
            initPointer();                                                                                             \
        if (packetPointer == nullptr)                                                                                  \
            return;                                                                                                    \
    }


typedef enum
{
    IM19_RESULT_OK = 0,
    IM19_RESULT_SAVING,
    IM19_RESULT_TIMEOUT_START_BYTE,
    IM19_RESULT_TIMEOUT_DATA_BYTE,
    IM19_RESULT_TIMEOUT_FRAME_LENGTH,
    IM19_RESULT_WRONG_DATA_TYPE,
    IM19_RESULT_COMMAND_ERROR,
    IM19_RESULT_BAD_START_BYTE,
    IM19_RESULT_BAD_CHECKSUM,
    IM19_RESULT_BAD_CRC,
    IM19_RESULT_TIMEOUT,
    IM19_RESULT_TIMEOUT_END_BYTE,
    IM19_RESULT_RESPONSE_OVERFLOW,
    IM19_RESULT_ERROR,
} Im19Result;

enum
{
    IM19_PARSE_STATE_WAITING_FOR_SYNC1 = 0,
    IM19_PARSE_STATE_SYNC2,
    IM19_PARSE_STATE_SYNC3,
    IM19_PARSE_STATE_FRAME,
    IM19_PARSE_STATE_DATA,
    IM19_PARSE_STATE_CHECKSUM,
    IM19_PARSE_STATE_SYNC4,
    IM19_PARSE_STATE_SYNC5,
};

#define IM19_PARSE_BUFFER_LENGTH 110 // FMIN = 100 bytes

typedef struct _IM19_PARSE_STATE
{
    uint8_t state;
    uint8_t buffer[IM19_PARSE_BUFFER_LENGTH]; // Buffer containing the message
    uint16_t length;
    uint8_t messageType;                 // FMIN, FMIG, FMIM
    uint16_t bytesRemaining;
    uint16_t checksum;
    uint16_t statedChecksum;
} IM19_PARSE_STATE;

typedef struct
{
    double timestamp = 0;
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;
    float rollAngle = 0;
    float pitchAngle = 0;
    float heading = 0;
    float positionAccuracy = 0;
    float accelZeroBiasX = 0;
    float accelZeroBiasY = 0;
    float accelZeroBiasZ = 0;
    float gyroZeroBiasX = 0;
    float gyroZeroBiasY = 0;
    float gyroZeroBiasZ = 0;
    float temperature = 0;
    uint32_t status = 0;
} IM19_NAVI_data_t;

typedef struct
{
    IM19_NAVI_data_t data;
    //void (*callbackPointerPtr)(IM19_NAVI_data_t *);
    //IM19_NAVI_data_t *callbackData;
} IM19_NAVI_t;

typedef struct
{
    double timestamp = 0;
    double latitude = 0;
    double longitude = 0;
    double altitude = 0;
    float northboundSpeed = 0;
    float eastboundSpeed = 0;
    float downwardSpeed = 0;
    float northernPositionVariance = 0;
    float easternPositionVariance = 0;
    float elevationVariance = 0;
    float HRMS = 0;
    float VRMS = 0;
    float HDOP = 0;
    float VDOP = 0;
    uint8_t satellitesInView = 0;
    uint8_t solutionState = 0;
    uint8_t differentialAge = 0;
} IM19_GNSS_data_t;

typedef struct
{
    IM19_GNSS_data_t data;
    //void (*callbackPointerPtr)(IM19_GNSS_data_t *);
    //IM19_GNSS_data_t *callbackData;
} IM19_GNSS_t;


const uint8_t im19Sync1 = 'f';
const uint8_t im19Sync2 = 'm';
const uint8_t im19Sync3 = 'i';
const uint8_t im19Sync4 = 'e';
const uint8_t im19Sync5 = 'd';
const uint8_t im19ResponseStart = '\r'; // Command responses are \r\nOK\r\n\r\n or ERROR
const uint8_t im19MessageTypeNavi = 'n';
const uint8_t im19MessageTypeGnss = 'g';
const uint8_t im19MessageTypeMems = 'm';

const uint8_t offsetSyncA = 0; //'f'
const uint8_t offsetSyncB = 1; //'m'
const uint8_t offsetSyncC = 2; //'i'
const uint8_t offsetDataType = 3;

const uint8_t offsetNaviTimestamp = 4;
const uint8_t offsetNaviLatitude = 12;
const uint8_t offsetNaviLongitude = 20;
const uint8_t offsetNaviAltitude = 28;
const uint8_t offsetNaviNorthboundSpeed = 36;
const uint8_t offsetNaviEastboundSpeed = 40;
const uint8_t offsetNaviDownwardSpeed = 44;
const uint8_t offsetNaviRollAngle = 48;
const uint8_t offsetNaviPitchAngle = 52;
const uint8_t offsetNaviHeading = 56;
const uint8_t offsetNaviPositionAccuracy = 56;
const uint8_t offsetNaviAccelZeroBiasX = 64;
const uint8_t offsetNaviAccelZeroBiasY = 68;
const uint8_t offsetNaviAccelZeroBiasZ = 72;
const uint8_t offsetNaviGyroZeroBiasX = 76;
const uint8_t offsetNaviGyroZeroBiasY = 80;
const uint8_t offsetNaviGyroZeroBiasZ = 84;
const uint8_t offsetNaviTemperature = 88;
const uint8_t offsetNaviStatus = 92;
const uint8_t offsetNaviCheckSum = 96;
const uint8_t im19NaviLength = 96;

const uint8_t offsetGnssTimestamp = 4;
const uint8_t offsetGnssLatitude = 12;
const uint8_t offsetGnssLongitude = 20;
const uint8_t offsetGnssAltitude = 28;
const uint8_t offsetGnssNorthboundSpeed = 36;
const uint8_t offsetGnssEastboundSpeed = 40;
const uint8_t offsetGnssDownwardSpeed = 44;
const uint8_t offsetGnssNorthernPositionVariance = 48;
const uint8_t offsetGnssEasternPositionVariance = 52;
const uint8_t offsetGnssElevationVariance = 56;
const uint8_t offsetGnssHRMS = 72;
const uint8_t offsetGnssVRMS = 76;
const uint8_t offsetGnssHDOP = 80;
const uint8_t offsetGnssVDOP = 84;
const uint8_t offsetGnssSatellitesInView = 88;
const uint8_t offsetGnssSolutionState = 89;
const uint8_t offsetGnssDifferentialAge = 90;
const uint8_t offsetGnssCheckSum = 91;
const uint8_t im19GnssLength = 91;

const uint8_t offsetMemsCheckSum = 48;
const uint8_t im19MemsLength = 48;

class IM19
{
  private:
    unsigned long lastUpdateNavi = 0;
    unsigned long lastUpdateGnss = 0;

    // void sendSerialHw(uint8_t commandLength);
    // void sendSerialSw(uint8_t commandLength);

    Print *_debugPort = nullptr; // The stream to send debug messages to if enabled. Usually Serial.

    void debugPrintf(const char *format, ...);

  protected:
    HardwareSerial *_hwSerialPort = nullptr;
#if __has_include("SoftwareSerial.h")
    SoftwareSerial *_swSerialPort = nullptr;
#else
    HardwareSerial *_swSerialPort = nullptr;
#endif

  public:
    bool begin(HardwareSerial &serialPort);
    bool isConnected();

    void enableDebugging(Print &debugPort = Serial);
    void disableDebugging();

    Im19Result getFrame(uint8_t frameType, uint8_t *response, uint8_t *maxResponseLength, uint16_t maxWaitMs = 500);
    Im19Result scanForStart(uint8_t characterToFind, uint16_t maxWaitMs);
    Im19Result sendString(const char *command, uint16_t maxWaitMs = 500);
    bool sendCommand(const char *command, uint16_t maxWaitMs = 500);

    uint16_t serialAvailable();
    uint8_t serialRead();
    uint8_t serialPeek();
    void serialPrintln(const char *command);
    void clearBuffer();

    bool update();
    bool updateOnce();
    void waitForSync1(IM19_PARSE_STATE *parse, uint8_t data);
    void parseSync2(IM19_PARSE_STATE *parse, uint8_t data);
    void parseSync3(IM19_PARSE_STATE *parse, uint8_t data);
    void parseType(IM19_PARSE_STATE *parse, uint8_t data);
    void parseData(IM19_PARSE_STATE *parse, uint8_t data);
    void parseChecksum(IM19_PARSE_STATE *parse, uint8_t data);
    void parseSync4(IM19_PARSE_STATE *parse, uint8_t data);
    void parseSync5(IM19_PARSE_STATE *parse, uint8_t data);
    void eomHandler(IM19_PARSE_STATE *parse);

    bool initNavi(uint8_t rate = 5);
    IM19_NAVI_t *packetNavi = nullptr;

    bool initGnss(uint8_t rate = 5);
    IM19_GNSS_t *packetGnss = nullptr;

    uint16_t getNaviFreshLimitMs();
    void setNaviFreshLimitMs(uint16_t staleAmountMs);
    uint32_t getNaviAge();
    uint16_t getGnssFreshLimitMs();
    void setGnssFreshLimitMs(uint16_t staleAmountMs);
    uint32_t getGnssAge();

    bool factoryReset();
    bool saveConfiguration();

    // Main helper functions
    bool setNavi(const char *port, const char *setting);
    bool setGnss(const char *port, const char *setting);
    bool setMems(const char *port, const char *setting);

    bool enableNaviUART1();
    bool disableNaviUART1();
    bool enableGnssUART1();
    bool disableGnssUART1();
    bool enableMemsUART1();
    bool disableMemsUART1();

    bool enableNaviUART3();
    bool disableNaviUART3();

    double getNaviTimestamp();
    double getNaviLatitude();
    double getNaviLongitude();
    double getNaviAltitude();
    float getNaviRollAngle();
    float getNaviPitchAngle();
    float getNaviHeading();
    float getNaviPositionAccuracy();
    float getNaviAccelZeroBiasX();
    float getNaviAccelZeroBiasY();
    float getNaviAccelZeroBiasZ();
    float getNaviGyroZeroBiasX();
    float getNaviGyroZeroBiasY();
    float getNaviGyroZeroBiasZ();
    float getNaviTemperature();
    uint32_t getNaviStatus();

    double getGnssTimestamp();
    double getGnssLatitude();
    double getGnssLongitude();
    double getGnssAltitude();
    float getGnssNorthboundSpeed();
    float getGnssEastboundSpeed();
    float getGnssDownwardSpeed();
    float getGnssNorthernPositionVariance();
    float getGnssEasternPositionVariance();
    float getGnssElevationVariance();
    float getGnssHRMS();
    float getGnssVRMS();
    float getGnssHDOP();
    float getGnssVDOP();
    uint8_t getGnssSatellitesInView();
    uint8_t getGnssSolutionState();
    uint8_t getGnssDifferentialAge();

    bool isRtkFixed(); //Check is GNSS is reporting as fixed. Requires "GNSS_OUTPUT=UART1,ON" during config.
    bool isReady(); //IMU is online and ready for a shake, but is not outputting new lat/lon/alt
    bool isCorrecting(); //IMU has been shaken and is outputting modified lat/lon/alt
    //bool isTiltError(); //Returns true when 
};

#endif //_SPARKFUN_IM19_IMU_ARDUINO_LIBRARY_H
