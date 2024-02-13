/*
  This is a library to control IM19 GNSS tilt compensation sensors.

  https://github.com/sparkfun/SparkFun_IM19_IMU_Arduino_Library
  Best used with the UM980 Breakout: https://www.sparkfun.com/products/xxxxx

  Development environment specifics:
  Arduino IDE 1.8.x

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include "Arduino.h"
#include "SparkFun_IM19_IMU_Arduino_Library.h"

IM19_PARSE_STATE im19Parse = {IM19_PARSE_STATE_WAITING_FOR_SYNC1};

bool IM19::begin(HardwareSerial &serialPort)
{
    _hwSerialPort = &serialPort;
    _swSerialPort = nullptr;

    // We assume the user has started the serial port with proper pins and baud rate prior to calling begin()
    //_hwSerialPort->begin(115200);

    if (isConnected() == false)
    {
        return (false);
    }

    return (true);
}

// Query the device with 'MODE', expect OK response
// Device may be booting and outputting other messages (ie, $devicename,COM3*65)
// Try a few times
bool IM19::isConnected()
{
    for (int x = 0; x < 4; x++)
    {
        debugPrintf("Sending disable Navi command."); // Response to query should start with OK
        if (disableNaviUART1() == true)
            return (true);
        debugPrintf("Failed to connect. Trying again.");
        delay(1000 *
              (x + 1)); // If the device has just been reset, it takes a few seconds before it can respond to commands
    }
    return (false);
}

// Calling this function with nothing sets the debug port to Serial
// You can also call it with other streams like Serial1, SerialUSB, etc.
void IM19::enableDebugging(Print &debugPort)
{
    _debugPort = &debugPort;
}
void IM19::disableDebugging()
{
    _debugPort = nullptr;
}

// Enable printfs
// https://stackoverflow.com/questions/42131753/wrapper-for-printf
void IM19::debugPrintf(const char *format, ...)
{
    if (_debugPort == nullptr)
        return;

    va_list args;
    va_start(args, format);

    va_list args2;
    va_copy(args2, args);
    char buf[vsnprintf(nullptr, 0, format, args) + sizeof("\r\n")];

    vsnprintf(buf, sizeof buf, format, args2);

    // Add CR+LF
    buf[sizeof(buf) - 3] = '\r';
    buf[sizeof(buf) - 2] = '\n';
    buf[sizeof(buf) - 1] = '\0';

    _debugPort->write(buf, strlen(buf));

    va_end(args);
    va_end(args2);
}

// Scan serial until a given start character ('f', 'm', '\r', etc) is found
// Return if success or timeout
Im19Result IM19::scanForStart(uint8_t characterToFind, uint16_t maxWaitMs)
{
    unsigned long startTime = millis();

    while (1)
    {
        if (millis() - startTime > maxWaitMs)
        {
            debugPrintf("Timeout start byte");
            return (IM19_RESULT_TIMEOUT_START_BYTE);
        }

        if (serialAvailable())
        {
            uint8_t incoming = serialRead();
            if (incoming == characterToFind)
                return (IM19_RESULT_OK);
        }

        delay(1);
    }
    return (IM19_RESULT_ERROR); // We should never get here
}

uint16_t IM19::serialAvailable()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->available());
    }
    else if (_swSerialPort != nullptr)
    {
        return (_swSerialPort->available());
    }
    return (0);
}

uint8_t IM19::serialRead()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->read());
    }
    else if (_swSerialPort != nullptr)
    {
        return (_swSerialPort->read());
    }
    return (0);
}

uint8_t IM19::serialPeek()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->peek());
    }
    else if (_swSerialPort != nullptr)
    {
        return (_swSerialPort->peek());
    }
    return (0);
}

void IM19::serialPrintln(const char *command)
{
    if (_hwSerialPort != nullptr)
    {
        _hwSerialPort->println(command);
    }
    else if (_swSerialPort != nullptr)
    {
        _swSerialPort->println(command);
    }
}

// Discards any characters sitting in RX buffer
void IM19::clearBuffer()
{
    while (serialAvailable())
        serialRead();
}

uint32_t IM19::getNaviAge()
{
    return (millis() - lastUpdateNavi);
}
uint32_t IM19::getGnssAge()
{
    return (millis() - lastUpdateGnss);
}

// Send a command and verify OK response
// Return IM19_RESULT_OK if OK is found
// Return IM19_RESULT_SAVING if 'Saving...' is found
Im19Result IM19::sendString(const char *command, uint16_t maxWaitMs)
{
    clearBuffer();

    serialPrintln(command); // Sends \r\n

    uint8_t response[150];
    uint8_t maxResponseLength = sizeof(response);
    uint8_t responseLength = 0;
    Im19Result result;

    // Responses to commands: \r\nOK\r\n\r\n or ERROR
    result = scanForStart(im19ResponseStart, maxWaitMs);
    if (result != IM19_RESULT_OK)
        return (result);

    response[responseLength++] = im19ResponseStart;

    // debugPrintf("sendString response: %c", im19ResponseStart);

    unsigned long lastByteReceivedTime = millis();
    while (1)
    {
        if (millis() - lastByteReceivedTime > maxWaitMs)
        {
            debugPrintf("Timeout end byte");
            return (IM19_RESULT_TIMEOUT_END_BYTE);
        }

        if (serialAvailable())
        {
            byte incoming = serialRead();
            response[responseLength] = incoming;
            if (incoming == '\r') // Stop when second \r is found
                break;

            // Serial.write(incoming);

            responseLength++;
            if (responseLength == maxResponseLength - 1) // Leave room for terminator
            {
                debugPrintf("Response overflow");
                return (IM19_RESULT_RESPONSE_OVERFLOW);
            }
        }

        delay(1);
    }

    response[responseLength] = '\0'; // Terminate

    char *okPointer = strstr((char *)response, "OK");
    if (okPointer == nullptr)
    {
        // Look for special response for SAVE_ALL
        char *savingPointer = strstr((char *)response, "Saving...");
        if (savingPointer != nullptr)
        {
            return (IM19_RESULT_SAVING);
        }

        debugPrintf("OK not found in response (%d bytes): %s", responseLength, response);
        return (IM19_RESULT_COMMAND_ERROR); // Not found
    }

    return (IM19_RESULT_OK);
}

// Send a command and verify OK response
// Return true if OK is found
bool IM19::sendCommand(const char *command, uint16_t maxWaitMs)
{
    // Add AT+ prefix
    char fullCommand[50];
    snprintf(fullCommand, sizeof(fullCommand), "AT+%s", command);

    if (sendString(fullCommand, maxWaitMs) == IM19_RESULT_OK)
        return (true);
    return (false);
}

// AT+LOAD_DEFAULT - The filter has a set of default parameters, which can be loaded when setting an error.
bool IM19::factoryReset()
{
    return (sendCommand("LOAD_DEFAULT"));
}

// AT+SAVE_ALL
// Device reports 'Saving...' instead of OK so we have to handle it differently
bool IM19::saveConfiguration()
{
    // Add AT+ prefix
    char command[] = "SAVE_ALL";
    char fullCommand[50];
    snprintf(fullCommand, sizeof(fullCommand), "AT+%s", command);

    if (sendString(fullCommand, 500) == IM19_RESULT_SAVING)
        return (true);
    return (false);
}

// Enable Navi Output AT+NAVI_OUTPUT=UART1,ON
bool IM19::enableNaviUART1()
{
    return (setNavi("UART1", "ON"));
}
bool IM19::disableNaviUART1()
{
    return (setNavi("UART1", "OFF"));
}

bool IM19::enableGnssUART1()
{
    return (setGnss("UART1", "ON"));
}
bool IM19::disableGnssUART1()
{
    return (setGnss("UART1", "OFF"));
}

bool IM19::enableMemsUART1()
{
    return (setMems("UART1", "ON"));
}
bool IM19::disableMemsUART1()
{
    return (setMems("UART1", "OFF"));
}

bool IM19::enableNaviUART3()
{
    return (setNavi("UART3", "ON"));
}
bool IM19::disableNaviUART3()
{
    return (setNavi("UART3", "OFF"));
}

// AT+NAVI_OUTPUT=UART1,OFF
bool IM19::setNavi(const char *port, const char *setting)
{
    // Put command together
    char command[50];
    snprintf(command, sizeof(command), "NAVI_OUTPUT=%s,%s", port, setting);

    return (sendCommand(command));
}

// AT+GNSS_OUTPUT=UART1,OFF
bool IM19::setGnss(const char *port, const char *setting)
{
    // Put command together
    char command[50];
    snprintf(command, sizeof(command), "GNSS_OUTPUT=%s,%s", port, setting);

    return (sendCommand(command));
}

// AT+MEMS_OUTPUT=UART1,OFF
bool IM19::setMems(const char *port, const char *setting)
{
    // Put command together
    char command[50];
    snprintf(command, sizeof(command), "MEMS_OUTPUT=%s,%s", port, setting);

    return (sendCommand(command));
}

// Check if we have RTK Fix to allow us to proceed to next tilt step
// This function only works if GNSS_OUTPUT=UART1,ON
bool IM19::isRtkFixed()
{
    if (getGnssSolutionState() == 4) // RTK Fix
        return (true);
    return (false);
}

// isReady - Bits 20 (RTK data)/19 (Time sync)/18 (PPS received), ready for a shake to begin active tilt measurements
bool IM19::isReady()
{
    if (isRtkFixed() == false)
        return (false);

    if ((getNaviStatus() & (1 << 18)) && (getNaviStatus() & (1 << 19)) && (getNaviStatus() & (1 << 20)))
        return (true);
    return (false);
}

// isCorrecting - Bits 20 (RTK data)/19 (Time sync)/18 (PPS received)/17 (init complete) IMU is
// outputting modified lat/lon/alt
bool IM19::isCorrecting()
{
    if (isReady() == false)
        return (false);

    if ((getNaviStatus() & (1 << 17)) && (getNaviStatus() & (1 << 18)) && (getNaviStatus() & (1 << 19)) &&
        (getNaviStatus() & (1 << 20)))
        return (true);
    return (false);
}

// Given the start character to look for
// Polls serial port until timout, or response received
// Returns OK if response is received with valid checksum
// Response is stored in response[]
// maxResponseLength length is updated to contain the length of the response
Im19Result IM19::getFrame(uint8_t messageType, uint8_t *response, uint8_t *maxResponseLength, uint16_t maxWaitMs)
{
    int responseLength = 0;

    // Find start byte
    // Raw NAVI begins with 'fmin', 100 bytes, ends in 'ed'
    // Raw GNSS begins with 'fmig', 95 bytes, ends in 'ed'
    // Raw MEMS begins with 'fmim', 52 bytes, ends in 'ed'
    // A response starts is 'OK'

    uint8_t expectedLength;
    uint8_t offsetCheckSum;
    if (messageType == 'n')
    {
        expectedLength = 100;
        offsetCheckSum = offsetNaviCheckSum;
    }
    else if (messageType == 'g')
    {
        expectedLength = 95;
        offsetCheckSum = offsetGnssCheckSum;
    }
    //  else if (messageType == 'm')
    //    expectedLength = 52;
    else
    {
        debugPrintf("Unknown message type");
        return (IM19_RESULT_ERROR);
    }

    // Im19Result result = scanForStart(im19Sync1, maxWaitMs);
    // if (result != IM19_RESULT_OK)
    //     return (result);

    unsigned long startTime = millis();

    while (1)
    {
        if (millis() - startTime > maxWaitMs)
        {
            debugPrintf("Timeout start byte");
            return (IM19_RESULT_TIMEOUT_START_BYTE);
        }

        if (serialAvailable() > 3)
        {
            if (serialRead() == im19Sync1) // f
            {
                if (serialRead() == im19Sync2) // m
                {
                    if (serialRead() == im19Sync3) // i
                    {
                        uint8_t incoming = serialRead();
                        if (incoming == messageType) // g or n
                        {
                            // debugPrintf("Found start of message");
                            break;
                        }
                        else
                        {
                            debugPrintf("Discarding message: Expecting '%c' found '%c'", messageType, incoming);
                        }
                    }
                    else
                    {
                        // debugPrintf("Broke sync2");
                    }
                }
                else
                {
                    // debugPrintf("Broke sync");
                }
            }
        }

        delay(1);
    }

    response[responseLength++] = im19Sync1; // Store start byte so that offsets align
    response[responseLength++] = im19Sync2;
    response[responseLength++] = im19Sync3;
    response[responseLength++] = messageType;

    bool badFrame = false;

    // Read in framelength of data
    unsigned long lastByteReceivedTime = millis();
    while (1)
    {
        if (millis() - lastByteReceivedTime > maxWaitMs)
        {
            debugPrintf("Timeout frame length");
            return (IM19_RESULT_TIMEOUT_FRAME_LENGTH);
        }

        if (serialAvailable())
        {
            uint8_t incoming = serialRead();

            lastByteReceivedTime = millis();

            response[responseLength] = incoming;

            if (responseLength == offsetDataType)
            {
                if (incoming != messageType)
                {
                    // badFrame = true;
                    // debugPrintf("Wrong data type. Expecting '%c', received '%c'", messageType, incoming);
                    return (IM19_RESULT_WRONG_DATA_TYPE);
                }
            }

            responseLength++;

            if (responseLength == expectedLength)
                break; // Reached the end of the packet

            if (responseLength == *maxResponseLength)
            {
                debugPrintf("Response overflow");
                return (IM19_RESULT_RESPONSE_OVERFLOW);
            }
        }

        delay(1);
    }

    uint16_t sentenceCheckSum;
    memcpy(&sentenceCheckSum, &response[offsetCheckSum], sizeof(uint16_t));

    // Sum up every byte, including header and type, up to but not including check sum bytes.
    uint16_t calculatedCheckSum = 0;
    for (int x = 0; x < offsetCheckSum; x++)
        calculatedCheckSum += response[x];

    if (sentenceCheckSum != calculatedCheckSum)
    {
        debugPrintf("Checksum failed. Sentence checksum: 0x%02X Calculated checksum: 0x%02X\r\n", sentenceCheckSum,
                    calculatedCheckSum);
        return (IM19_RESULT_BAD_CHECKSUM);
    }

    *maxResponseLength = responseLength; // Update caller's copy.
    debugPrintf("Good frame");

    return (IM19_RESULT_OK);
}

// Main parsing
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Check for new data until there is no more
bool IM19::update()
{
    bool newData = false;
    while (serialAvailable())
        newData = updateOnce();
    return (newData);
}

// Checks for new data once
// Used during commands
bool IM19::updateOnce()
{
    if (serialAvailable())
    {
        uint8_t incoming = serialRead();

        // Move byte into parser
        im19Parse.buffer[im19Parse.length++] = incoming;
        im19Parse.length %= IM19_PARSE_BUFFER_LENGTH;

        //  Update the parser state based on the incoming byte
        switch (im19Parse.state)
        {
        default:
            debugPrintf("Case not found! : %d\r\n", im19Parse.state);
            // Drop to waiting for preamble
        case (IM19_PARSE_STATE_WAITING_FOR_SYNC1):
            waitForSync1(&im19Parse, incoming);
            break;
        case (IM19_PARSE_STATE_SYNC2):
            parseSync2(&im19Parse, incoming);
            break;
        case (IM19_PARSE_STATE_SYNC3):
            parseSync3(&im19Parse, incoming);
            break;
        case (IM19_PARSE_STATE_FRAME):
            parseType(&im19Parse, incoming);
            break;
        case (IM19_PARSE_STATE_DATA):
            parseData(&im19Parse, incoming);
            break;
        case (IM19_PARSE_STATE_CHECKSUM):
            parseChecksum(&im19Parse, incoming);
            break;
        case (IM19_PARSE_STATE_SYNC4):
            parseSync4(&im19Parse, incoming);
            break;
        case (IM19_PARSE_STATE_SYNC5):
            parseSync5(&im19Parse, incoming);
            break;
        }
        return (true);
    }
    return (false);
}

// Wait for the preamble byte
void IM19::waitForSync1(IM19_PARSE_STATE *parse, uint8_t data)
{
    //
    //    IM19 Message
    //
    //    +----------+---------+---------+----------+---------+
    //    | Preamble |  Type   |  Data   | Checksum |  Tail   |
    //    |  3 bytes | 1 byte  | n bytes | 2 bytes  | 2 bytes |
    //    |  'fmi'   |  n/g/m  |         |          |  'ed'   |
    //    +----------+---------+---------+----------+---------+
    //    |                              |
    //    |<--------- Checksum --------->|
    //    Sum up every byte, including header and type, up to but not including check sum or tail bytes.

    if (data == im19Sync1)
    {
        parse->checksum = data;
        parse->state = IM19_PARSE_STATE_SYNC2;
        return;
    }

    // Preamble byte not found
    parse->length = 0;
    parse->state = IM19_PARSE_STATE_WAITING_FOR_SYNC1;
}

// Read the second sync byte
void IM19::parseSync2(IM19_PARSE_STATE *parse, uint8_t data)
{
    // Verify sync byte 2
    if (data != im19Sync2)
    {
        // Invalid sync byte, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (waitForSync1(parse, data)); // Start searching for a preamble byte
    }

    parse->checksum += data;
    parse->state = IM19_PARSE_STATE_SYNC3; // Move on
}

// Read the third sync byte
void IM19::parseSync3(IM19_PARSE_STATE *parse, uint8_t data)
{
    // Verify sync byte 3
    if (data != im19Sync3)
    {
        // Invalid sync byte, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (waitForSync1(parse, data)); // Start searching for a preamble byte
    }

    parse->checksum += data;
    parse->state = IM19_PARSE_STATE_FRAME; // Move on
}

// Get frame type and info
void IM19::parseType(IM19_PARSE_STATE *parse, uint8_t data)
{
    switch (data)
    {
    default:
        debugPrintf("Unknown frame type");
        // Invalid, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (waitForSync1(parse, data)); // Start searching for a preamble byte
        break;

    case (im19MessageTypeNavi):
        parse->bytesRemaining = im19NaviLength - 4;
        break;
    case (im19MessageTypeGnss):
        parse->bytesRemaining = im19GnssLength - 4;
        break;
    case (im19MessageTypeMems):
        parse->bytesRemaining = im19MemsLength - 4;
        break;
    }

    parse->messageType = data;
    parse->checksum += data;
    parse->state = IM19_PARSE_STATE_DATA; // Move on
}

// Get frame type and info
void IM19::parseData(IM19_PARSE_STATE *parse, uint8_t data)
{
    // Account for this data byte
    parse->bytesRemaining -= 1;

    parse->checksum += data;

    // Wait until all the data is received, including the 4 byte CRC
    if (parse->bytesRemaining > 0)
        return;

    parse->bytesRemaining = 2;
    parse->state = IM19_PARSE_STATE_CHECKSUM; // Move on
}

// Get frame type and info
void IM19::parseChecksum(IM19_PARSE_STATE *parse, uint8_t data)
{
    // Account for this data byte
    parse->bytesRemaining -= 1;

    parse->statedChecksum >>= 8;
    parse->statedChecksum |= (data << 8);

    // Wait until all the data is received, including the 4 byte CRC
    if (parse->bytesRemaining > 0)
        return;

    parse->state = IM19_PARSE_STATE_SYNC4; // Move on
}

// Read the fourth sync byte
void IM19::parseSync4(IM19_PARSE_STATE *parse, uint8_t data)
{
    // Verify sync byte 4
    if (data != im19Sync4)
    {
        // Invalid sync byte, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (waitForSync1(parse, data)); // Start searching for a preamble byte
    }

    parse->state = IM19_PARSE_STATE_SYNC5; // Move on
}

// Read the final sync byte
void IM19::parseSync5(IM19_PARSE_STATE *parse, uint8_t data)
{
    // Verify sync byte 5
    if (data != im19Sync5)
    {
        // Invalid sync byte, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (waitForSync1(parse, data)); // Start searching for a preamble byte
    }

    if (parse->checksum != parse->statedChecksum)
    {
        debugPrintf("Bad checksum. Expected 0x%04X, calculated 0x%04X\r\n", parse->statedChecksum, parse->checksum);

        // Invalid checksum, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (waitForSync1(parse, data)); // Start searching for a preamble byte
    }

    eomHandler(parse); // Update data structs

    parse->length = 0;
    parse->state = IM19_PARSE_STATE_WAITING_FOR_SYNC1; // Move on
}

// Cracks a given binary message into the applicable container
void IM19::eomHandler(IM19_PARSE_STATE *parse)
{
    if (parse->messageType == im19MessageTypeNavi)
    {
        // debugPrintf("EOM NAVI handler");

        IM19_CHECK_POINTER_VOID(packetNavi, initNavi); // Check that RAM has been allocated

        lastUpdateNavi = millis(); // Update stale marker

        // Move data into given containers
        memcpy(&packetNavi->data.timestamp, &parse->buffer[offsetNaviTimestamp], sizeof(double));
        memcpy(&packetNavi->data.latitude, &parse->buffer[offsetNaviLatitude], sizeof(double));
        memcpy(&packetNavi->data.longitude, &parse->buffer[offsetNaviLongitude], sizeof(double));
        memcpy(&packetNavi->data.altitude, &parse->buffer[offsetNaviAltitude], sizeof(double));
        memcpy(&packetNavi->data.rollAngle, &parse->buffer[offsetNaviRollAngle], sizeof(float));
        memcpy(&packetNavi->data.pitchAngle, &parse->buffer[offsetNaviPitchAngle], sizeof(float));
        memcpy(&packetNavi->data.heading, &parse->buffer[offsetNaviHeading], sizeof(float));
        memcpy(&packetNavi->data.positionAccuracy, &parse->buffer[offsetNaviPositionAccuracy], sizeof(float));
        memcpy(&packetNavi->data.accelZeroBiasX, &parse->buffer[offsetNaviAccelZeroBiasX], sizeof(float));
        memcpy(&packetNavi->data.accelZeroBiasY, &parse->buffer[offsetNaviAccelZeroBiasY], sizeof(float));
        memcpy(&packetNavi->data.accelZeroBiasZ, &parse->buffer[offsetNaviAccelZeroBiasZ], sizeof(float));
        memcpy(&packetNavi->data.gyroZeroBiasX, &parse->buffer[offsetNaviGyroZeroBiasX], sizeof(float));
        memcpy(&packetNavi->data.gyroZeroBiasY, &parse->buffer[offsetNaviGyroZeroBiasY], sizeof(float));
        memcpy(&packetNavi->data.gyroZeroBiasZ, &parse->buffer[offsetNaviGyroZeroBiasZ], sizeof(float));
        memcpy(&packetNavi->data.temperature, &parse->buffer[offsetNaviTemperature], sizeof(float));
        memcpy(&packetNavi->data.status, &parse->buffer[offsetNaviStatus], sizeof(uint32_t));
    }
    else if (parse->messageType == im19MessageTypeGnss)
    {
        // debugPrintf("EOM GNSS handler");

        IM19_CHECK_POINTER_VOID(packetGnss, initGnss); // Check that RAM has been allocated

        lastUpdateGnss = millis(); // Update stale marker

        // Move data into given containers
        memcpy(&packetGnss->data.timestamp, &parse->buffer[offsetGnssTimestamp], sizeof(double));
        memcpy(&packetGnss->data.latitude, &parse->buffer[offsetGnssLatitude], sizeof(double));
        memcpy(&packetGnss->data.longitude, &parse->buffer[offsetGnssLongitude], sizeof(double));
        memcpy(&packetGnss->data.altitude, &parse->buffer[offsetGnssAltitude], sizeof(double));
        memcpy(&packetGnss->data.northboundSpeed, &parse->buffer[offsetGnssNorthboundSpeed], sizeof(float));
        memcpy(&packetGnss->data.eastboundSpeed, &parse->buffer[offsetGnssEastboundSpeed], sizeof(float));
        memcpy(&packetGnss->data.downwardSpeed, &parse->buffer[offsetGnssDownwardSpeed], sizeof(float));
        memcpy(&packetGnss->data.northernPositionVariance, &parse->buffer[offsetGnssNorthernPositionVariance],
               sizeof(float));
        memcpy(&packetGnss->data.easternPositionVariance, &parse->buffer[offsetGnssEasternPositionVariance],
               sizeof(float));
        memcpy(&packetGnss->data.elevationVariance, &parse->buffer[offsetGnssElevationVariance], sizeof(float));
        memcpy(&packetGnss->data.HRMS, &parse->buffer[offsetGnssHRMS], sizeof(float));
        memcpy(&packetGnss->data.VRMS, &parse->buffer[offsetGnssVRMS], sizeof(float));
        memcpy(&packetGnss->data.HDOP, &parse->buffer[offsetGnssHDOP], sizeof(float));
        memcpy(&packetGnss->data.VDOP, &parse->buffer[offsetGnssVDOP], sizeof(float));
        memcpy(&packetGnss->data.satellitesInView, &parse->buffer[offsetGnssSatellitesInView], sizeof(uint8_t));
        memcpy(&packetGnss->data.solutionState, &parse->buffer[offsetGnssSolutionState], sizeof(uint8_t));
        memcpy(&packetGnss->data.differentialAge, &parse->buffer[offsetGnssDifferentialAge], sizeof(uint8_t));
    }
    else
    {
        debugPrintf("Unknown message type: %d\r\n", parse->messageType);
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Allocate RAM for packet and initialize it
bool IM19::initNavi(uint8_t rate)
{
    packetNavi = new IM19_NAVI_t; // Allocate RAM for the struct
    if (packetNavi == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetNavi->callbackPointerPtr = nullptr;
    //   packetNavi->callbackData = nullptr;

    // Start outputting Navi in Binary on this COM port
    if (setNavi("UART1", "ON") == false)
    {
        debugPrintf("Failed to start NAVI output");
        delete packetNavi;
        packetNavi = nullptr;
        return (false);
    }

    // Wait until first report is available
    lastUpdateNavi = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateNavi > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("Tilt: Failed to get response from NAVI start");
            delete packetNavi;
            packetNavi = nullptr;
            return (false);
        }
    }

    return (true);
}

// Allocate RAM for packet and initialize it
bool IM19::initGnss(uint8_t rate)
{
    packetGnss = new IM19_GNSS_t; // Allocate RAM for the struct
    if (packetGnss == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetGnss->callbackPointerPtr = nullptr;
    //   packetGnss->callbackData = nullptr;

    // Start outputting Gnss in Binary on this COM port
    if (setGnss("UART1", "ON") == false)
    {
        debugPrintf("Tilt: Failed to start GNSS output");
        delete packetGnss;
        packetGnss = nullptr;
        return (false);
    }

    // Wait until first report is available
    lastUpdateGnss = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateGnss > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("Tilt: Failed to get response from GNSS start");
            delete packetGnss;
            packetGnss = nullptr;
            return (false);
        }
    }
    return (true);
}

double IM19::getNaviTimestamp()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.timestamp);
}
double IM19::getNaviLatitude()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.latitude);
}
double IM19::getNaviLongitude()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.longitude);
}
double IM19::getNaviAltitude()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.altitude);
}
float IM19::getNaviRollAngle()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.rollAngle);
}
float IM19::getNaviPitchAngle()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.pitchAngle);
}
float IM19::getNaviHeading()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.heading);
}

float IM19::getNaviPositionAccuracy()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.positionAccuracy);
}

float IM19::getNaviAccelZeroBiasX()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.accelZeroBiasX);
}
float IM19::getNaviAccelZeroBiasY()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.accelZeroBiasY);
}
float IM19::getNaviAccelZeroBiasZ()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.accelZeroBiasZ);
}
float IM19::getNaviGyroZeroBiasX()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.gyroZeroBiasX);
}
float IM19::getNaviGyroZeroBiasY()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.gyroZeroBiasY);
}
float IM19::getNaviGyroZeroBiasZ()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.gyroZeroBiasZ);
}
float IM19::getNaviTemperature()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.temperature);
}
uint32_t IM19::getNaviStatus()
{
    IM19_CHECK_POINTER_BOOL(packetNavi, initNavi); // Check that RAM has been allocated
    return (packetNavi->data.status);
}

double IM19::getGnssTimestamp()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.timestamp);
}
double IM19::getGnssLatitude()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.latitude);
}
double IM19::getGnssLongitude()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.longitude);
}
double IM19::getGnssAltitude()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.altitude);
}
float IM19::getGnssNorthboundSpeed()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.northboundSpeed);
}
float IM19::getGnssEastboundSpeed()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.eastboundSpeed);
}
float IM19::getGnssDownwardSpeed()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.downwardSpeed);
}
float IM19::getGnssNorthernPositionVariance()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.northernPositionVariance);
}
float IM19::getGnssEasternPositionVariance()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.easternPositionVariance);
}
float IM19::getGnssElevationVariance()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.elevationVariance);
}
float IM19::getGnssHRMS()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.HRMS);
}
float IM19::getGnssVRMS()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.VRMS);
}
float IM19::getGnssHDOP()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.HDOP);
}
float IM19::getGnssVDOP()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.VDOP);
}
uint8_t IM19::getGnssSatellitesInView()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.satellitesInView);
}
// 0 = No fix, 1 = 3D, 4 = RTK Fix
uint8_t IM19::getGnssSolutionState()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.solutionState);
}
uint8_t IM19::getGnssDifferentialAge()
{
    IM19_CHECK_POINTER_BOOL(packetGnss, initGnss); // Check that RAM has been allocated
    return (packetGnss->data.differentialAge);
}