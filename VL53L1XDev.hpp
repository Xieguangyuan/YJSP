#pragma once
#include "vl53l1_register_map.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <unistd.h>
#include <stdint.h>
#define byte uint8_t

class VL53L1XDevice
{
public:
    bool begin(char *i2cdev, uint8_t deviceAddress)
    {
        _deviceAddress = deviceAddress;
        char *filename = i2cdev;
        if ((i2c_port = open(filename, O_RDWR)) < 0)
            return false;
        int addr = _deviceAddress;
        if (ioctl(i2c_port, I2C_SLAVE, addr) < 0)
            return false;
        uint16_t modelID = readRegister16(VL53L1_IDENTIFICATION__MODEL_ID);
        std::cout << (int)modelID << "\n";
        if (modelID != 0xEACC)
            return (false);
        // softReset();
        // int counter = 0;
        // while (readRegister16(VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x01 == 0)
        // {
        //     if (counter++ == 100)
        //         return (false);
        //     usleep(10);
        // }
        // uint16_t result = readRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG);
        // result = (result & 0xFE) | 0x01;
        // writeRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, result);
        return (true);
    }

    bool Setmode(char mode)
    {
        switch (mode)
        {
        case 'S':
            // from VL53L1_preset_mode_standard_ranging_short_range()

            // timing config
            writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
            writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
            writeRegister(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

            // dynamic config
            writeRegister(VL53L1_SD_CONFIG__WOI_SD0, 0x07);
            writeRegister(VL53L1_SD_CONFIG__WOI_SD1, 0x05);
            writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
            writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

            break;

        case 'M':
            // from VL53L1_preset_mode_standard_ranging()

            // timing config
            writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
            writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
            writeRegister(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

            // dynamic config
            writeRegister(VL53L1_SD_CONFIG__WOI_SD0, 0x0B);
            writeRegister(VL53L1_SD_CONFIG__WOI_SD1, 0x09);
            writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
            writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

            break;

        case 'L': // long
            // from VL53L1_preset_mode_standard_ranging_long_range()

            // timing config
            writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
            writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
            writeRegister(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

            // dynamic config
            writeRegister(VL53L1_SD_CONFIG__WOI_SD0, 0x0F);
            writeRegister(VL53L1_SD_CONFIG__WOI_SD1, 0x0D);
            writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
            writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

            break;

        default:
            // unrecognized mode - do nothing
            return false;
        }
    }

    void startMeasurement(uint8_t offset)
    {
        unsigned int result;
        uint8_t address = 1 + offset; //Start at memory location 0x01, add offset

        //uint8_t leftToSend = sizeof(configBlock) - offset;
        int length = 137;
        char buffer[length] = {0};
        buffer[0] = 0x00;
        buffer[1] = 0x01;
        for (int x = 2; x < 135; x++)
        {
            buffer[x] = configBlock[x - 2];
            //  result = buffer[x];
            //cout << hex << result << endl;
        }
        buffer[135] = 0x01;
        buffer[136] = 0x40;
        for (byte x = 0; x < 137; x++)
        {
            result = buffer[x];
            // cout << hex << result << endl;
        }
        write(i2c_port, buffer, length);
    }

    bool newDataReady()
    {
        uint16_t result;
        result = readRegister(VL53L1_GPIO__TIO_HV_STATUS);
        if (readRegister(VL53L1_GPIO__TIO_HV_STATUS) != 0x03)
            return (true);
        return (false);
    }

    void softReset()
    {
        writeRegister(VL53L1_SOFT_RESET, 0x00);
        usleep(1);
        writeRegister(VL53L1_SOFT_RESET, 0x01);
    }

    uint16_t getDistance()
    {
        return (readRegister16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0));
    }

    uint16_t getSignalRate()
    {
        uint16_t reading = readRegister16(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0); // << 9; //FIXPOINT97TOFIXPOINT1616
        return (reading);
    }

    uint8_t getRangeStatus()
    {
#define VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE (1)
#define VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE (2)
#define VL53L1_DEVICEERROR_NOVHVVALUEFOUND (3)
#define VL53L1_DEVICEERROR_MSRCNOTARGET (4)
#define VL53L1_DEVICEERROR_RANGEPHASECHECK (5)
#define VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK (6)
#define VL53L1_DEVICEERROR_PHASECONSISTENCY (7)
#define VL53L1_DEVICEERROR_MINCLIP (8)
#define VL53L1_DEVICEERROR_RANGECOMPLETE (9)
#define VL53L1_DEVICEERROR_ALGOUNDERFLOW (10)
#define VL53L1_DEVICEERROR_ALGOOVERFLOW (11)
#define VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD (12)
#define VL53L1_DEVICEERROR_USERROICLIP (13)
#define VL53L1_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS (14)
#define VL53L1_DEVICEERROR_REFSPADCHARMORETHANTARGET (15)
#define VL53L1_DEVICEERROR_REFSPADCHARLESSTHANTARGET (16)
#define VL53L1_DEVICEERROR_MULTCLIPFAIL (17)
#define VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY (18)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK (19)
#define VL53L1_DEVICEERROR_EVENTCONSISTENCY (20)
#define VL53L1_DEVICEERROR_MINSIGNALEVENTCHECK (21)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE (22)

#define VL53L1_RANGESTATUS_RANGE_VALID 0                    /*!<The Range is valid. */
#define VL53L1_RANGESTATUS_SIGMA_FAIL 1                     /*!<Sigma Fail. */
#define VL53L1_RANGESTATUS_SIGNAL_FAIL 2                    /*!<Signal fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED 3  /*!<Target is below minimum detection threshold. */
#define VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL 4               /*!<Phase out of valid limits -  different to a wrap exit. */
#define VL53L1_RANGESTATUS_HARDWARE_FAIL 5                  /*!<Hardware fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL 6 /*!<The Range is valid but the wraparound check has not been done. */
#define VL53L1_RANGESTATUS_WRAP_TARGET_FAIL 7               /*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define VL53L1_RANGESTATUS_PROCESSING_FAIL 8                /*!<Internal algo underflow or overflow in lite ranging. */
#define VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL 9              /*!<Specific to lite ranging. */
#define VL53L1_RANGESTATUS_SYNCRONISATION_INT 10            /*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE 11      /*!<All Range ok but object is result of multiple pulses merging together.*/
#define VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL 12 /*!<Used  by RQL  as different to phase fail. */
#define VL53L1_RANGESTATUS_MIN_RANGE_FAIL 13                /*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define VL53L1_RANGESTATUS_RANGE_INVALID 14                 /*!<lld returned valid range but negative value ! */
#define VL53L1_RANGESTATUS_NONE 255                         /*!<No Update. */

        //Read status
        uint8_t measurementStatus = readRegister(VL53L1_RESULT__RANGE_STATUS) & 0x1F;

        //Convert status from one to another - From vl53l1_api.c
        switch (measurementStatus)
        {
        case VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY:
            measurementStatus = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
            break;
        case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
            measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
            break;
        case VL53L1_DEVICEERROR_RANGEPHASECHECK:
            measurementStatus = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
            break;
        case VL53L1_DEVICEERROR_MSRCNOTARGET:
            measurementStatus = VL53L1_RANGESTATUS_SIGNAL_FAIL;
            break;
        case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
            measurementStatus = VL53L1_RANGESTATUS_SIGMA_FAIL;
            break;
        case VL53L1_DEVICEERROR_PHASECONSISTENCY:
            measurementStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
            break;
        case VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD:
            measurementStatus = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
            break;
        case VL53L1_DEVICEERROR_MINCLIP:
            measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
            break;
        case VL53L1_DEVICEERROR_RANGECOMPLETE:
            measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID;
            break;
        default:
            measurementStatus = VL53L1_RANGESTATUS_NONE;
        }

        return measurementStatus;
    }

    ~VL53L1XDevice()
    {
        close(i2c_port);
    }

    uint8_t readRegister(uint16_t addr)
    {
        char buffer1[2] = {0};
        buffer1[0] = addr >> 8;
        buffer1[1] = addr & 0x00FF;
        int length1 = 2; //<<< Number of bytes to write
        write(i2c_port, buffer1, length1);

        unsigned char result;
        char buffer2[1] = {0};
        int length2 = 1; //<<< Number of bytes to read
        if (read(i2c_port, buffer2, length2) != length2)
        {
            //ERROR HANDLING: i2c transaction failed
            return (0);
        }
        else
        {
            result = buffer2[0];
        }
        return (result);
    }

    uint16_t readRegister16(uint16_t addr)
    {
        unsigned int result;
        char buffer1[2] = {0};
        buffer1[0] = addr >> 8;
        buffer1[1] = addr & 0x00FF;
        int length1 = 2; //<<< Number of bytes to write
        write(i2c_port, buffer1, length1);

        char buffer2[2] = {0};
        int length2 = 2; //<<< Number of bytes to read
        if (read(i2c_port, buffer2, length2) != length2)
        {
            //ERROR HANDLING: i2c transaction failed
            return (0);
        }
        else
        {
            result = buffer2[0] << 8 | buffer2[1];
        }
        return (result);
    }

    bool writeRegister(uint16_t addr, uint8_t val)
    {
        char buffer[3] = {0};
        buffer[0] = addr >> 8;
        buffer[1] = addr & 0x00FF;
        buffer[2] = val;
        int length = 3; //<<< Number of bytes to write

        if (write(i2c_port, buffer, length) != length)
        {
            return (0);
        }
        return (1); //All done!
    }

    bool writeRegister16(uint16_t addr, uint16_t val)
    {
        char buffer[4] = {0};
        buffer[0] = addr >> 8;
        buffer[1] = addr & 0x00FF;
        buffer[2] = val >> 8;
        buffer[3] = val & 0x00FF;
        int length = 4; //<<< Number of bytes to write

        if (write(i2c_port, buffer, length) != length)
        {
            return (0);
        }
        return (1); //All done!
    }

private:
    int i2c_port = -1;

    uint8_t _deviceAddress;

    const uint8_t configBlock[135] = {
        0x29, 0x02, 0x10, 0x00, 0x28, 0xBC, 0x7A, 0x81, //8
        0x80, 0x07, 0x95, 0x00, 0xED, 0xFF, 0xF7, 0xFD, //16
        0x9E, 0x0E, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, //24
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00, //32
        0x28, 0x00, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00, //40
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, //48
        0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, //56
        0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x02, //64
        0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, //72
        0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x02, 0x00, //80
        0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, //88
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91, 0x0F, //96
        0x00, 0xA5, 0x0D, 0x00, 0x80, 0x00, 0x0C, 0x08, //104
        0xB8, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x10, 0x00, //112
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, //120
        0x0D, 0x0E, 0x0E, 0x01, 0x00, 0x02, 0xC7, 0xFF, //128
        0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40        //129 - 135 (0x81 - 0x87)
    };
};