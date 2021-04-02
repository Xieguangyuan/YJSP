#include "YJSP_Schedule.hpp"
int YJSP_AP::YJSP::YJSP_Init()
{
    wiringPiSetup();
    DF.fd = pca9685Setup(65, 0x40, 50);
    DF.myIbusDevice = new Ibus(DF.RCDeviceInfo);
    DF.MPUDevice = new RPiMPU9250(1, false, 1, 0x68, TF.TimeMax, 0);
    SF.AccelCaliData[MPUAccelCaliX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Cali");
    SF.AccelCaliData[MPUAccelCaliY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Cali");
    SF.AccelCaliData[MPUAccelCaliZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Cali");
    SF.AccelCaliData[MPUAccelScalX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Scal");
    SF.AccelCaliData[MPUAccelScalY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Scal");
    SF.AccelCaliData[MPUAccelScalZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Scal");
    PF.PIDYawPGain = configSettle("../MPU9250Car.json", "_Car_PIDYawPGain");
    PF.PIDYawIGain = configSettle("../MPU9250Car.json", "_Car_PIDYawIGain");
    PF.PIDYawDGain = configSettle("../MPU9250Car.json", "_Car_PIDYawDGain");
    DF.MPUDevice->MPUCalibration(SF.AccelCaliData);
    DF.MPUDevice->MPUSensorsDataGet();
    DF.MPUDevice->ResetMPUMixAngle();
    RF.RCForwardMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIDDLE");
    RF.RCForwardMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIN");
    RF.RCForwardMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MAX");
    RF.RCHorizontalMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIDDLE");
    RF.RCHorizontalMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIN");
    RF.RCHorizontalMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MAX");
    RF.RCYawMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIDDLE");
    RF.RCYawMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIN");
    RF.RCYawMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MAX");
}

void YJSP_AP::YJSP::MPUThreadREG()
{
    TF.MPUThreading = std::thread([&] {
        while (true)
        {
            TF.TimeStart = micros();
            TF.TimeNext = TF.TimeStart - TF.TimeEnd;
            SF.myData = DF.MPUDevice->MPUSensorsDataGet();
            SF.Real_Yaw += ((float)SF.myData._uORB_MPU9250_G_Z / 65.5) / (float)TF.TimeMax;
            EF.SPEED_X = EF.SPEED_X + (SF.myData._uORB_Acceleration_X * 0.001);
            EF.SPEED_Y = EF.SPEED_Y + (SF.myData._uORB_Acceleration_Y * 0.001);

            {
                float YawInput;
                if (RF.RC_Auto)
                {
                    RF.TotalForward = RF.Auto_Forward;
                    RF.TotalHorizontal = RF.Auto_Horizontal;
                    YawInput = (RF.Auto_Yaw + SF.myData._uORB_Gryo___Yaw * 5) + SF.Real_Yaw * 15.f - RF.InputYaw * 15.f;
                }
                else
                {
                    RF.TotalForward = RF.RCForward;
                    RF.TotalHorizontal = RF.RCHorizontal;
                    // RF.TotalYaw = RF.RCYaw;
                    YawInput = (RF.RCYaw + SF.myData._uORB_Gryo___Yaw * 5);
                }

                YawInput = YawInput > 500.f ? 500.f : YawInput;
                PF.TotalYawIFilter += (YawInput - PF.TotalYawIFilter) * 0.8;
                PF.TotalYawDFilter += (YawInput - PF.TotalYawDFilter) * 0.915;
                PIDCacl(YawInput, PF.TotalYawIFilter, PF.TotalYawDFilter, RF.TotalYaw, PF.PIDYawLastIData,
                        PF.PIDYawLastDData, PF.PIDYawPGain, PF.PIDYawIGain, PF.PIDYawDGain, 300.f);
            }

            TF.TimeEnd = micros();
            if (TF.TimeMax < ((TF.TimeEnd - TF.TimeStart) + TF.TimeNext) || (TF.TimeNext) < 0)
                usleep(50);
            else
                usleep(TF.TimeMax - (TF.TimeEnd - TF.TimeStart) - TF.TimeNext);
            TF.TimeEnd = micros();
        }
    });
}

void YJSP_AP::YJSP::UserInput(int Forward, int Horizontal, int Yaw, int Input_Yaw)
{
    RF.Auto_Forward = Forward;
    RF.Auto_Horizontal = Horizontal;
    RF.Auto_Yaw = Yaw;
    RF.InputYaw = Input_Yaw;
}

void YJSP_AP::YJSP::RCThreadREG()
{
    TF.RCThreading = std::thread([&] {
        while (true)
        {
            int lose = DF.myIbusDevice->IbusRead(RF.IbusData, 4000, 2);
            if (RF.IbusData[0] < RF.RCHorizontalMiddle + 10 && RF.IbusData[0] > RF.RCHorizontalMiddle - 10)
                RF.RCHorizontal = 0;
            else
                RF.RCHorizontal = RF.IbusData[0] - RF.RCHorizontalMiddle;

            if (RF.IbusData[1] < RF.RCForwardMiddle + 10 && RF.IbusData[1] > RF.RCForwardMiddle - 10)
                RF.RCForward = 0;
            else
                RF.RCForward = RF.IbusData[1] - RF.RCForwardMiddle;

            if (RF.IbusData[3] < RF.RCYawMiddle + 10 && RF.IbusData[3] > RF.RCYawMiddle - 10)
                RF.RCYaw = 0;
            else
                RF.RCYaw = RF.IbusData[3] - RF.RCYawMiddle;
            if (RF.IbusData[9] < 1400 && RF.IbusData[9] > 900)
                RF.RCARM = true;
            else
                RF.RCARM = false;
            if (RF.IbusData[7] > 1700 && RF.IbusData[7] < 2000)
                RF.RC_Auto = true;
            else
                RF.RC_Auto = false;
        }
    });
}

void YJSP_AP::YJSP::ESCThreadREG()
{
    TF.ESCThreading = std::thread([&] {
        while (true)
        {
            EF.SpeedA1 = RF.TotalForward - RF.TotalHorizontal - RF.TotalYaw;
            EF.SpeedA2 = RF.TotalForward + RF.TotalHorizontal + RF.TotalYaw;
            EF.SpeedB1 = RF.TotalForward + RF.TotalHorizontal - RF.TotalYaw;
            EF.SpeedB2 = RF.TotalForward - RF.TotalHorizontal + RF.TotalYaw;
            EF.SpeedA1TO = (EF.SpeedA1 / 500.f) * 3900.f;
            EF.SpeedA2TO = (EF.SpeedA2 / 500.f) * 3900.f;
            EF.SpeedB1TO = (EF.SpeedB1 / 500.f) * 3900.f;
            EF.SpeedB2TO = (EF.SpeedB2 / 500.f) * 3900.f;

            EF.SpeedA1TO = EF.SpeedA1TO > 3900 ? 3900 : EF.SpeedA1TO;
            EF.SpeedA2TO = EF.SpeedA2TO > 3900 ? 3900 : EF.SpeedA2TO;
            EF.SpeedB1TO = EF.SpeedB1TO > 3900 ? 3900 : EF.SpeedB1TO;
            EF.SpeedB2TO = EF.SpeedB2TO > 3900 ? 3900 : EF.SpeedB2TO;

            if (!RF.RCARM)
            {
                if (EF.SpeedA1 > 0)
                {
                    pca9685PWMWrite(DF.fd, 14, 0, (int)abs(EF.SpeedA1TO));
                    pca9685PWMWrite(DF.fd, 15, 0, 0);
                }
                else
                {
                    pca9685PWMWrite(DF.fd, 15, 0, (int)abs(EF.SpeedA1TO));
                    pca9685PWMWrite(DF.fd, 14, 0, 0);
                }

                if (EF.SpeedA2 > 0)
                {
                    pca9685PWMWrite(DF.fd, 3, 0, (int)abs(EF.SpeedA2TO));
                    pca9685PWMWrite(DF.fd, 2, 0, 0);
                }
                else
                {
                    pca9685PWMWrite(DF.fd, 2, 0, (int)abs(EF.SpeedA2TO));
                    pca9685PWMWrite(DF.fd, 3, 0, 0);
                }

                if (EF.SpeedB1 > 0)
                {
                    pca9685PWMWrite(DF.fd, 12, 0, (int)abs(EF.SpeedB1TO));
                    pca9685PWMWrite(DF.fd, 13, 0, 0);
                }
                else
                {
                    pca9685PWMWrite(DF.fd, 13, 0, (int)abs(EF.SpeedB1TO));
                    pca9685PWMWrite(DF.fd, 12, 0, 0);
                }

                if (EF.SpeedB2 > 0)
                {
                    pca9685PWMWrite(DF.fd, 1, 0, (int)abs(EF.SpeedB2TO));
                    pca9685PWMWrite(DF.fd, 0, 0, 0);
                }
                else
                {
                    pca9685PWMWrite(DF.fd, 0, 0, (int)abs(EF.SpeedB2TO));
                    pca9685PWMWrite(DF.fd, 1, 0, 0);
                }
            }
            else
            {
                pca9685PWMWrite(DF.fd, 0, 0, 0);
                pca9685PWMWrite(DF.fd, 1, 0, 0);
                pca9685PWMWrite(DF.fd, 12, 0, 0);
                pca9685PWMWrite(DF.fd, 13, 0, 0);
                pca9685PWMWrite(DF.fd, 2, 0, 0);
                pca9685PWMWrite(DF.fd, 3, 0, 0);
                pca9685PWMWrite(DF.fd, 14, 0, 0);
                pca9685PWMWrite(DF.fd, 15, 0, 0);
            }

            usleep(4000);
        }
    });
}

void YJSP_AP::YJSP::PWMUserInput(int pinBase, int on, int off)
{
    pca9685PWMWrite(DF.fd, pinBase, on, off);
}

void YJSP_AP::YJSP::DEBUGThreadREG()
{
    while (true)
    {
        OnRCDataInComing(RF.IbusData);
        TF.ClearCount++;
        if (TF.ClearCount == 100)
        {
            system("clear");
            TF.ClearCount = 0;
        }
        std::cout << "\033[20A";
        std::cout << "\033[K";
        std::cout << "Accel Roll: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Accel__Roll << "|"
                  << "AccelPitch: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Accel_Pitch << "| \n";
        std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Gryo__Roll << "|"
                  << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Gryo_Pitch << "|"
                  << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Gryo___Yaw << "| \n";
        std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Real__Roll << "|"
                  << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Real_Pitch << "| \n";
        std::cout << "AccelX    : " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Acceleration_X << "cm/s2|"
                  << "AccelY    : " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Acceleration_Y << "cm/s2|"
                  << "AccelZ    : " << std::setw(7) << std::setfill(' ') << (int)SF.myData._uORB_Acceleration_Z << "cm/s2| \n";
        std::cout << "Real_Yaw: " << std::setw(7) << std::setfill(' ') << (int)SF.Real_Yaw << "\n";
        std::cout << "SPPED X   : " << std::setw(7) << std::setfill(' ') << EF.SPEED_X << "cm/s|"
                  << "SPPED Y   : " << std::setw(7) << std::setfill(' ') << EF.SPEED_Y << "cm/s|\n";

        for (int i = 0; i < 10; i++)
        {
            std::cout << RF.IbusData[i] << " ";
        }
        std::cout << "\n";
        std::cout << "RCAUTO     " << RF.RC_Auto << "           \n";
        std::cout << "RCARM      " << RF.RCARM << "           \n";
        std::cout << "RCForward: " << RF.TotalForward << "           \n";
        std::cout << "RCHor:     " << RF.TotalHorizontal << "             \n";
        std::cout << "RCYaw:     " << RF.TotalYaw << "                \n";
        std::cout << "speed A1   " << EF.SpeedA1TO << "               \n";
        std::cout << "speed A2   " << EF.SpeedA2TO << "               \n";
        std::cout << "speed B1   " << EF.SpeedB1TO << "               \n";
        std::cout << "speed B2   " << EF.SpeedB2TO << "               \n";

        usleep(20000);
    }
}

void YJSP_AP::YJSP::PIDCacl(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                            float &last_I_Data, float &last_D_Data,
                            float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
    //P caculate
    outputData = P_Gain * inputDataP;
    //D caculate
    outputData += D_Gain * (inputDataD - last_D_Data);
    last_D_Data = inputDataD;
    //I caculate
    last_I_Data += inputDataD * I_Gain;
    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;
    //P_I_D Mix OUTPUT
    outputData += last_I_Data;
}

double YJSP_AP::YJSP::configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}

double YJSP_AP::YJSP::configWrite(const char *configDir, const char *Target, double obj)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    Configdata[Target] = obj;
    std::ofstream configs;
    configs.open(configDir);
    configs << std::setw(4) << Configdata << std::endl;
    configs.close();
}

void YJSP_AP::YJSP::Servo_ArmGrab1()
{
    for (int i = 250; i <= 270; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 400; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 120; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i >= 140; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }

    for (int i = 270; i <= 350; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i <= 480; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 140; i <= 230; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    PWMUserInput(8, 0, 390);
    delay(1000);
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 350; i >= 270; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 230; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 480; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    PWMUserInput(8, 0, 410);
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 270; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmGrab2()
{
    for (int i = 250; i <= 270; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 400; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 120; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i >= 140; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }

    for (int i = 270; i <= 350; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 140; i <= 230; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 410; i >= 360; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 230; i <= 380; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 400; i >= 90; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 350; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 250; i <= 380; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 380; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 90; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 360; i <= 410; i++)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 270; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmGrab3()
{
    for (int i = 250; i <= 270; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 400; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 120; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i >= 140; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }

    for (int i = 300; i <= 350; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 140; i <= 330; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 410; i <= 490; i++)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 400; i >= 120; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 350; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 250; i <= 270; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 330; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 120; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 490; i >= 410; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 270; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmPlace1()
{
    for (int i = 410; i >= 360; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 400; i >= 370; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 120; i >= 90; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 270; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }

    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    delay(1000);
    for (int i = 90; i <= 170; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 250; i <= 270; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 360; i <= 410; i++)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }

    for (int i = 270; i <= 360; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 170; i <= 210; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 370; i >= 130; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 210; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 130; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 400; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 360; i >= 270; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 270; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmPlace2()
{
    for (int i = 410; i <= 490; i++)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 400; i >= 320; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    delay(1000);
    for (int i = 320; i <= 350; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 250; i <= 380; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 120; i <= 200; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 490; i >= 410; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 350; i >= 130; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }

    for (int i = 380; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 200; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 130; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    delay(1000);
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 400; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmPlace3()
{
    for (int i = 410; i >= 400; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 250; i <= 270; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 400; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 120; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i >= 220; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i <= 500; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 270; i <= 320; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    delay(1000);
    for (int i = 320; i >= 270; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 500; i >= 480; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 400; i <= 410; i++)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 220; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 380; i >= 200; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 250; i <= 370; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i >= 130; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 370; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 200; i <= 400; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 130; i <= 300; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 400; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmGet1()
{
    PWMUserInput(8, 0, 410);
    for (int i = 250; i <= 370; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i >= 200; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 400; i >= 130; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 370; i >= 350; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 410; i >= 400; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 200; i <= 480; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 130; i <= 240; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 350; i >= 250; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 240; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 480; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmGet2()
{
    PWMUserInput(8, 0, 410);
    for (int i = 250; i <= 370; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i >= 200; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 400; i >= 130; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 200; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 130; i <= 330; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 410; i <= 490; i++)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 400; i >= 130; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 370; i >= 220; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 220; i <= 250; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }

    for (int i = 130; i <= 300; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 330; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 490; i >= 410; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 120; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmGet3()
{
    PWMUserInput(8, 0, 410);
    for (int i = 250; i <= 370; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i >= 200; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 400; i >= 130; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 0; i <= 100; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 200; i <= 350; i++)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 130; i <= 380; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }
    for (int i = 410; i >= 355; i--)
    {
        delay(5);
        PWMUserInput(8, 0, i);
    }
    for (int i = 350; i >= 110; i--)
    {
        delay(5);
        PWMUserInput(6, 0, i);
    }
    for (int i = 370; i >= 220; i--)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }

    for (int i = 100; i <= 150; i++)
    {
        delay(5);
        PWMUserInput(4, 0, i);
    }
    for (int i = 220; i <= 250; i++)
    {
        delay(5);
        PWMUserInput(5, 0, i);
    }
    for (int i = 110; i <= 300; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 380; i >= 300; i--)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }

    for (int i = 355; i <= 410; i++)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }

    for (int i = 300; i <= 400; i++)
    {
        delay(5);
        PWMUserInput(7, 0, i);
    }

    for (int i = 300; i >= 120; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
}
