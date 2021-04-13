#include "YJSP_Schedule.hpp"
int YJSP_AP::YJSP::YJSP_Init()
{
    wiringPiSetup();
    DF.fd = pca9685Setup(65, 0x40, 50);
    DF.myIbusDevice = new Ibus(DF.RCDeviceInfo);
    DF.MPUDevice = new RPiMPU9250(1, false, 1, 0x68, TF.TimeMax, 0);
    DF.myVL53L1X.begin("/dev/i2c-1", 0x29);
    DF.MYVL53L1X.begin("/dev/i2c-0", 0x29);
    DF.myVL53L1X.Setmode('L');
    DF.MYVL53L1X.Setmode('L');
    SF.AccelCaliData[MPUAccelCaliX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Cali");
    SF.AccelCaliData[MPUAccelCaliY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Cali");
    SF.AccelCaliData[MPUAccelCaliZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Cali");
    SF.AccelCaliData[MPUAccelScalX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Scal");
    SF.AccelCaliData[MPUAccelScalY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Scal");
    SF.AccelCaliData[MPUAccelScalZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Scal");
    PF.PIDYawPGain = configSettle("../MPU9250Car.json", "_Car_PIDYawPGain");
    PF.PIDYawIGain = configSettle("../MPU9250Car.json", "_Car_PIDYawIGain");
    PF.PIDYawDGain = configSettle("../MPU9250Car.json", "_Car_PIDYawDGain");
    PF.PIDForwardPGain = configSettle("../MPU9250Car.json", "_Car_PIDForwardPGain");
    PF.PIDForwardIGain = configSettle("../MPU9250Car.json", "_Car_PIDForwardIGain");
    PF.PIDForwardDGain = configSettle("../MPU9250Car.json", "_Car_PIDForwardDGain");
    PF.PIDHorPGain = configSettle("../MPU9250Car.json", "_Car_PIDHorizontalPGain");
    PF.PIDHorIGain = configSettle("../MPU9250Car.json", "_Car_PIDHorizontalIGain");
    PF.PIDHorDGain = configSettle("../MPU9250Car.json", "_Car_PIDHorizontalDGain");
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
            EF.SPEED_X = EF.SPEED_X + (int)SF.myData._uORB_Acceleration_X * (TF.TimeMax / 1000000.f);
            EF.SPEED_Y = EF.SPEED_Y + (int)SF.myData._uORB_Acceleration_Y * (TF.TimeMax / 1000000.f);
            EF.SPEED_X = EF.SPEED_X * 0.985 + (EF.SPEED_X_EKF) * (1.f - 0.985);
            EF.SPEED_Y = EF.SPEED_Y * 0.985 + (EF.SPEED_Y) * (1.f - 0.985);

            double EKFInput[2] = {EF.SPEED_X, (double)SF.speed_x};

            DF.EKFIMS.step(EKFInput);
            EF.SPEED_X_EKF = DF.EKFIMS.getX(0);

            {
                float YawInput;
                float ForInput;
                float HorInput;

                if (RF.RC_Auto)
                {
                    RF.TotalForward = RF.Auto_Forward;
                    RF.TotalHorizontal = RF.Auto_Horizontal;
                    YawInput = (RF.Auto_Yaw + SF.myData._uORB_Gryo___Yaw) + SF.Real_Yaw * 15.f - RF.InputYaw * 15.f;
                }
                else
                {
                    RF.TotalForward = RF.RCForward;
                    RF.TotalHorizontal = RF.RCHorizontal;
                    // ForInput = EF.SPEED_X_EKF - (RF.RCForward / 500.f) * 500.f;
                    // HorInput = EF.SPEED_Y - (RF.RCHorizontal / 500.f) * 200.f;
                    YawInput = (RF.RCYaw + SF.myData._uORB_Gryo___Yaw * 5);
                }
                PF.TotalYawIFilter += (YawInput - PF.TotalYawIFilter) * 0.92;
                PF.TotalYawDFilter += (YawInput - PF.TotalYawDFilter) * 0.985;

                PIDCacl(YawInput, PF.TotalYawIFilter, PF.TotalYawDFilter, RF.TotalYaw, PF.PIDYawLastIData,
                        PF.PIDYawLastDData, PF.PIDYawPGain, PF.PIDYawIGain, PF.PIDYawDGain, 300.f);

                // PIDCacl(ForInput, ForInput, ForInput, RF.TotalForward, PF.PIDForwardLastIData,
                //         PF.PIDForwardLastDData, PF.PIDForwardPGain, PF.PIDForwardIGain, PF.PIDForwardDGain, 500.f);

                // PIDCacl(HorInput, HorInput, HorInput, RF.TotalHorizontal, PF.PIDHorLastIData,
                //         PF.PIDHorLastDData, PF.PIDHorPGain, PF.PIDHorIGain, PF.PIDHorDGain, 100.f);
            }

            EF.SpeedA1 = RF.TotalForward + RF.TotalHorizontal - RF.TotalYaw;
            EF.SpeedA2 = RF.TotalForward - RF.TotalHorizontal + RF.TotalYaw;
            EF.SpeedB1 = RF.TotalForward - RF.TotalHorizontal - RF.TotalYaw;
            EF.SpeedB2 = RF.TotalForward + RF.TotalHorizontal + RF.TotalYaw;
            EF.SpeedA1TO = (EF.SpeedA1 / 500.f) * 3900.f;
            EF.SpeedA2TO = (EF.SpeedA2 / 500.f) * 3900.f;
            EF.SpeedB1TO = (EF.SpeedB1 / 500.f) * 3900.f;
            EF.SpeedB2TO = (EF.SpeedB2 / 500.f) * 3900.f;

            EF.SpeedA1TO = EF.SpeedA1TO > 3900 ? 3900 : EF.SpeedA1TO;
            EF.SpeedA2TO = EF.SpeedA2TO > 3900 ? 3900 : EF.SpeedA2TO;
            EF.SpeedB1TO = EF.SpeedB1TO > 3900 ? 3900 : EF.SpeedB1TO;
            EF.SpeedB2TO = EF.SpeedB2TO > 3900 ? 3900 : EF.SpeedB2TO;

            TF.TimeEnd = micros();
            if (TF.TimeMax < ((TF.TimeEnd - TF.TimeStart) + TF.TimeNext) || (TF.TimeNext) < 0)
                usleep(50);
            else
                usleep(TF.TimeMax - (TF.TimeEnd - TF.TimeStart) - TF.TimeNext);
            TF.TimeEnd = micros();
        }
    });
}

void YJSP_AP::YJSP::PositionThreadREG()
{
    TF.PosThreading = std::thread([&] {
        int timeStart;
        int timeEnd;
        DF.myVL53L1X.startMeasurement(0);
        while (true)
        {
            if (DF.myVL53L1X.newDataReady())
            {
                timeStart = micros();
                SF.dataLast = SF.distance_X;
                SF.distance_X = (DF.myVL53L1X.getDistance() / 10.f);
                if (SF.distance_X < 0)
                {
                    SF.distance_X = SF.dataLast;
                }
                SF.speed_x = (SF.distance_X - SF.dataLast) / ((timeStart - timeEnd) / 1000000.f);
                DF.myVL53L1X.startMeasurement(0);
                timeEnd = micros();
            }
            usleep(10000);
        }
    });

    TF.NewPosThreading = std::thread([&] {
        int timeStart;
        int timeEnd;
        DF.MYVL53L1X.startMeasurement(0);
        while (true)
        {
            if (DF.MYVL53L1X.newDataReady())
            {
                timeStart = micros();
                SF.dataLast = SF.distance_Y;
                SF.distance_Y = (DF.MYVL53L1X.getDistance() / 10.f);
                if (SF.distance_Y < 0)
                {
                    SF.distance_Y = SF.dataLast;
                }
                SF.speed_y = (SF.distance_Y - SF.dataLast) / ((timeStart - timeEnd) / 1000000.f);
                DF.MYVL53L1X.startMeasurement(0);
                timeEnd = micros();
            }
            usleep(100000);
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

            usleep(6000);
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
        std::cout << "SPPED X   : " << std::setw(7) << std::setfill(' ') << (int)EF.SPEED_X << "cm/s|"
                  << "SPEED XEKF: " << std::setw(7) << std::setfill(' ') << (int)EF.SPEED_X_EKF << "cm/s|"
                  << "SPPED Y   : " << std::setw(7) << std::setfill(' ') << (int)EF.SPEED_Y << "cm/s|\n";
        std::cout << "speed x   : " << std::setw(7) << std::setfill(' ') << (int)(SF.speed_x / 10) << "cm/s|\n";
        std::cout << "speed y   : " << std::setw(7) << std::setfill(' ') << (int)(SF.speed_y / 10) << "cm/s|\n";
        std::cout << "Distance_X  : " << std::setw(7) << std::setfill(' ') << SF.distance_X << " cm\n";
        std::cout << "Distance_Y  : " << std::setw(7) << std::setfill(' ') << SF.distance_Y << " cm\n";

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
    for (int i = 420; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 450; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 390; i >= 230; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }

    for (int i = 300; i <= 360; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 200; i >= 150; i--)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 230; i <= 390; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 360; i <= 450; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 330; i <= 420; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 400; i >= 350; i--)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
    for (int i = 420; i <= 440; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 450; i <= 460; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 390; i >= 340; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }

    for (int i = 150; i <= 200; i++)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 440; i >= 420; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 460; i >= 450; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 340; i <= 390; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 350; i <= 400; i++)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmGrab2()
{
    for (int i = 420; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 450; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 390; i >= 230; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }

    for (int i = 300; i <= 360; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 200; i >= 150; i--)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 230; i <= 380; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 360; i <= 450; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 330; i <= 420; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 400; i <= 480; i++)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
    for (int i = 380; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 150; i <= 200; i++)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 320; i <= 390; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 480; i >= 400; i--)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmGrab3()
{
    for (int i = 420; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 450; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 390; i >= 230; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }

    for (int i = 300; i <= 360; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 200; i >= 150; i--)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }

    for (int i = 360; i >= 100; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 320; i >= 230; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }

    for (int i = 150; i <= 200; i++)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }

    for (int i = 230; i <= 320; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }

    for (int i = 100; i <= 120; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 230; i <= 305; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }

    for (int i = 120; i <= 300; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 305; i <= 390; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }

    for (int i = 300; i <= 450; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }

    for (int i = 320; i <= 420; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmPlace1()
{
    for (int i = 390; i >= 360; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 400; i >= 350; i--)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
    for (int i = 450; i <= 475; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 420; i <= 425; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 360; i >= 340; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 200; i >= 150; i--)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 340; i <= 360; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 350; i <= 405; i++)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
    for (int i = 425; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 360; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 475; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 320; i >= 210; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i <= 380; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i >= 180; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 150; i <= 200; i++)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 180; i <= 390; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 380; i <= 450; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 210; i <= 420; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmPlace2()
{
    for (int i = 390; i >= 360; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 405; i <= 480; i++)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
    for (int i = 450; i <= 470; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 420; i >= 380; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 360; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 200; i >= 150; i--)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 320; i <= 360; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 480; i >= 405; i--)
    {
        delay(10);
        PWMUserInput(8, 0, i);
    }
    for (int i = 380; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 360; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 470; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 320; i >= 210; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 300; i <= 380; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i >= 180; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 150; i <= 200; i++)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 180; i <= 390; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 380; i <= 450; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 210; i <= 420; i++)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
}

void YJSP_AP::YJSP::Servo_ArmPlace3()
{
    for (int i = 420; i >= 320; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 390; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 450; i >= 300; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 300; i >= 245; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 300; i >= 90; i--)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 320; i >= 250; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 200; i >= 150; i--)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 245; i >= 180; i--)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 250; i >= 210; i--)
    {
        delay(10);
        PWMUserInput(5, 0, i);
    }
    for (int i = 90; i <= 380; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 150; i <= 200; i++)
    {
        delay(10);
        PWMUserInput(4, 0, i);
    }
    for (int i = 180; i <= 390; i++)
    {
        delay(10);
        PWMUserInput(7, 0, i);
    }
    for (int i = 380; i <= 450; i++)
    {
        delay(10);
        PWMUserInput(6, 0, i);
    }
    for (int i = 210; i <= 420; i++)
    {
        delay(10);
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
